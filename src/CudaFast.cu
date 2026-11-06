#include "CudaFast.h"
#include <cuda_runtime.h>
#include <atomic>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
namespace {
__device__ __constant__ int dx[16]={0,1,2,3,3,3,2,1,0,-1,-2,-3,-3,-3,-2,-1};
__device__ __constant__ int dy[16]={3,3,2,1,0,-1,-2,-3,-3,-3,-2,-1,0,1,2,3};
__global__ void scoreFast9(const unsigned char* image,int width,int height,unsigned char* output,int threshold) {
    const int x=blockIdx.x*blockDim.x+threadIdx.x,y=blockIdx.y*blockDim.y+threadIdx.y;
    if(x>=width || y>=height)return;
    int result=0;
    if(x>=3 && x<width-3 && y>=3 && y<height-3) {
        const int center=image[y*width+x];
        short diff[25];
        unsigned brightMask=0,darkMask=0;
        #pragma unroll
        for(int i=0;i<16;++i) {
            const int d=int(image[(y+dy[i])*width+x+dx[i]])-center;diff[i]=d;
            if(d>threshold)brightMask|=1u<<i;
            if(-d>threshold)darkMask|=1u<<i;
        }
        if(__popc(brightMask)>=9 || __popc(darkMask)>=9) {
            brightMask|=brightMask<<16;darkMask|=darkMask<<16;
            #pragma unroll
            for(int i=16;i<25;++i)diff[i]=diff[i-16];
            #pragma unroll
            for(int start=0;start<16;++start) {
                const bool brightRun=((brightMask>>start)&511u)==511u;
                const bool darkRun=((darkMask>>start)&511u)==511u;
                if(brightRun || darkRun) {
                    int strength=255;
                    #pragma unroll
                    for(int i=0;i<9;++i)strength=min(strength,brightRun?int(diff[start+i]):-int(diff[start+i]));
                    result=max(result,strength-1);
                }
            }
        }
    }
    output[y*width+x]=static_cast<unsigned char>(result);
}

void checked(cudaError_t e) {if(e!=cudaSuccess)throw std::runtime_error(cudaGetErrorString(e));}
struct State {
    bool busy=false;unsigned char *image=nullptr,*score=nullptr;std::size_t capacity=0;
    cudaStream_t stream=nullptr;
    ~State() {if(stream)cudaStreamDestroy(stream);if(image)cudaFree(image);if(score)cudaFree(score);}
};
struct Pool {
    std::mutex mutex;std::vector<std::unique_ptr<State>> states;
    std::atomic<bool> disabled{false};std::atomic<std::uint64_t> calls{0};
};
Pool& pool() {static Pool value;return value;}
struct Lease {
    Pool& owner;State* state=nullptr;
    explicit Lease(Pool& p):owner(p) {
        std::lock_guard<std::mutex> lock(owner.mutex);
        for(auto& s:owner.states)if(!s->busy){state=s.get();break;}
        if(!state){owner.states.push_back(std::make_unique<State>());state=owner.states.back().get();}
        state->busy=true;
    }
    ~Lease(){std::lock_guard<std::mutex> lock(owner.mutex);state->busy=false;}
};
}
namespace ORB_SLAM3 {
std::uint64_t CudaFastCalls() {return pool().calls.load();}
bool CudaFastScores(const std::vector<cv::Mat>& pyramid,std::vector<cv::Mat>& scores,int minimumThreshold) {
    const char* backend=std::getenv("ORB_SLAM3_FAST_BACKEND");
    if(!backend || std::strcmp(backend,"cuda") || pool().disabled || minimumThreshold<1)return false;
    auto& owner=pool();Lease lease(owner);auto& state=*lease.state;
    try {
        checked(cudaSetDevice(0));
        if(!state.stream)checked(cudaStreamCreateWithFlags(&state.stream,cudaStreamNonBlocking));
        std::size_t total=0;
        for(const auto& m:pyramid) {
            if(m.type()!=CV_8UC1 || m.cols<7 || m.rows<7)throw std::runtime_error("invalid pyramid");
            total+=m.total();
        }
        if(total>state.capacity) {
            if(state.image){checked(cudaFree(state.image));state.image=nullptr;}
            if(state.score){checked(cudaFree(state.score));state.score=nullptr;}
            checked(cudaMalloc(&state.image,total));checked(cudaMalloc(&state.score,total));state.capacity=total;
        }
        scores.resize(pyramid.size());std::size_t offset=0;
        for(std::size_t i=0;i<pyramid.size();++i) {
            const auto& m=pyramid[i];scores[i].create(m.size(),CV_8UC1);
            checked(cudaMemcpy2DAsync(state.image+offset,m.cols,m.data,m.step,m.cols,m.rows,cudaMemcpyHostToDevice,state.stream));
            scoreFast9<<<dim3((m.cols+31)/32,(m.rows+7)/8),dim3(32,8),0,state.stream>>>(state.image+offset,m.cols,m.rows,state.score+offset,minimumThreshold);
            checked(cudaGetLastError());
            checked(cudaMemcpy2DAsync(scores[i].data,scores[i].step,state.score+offset,m.cols,m.cols,m.rows,cudaMemcpyDeviceToHost,state.stream));
            offset+=m.total();
        }
        checked(cudaStreamSynchronize(state.stream));++owner.calls;return true;
    } catch(const std::exception& e) {
        if(state.stream)cudaStreamSynchronize(state.stream);
        if(!owner.disabled.exchange(true))std::cerr<<"[ORB] CUDA FAST unavailable: "<<e.what()<<"; CPU fallback"<<std::endl;
        scores.clear();return false;
    }
}
void CudaFastCell(const cv::Mat& scores,const cv::Rect& roi,int threshold,std::vector<cv::KeyPoint>& keypoints) {
    keypoints.clear();
    const int x0=roi.x+3,y0=roi.y+3,x1=roi.x+roi.width-3,y1=roi.y+roi.height-3;
    for(int y=y0;y<y1;++y) {
        const auto* row=scores.ptr<unsigned char>(y);
        for(int x=x0;x<x1;++x) {
            const int score=row[x];if(score<threshold)continue;
            bool keep=true;
            for(int dy=-1;dy<=1 && keep;++dy) {
                const int ny=y+dy;if(ny<y0 || ny>=y1)continue;
                const auto* neighbor=scores.ptr<unsigned char>(ny);
                for(int dx=-1;dx<=1;++dx) {
                    if(!dx && !dy)continue;
                    const int nx=x+dx;if(nx<x0 || nx>=x1)continue;
                    if(score<=neighbor[nx]){keep=false;break;}
                }
            }
            if(keep)keypoints.emplace_back(float(x-roi.x),float(y-roi.y),7.f,-1.f,float(score));
        }
    }
}
}
