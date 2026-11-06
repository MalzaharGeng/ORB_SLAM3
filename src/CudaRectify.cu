#include "CudaRectify.h"
#include <cuda_runtime.h>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <stdexcept>
namespace {
void check(cudaError_t e) {
    if(e!=cudaSuccess)throw std::runtime_error(cudaGetErrorString(e));
}
__device__ int sample(const unsigned char* input,int pitch,int roiX,int roiY,
                      int width,int height,int x,int y) {
    return (x>=0 && x<width && y>=0 && y<height)?input[(roiY+y)*pitch+roiX+x]:0;
}
__global__ void remapFixed(const unsigned char* input,int pitch,int roiX,int roiY,
                           int width,int height,const short2* map,
                           const unsigned short* fractions,unsigned char* output,
                           int outputWidth,int outputHeight) {
    const int x=blockIdx.x*blockDim.x+threadIdx.x;
    const int y=blockIdx.y*blockDim.y+threadIdx.y;
    if(x>=outputWidth || y>=outputHeight)return;
    const int at=y*outputWidth+x;
    const short2 xy=map[at];
    const int f=fractions[at],fx=f&31,fy=(f>>5)&31;
    const int value=sample(input,pitch,roiX,roiY,width,height,xy.x,xy.y)*(32-fx)*(32-fy)
        +sample(input,pitch,roiX,roiY,width,height,xy.x+1,xy.y)*fx*(32-fy)
        +sample(input,pitch,roiX,roiY,width,height,xy.x,xy.y+1)*(32-fx)*fy
        +sample(input,pitch,roiX,roiY,width,height,xy.x+1,xy.y+1)*fx*fy;
    output[at]=static_cast<unsigned char>((value+512)>>10);
}
}
namespace ORB_SLAM3 {
struct CudaStereoRectifier::State {
    cudaStream_t stream=nullptr;
    unsigned char* image=nullptr;
    unsigned char* output=nullptr;
    short2* maps=nullptr;
    unsigned short* fractions=nullptr;
    cv::Size inputSize,mapSize;
    cv::Rect roi[2];
    const unsigned char* mapPointers[4]={};
    bool ready=false,disabled=false;
    std::uint64_t calls=0;
    ~State() {
        if(stream)cudaStreamSynchronize(stream);
        if(image)cudaFree(image);
        if(output)cudaFree(output);
        if(maps)cudaFree(maps);
        if(fractions)cudaFree(fractions);
        if(stream)cudaStreamDestroy(stream);
    }
};
CudaStereoRectifier::CudaStereoRectifier():state_(std::make_unique<State>()) {}
CudaStereoRectifier::~CudaStereoRectifier()=default;
std::uint64_t CudaStereoRectifier::Calls()const {return state_->calls;}
bool CudaStereoRectifier::Remap(const cv::Mat& gray,const cv::Rect& leftRoi,const cv::Rect& rightRoi,
    const cv::Mat& leftMap,const cv::Mat& leftFraction,const cv::Mat& rightMap,const cv::Mat& rightFraction,
    cv::Mat& left,cv::Mat& right) {
    auto& s=*state_;
    const char* backend=std::getenv("ORB_SLAM3_RECTIFY_BACKEND");
    if(!backend || std::strcmp(backend,"cuda") || s.disabled)return false;
    try {
        const cv::Mat* map[2]={&leftMap,&rightMap};
        const cv::Mat* fraction[2]={&leftFraction,&rightFraction};
        const cv::Rect roi[2]={leftRoi,rightRoi};
        if(gray.type()!=CV_8UC1 || gray.empty())throw std::runtime_error("expected grayscale input");
        for(int eye=0;eye<2;++eye) {
            if(map[eye]->type()!=CV_16SC2 || fraction[eye]->type()!=CV_16UC1 ||
               map[eye]->size()!=leftMap.size() || fraction[eye]->size()!=leftMap.size() ||
               (roi[eye]&cv::Rect(0,0,gray.cols,gray.rows))!=roi[eye] || roi[eye].empty())
                throw std::runtime_error("invalid fixed-point maps or crop");
        }
        check(cudaSetDevice(0));
        const auto n=leftMap.total();
        if(!s.ready) {
            check(cudaStreamCreateWithFlags(&s.stream,cudaStreamNonBlocking));
            check(cudaMalloc(&s.image,gray.total()));
            check(cudaMalloc(&s.output,2*n));
            check(cudaMalloc(&s.maps,2*n*sizeof(short2)));
            check(cudaMalloc(&s.fractions,2*n*sizeof(unsigned short)));
            for(int eye=0;eye<2;++eye) {
                check(cudaMemcpy2DAsync(s.maps+eye*n,leftMap.cols*sizeof(short2),
                    map[eye]->data,map[eye]->step,leftMap.cols*sizeof(short2),leftMap.rows,
                    cudaMemcpyHostToDevice,s.stream));
                check(cudaMemcpy2DAsync(s.fractions+eye*n,leftMap.cols*sizeof(unsigned short),
                    fraction[eye]->data,fraction[eye]->step,leftMap.cols*sizeof(unsigned short),leftMap.rows,
                    cudaMemcpyHostToDevice,s.stream));
                s.roi[eye]=roi[eye];s.mapPointers[eye*2]=map[eye]->data;s.mapPointers[eye*2+1]=fraction[eye]->data;
            }
            s.inputSize=gray.size();s.mapSize=leftMap.size();s.ready=true;
        } else {
            if(gray.size()!=s.inputSize || leftMap.size()!=s.mapSize)
                throw std::runtime_error("rectifier geometry changed");
            for(int eye=0;eye<2;++eye)
                if(roi[eye]!=s.roi[eye] || map[eye]->data!=s.mapPointers[eye*2] ||
                   fraction[eye]->data!=s.mapPointers[eye*2+1])
                    throw std::runtime_error("rectifier maps changed");
        }
        check(cudaMemcpy2DAsync(s.image,gray.cols,gray.data,gray.step,gray.cols,gray.rows,
            cudaMemcpyHostToDevice,s.stream));
        cv::Mat* out[2]={&left,&right};
        for(int eye=0;eye<2;++eye) {
            out[eye]->create(s.mapSize,CV_8UC1);
            remapFixed<<<dim3((s.mapSize.width+31)/32,(s.mapSize.height+7)/8),dim3(32,8),0,s.stream>>>(
                s.image,gray.cols,roi[eye].x,roi[eye].y,roi[eye].width,roi[eye].height,
                s.maps+eye*n,s.fractions+eye*n,s.output+eye*n,s.mapSize.width,s.mapSize.height);
            check(cudaGetLastError());
            check(cudaMemcpy2DAsync(out[eye]->data,out[eye]->step,s.output+eye*n,s.mapSize.width,
                s.mapSize.width,s.mapSize.height,cudaMemcpyDeviceToHost,s.stream));
        }
        check(cudaStreamSynchronize(s.stream));
        if(!s.calls)std::cout<<"[Rectify] CUDA fixed-point remap active; CPU-compatible interpolation"<<std::endl;
        ++s.calls;return true;
    } catch(const std::exception& e) {
        if(s.stream)cudaStreamSynchronize(s.stream);
        s.disabled=true;
        std::cerr<<"[Rectify] CUDA unavailable: "<<e.what()<<"; CPU fallback"<<std::endl;
        return false;
    }
}
}
