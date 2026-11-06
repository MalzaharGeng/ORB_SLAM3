#include "Frame.h"
#include "ORBextractor.h"
#include "ORBmatcher.h"
#include "Pinhole.h"
#include "StereoPatch.h"
#include <opencv2/core.hpp>
#include <chrono>
#include <cstring>
#include <climits>
#include <filesystem>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <stdexcept>
using namespace ORB_SLAM3;
using namespace std;
class ReferenceFrame:public Frame {
public:
    explicit ReferenceFrame(const Frame& f):Frame(f){}
void ComputeReference()
{
    mvuRight = vector<float>(N,-1.0f);
    mvDepth = vector<float>(N,-1.0f);

    const int thOrbDist = (ORBmatcher::TH_HIGH+ORBmatcher::TH_LOW)/2;

    const int nRows = mpORBextractorLeft->mvImagePyramid[0].rows;

    //Assign keypoints to row table
    vector<vector<size_t> > vRowIndices(nRows,vector<size_t>());

    for(int i=0; i<nRows; i++)
        vRowIndices[i].reserve(200);

    const int Nr = mvKeysRight.size();

    for(int iR=0; iR<Nr; iR++)
    {
        const cv::KeyPoint &kp = mvKeysRight[iR];
        const float &kpY = kp.pt.y;
        const float r = 2.0f*mvScaleFactors[mvKeysRight[iR].octave];
        const int maxr = ceil(kpY+r);
        const int minr = floor(kpY-r);

        for(int yi=minr;yi<=maxr;yi++)
            vRowIndices[yi].push_back(iR);
    }

    // Set limits for search
    const float minZ = mb;
    const float minD = 0;
    const float maxD = mbf/minZ;

    // For each left keypoint search a match in the right image
    vector<pair<int, int> > vDistIdx;
    vDistIdx.reserve(N);

    for(int iL=0; iL<N; iL++)
    {
        const cv::KeyPoint &kpL = mvKeys[iL];
        const int &levelL = kpL.octave;
        const float &vL = kpL.pt.y;
        const float &uL = kpL.pt.x;

        const vector<size_t> &vCandidates = vRowIndices[vL];

        if(vCandidates.empty())
            continue;

        const float minU = uL-maxD;
        const float maxU = uL-minD;

        if(maxU<0)
            continue;

        int bestDist = ORBmatcher::TH_HIGH;
        size_t bestIdxR = 0;

        const cv::Mat &dL = mDescriptors.row(iL);

        // Compare descriptor to right keypoints
        for(size_t iC=0; iC<vCandidates.size(); iC++)
        {
            const size_t iR = vCandidates[iC];
            const cv::KeyPoint &kpR = mvKeysRight[iR];

            if(kpR.octave<levelL-1 || kpR.octave>levelL+1)
                continue;

            const float &uR = kpR.pt.x;

            if(uR>=minU && uR<=maxU)
            {
                const cv::Mat &dR = mDescriptorsRight.row(iR);
                const int dist = ORBmatcher::DescriptorDistance(dL,dR);

                if(dist<bestDist)
                {
                    bestDist = dist;
                    bestIdxR = iR;
                }
            }
        }

        // Subpixel match by correlation
        if(bestDist<thOrbDist)
        {
            // coordinates in image pyramid at keypoint scale
            const float uR0 = mvKeysRight[bestIdxR].pt.x;
            const float scaleFactor = mvInvScaleFactors[kpL.octave];
            const float scaleduL = round(kpL.pt.x*scaleFactor);
            const float scaledvL = round(kpL.pt.y*scaleFactor);
            const float scaleduR0 = round(uR0*scaleFactor);

            // sliding window search
            const int w = 5;
            cv::Mat IL = mpORBextractorLeft->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduL-w,scaleduL+w+1);

            int bestDist = INT_MAX;
            int bestincR = 0;
            const int L = 5;
            vector<float> vDists;
            vDists.resize(2*L+1);

            const float iniu = scaleduR0+L-w;
            const float endu = scaleduR0+L+w+1;
            if(iniu<0 || endu >= mpORBextractorRight->mvImagePyramid[kpL.octave].cols)
                continue;

            for(int incR=-L; incR<=+L; incR++)
            {
                cv::Mat IR = mpORBextractorRight->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduR0+incR-w,scaleduR0+incR+w+1);

                float dist = cv::norm(IL,IR,cv::NORM_L1);
                if(dist<bestDist)
                {
                    bestDist =  dist;
                    bestincR = incR;
                }

                vDists[L+incR] = dist;
            }

            if(bestincR==-L || bestincR==L)
                continue;

            // Sub-pixel match (Parabola fitting)
            const float dist1 = vDists[L+bestincR-1];
            const float dist2 = vDists[L+bestincR];
            const float dist3 = vDists[L+bestincR+1];

            const float deltaR = (dist1-dist3)/(2.0f*(dist1+dist3-2.0f*dist2));

            if(deltaR<-1 || deltaR>1)
                continue;

            // Re-scaled coordinate
            float bestuR = mvScaleFactors[kpL.octave]*((float)scaleduR0+(float)bestincR+deltaR);

            float disparity = (uL-bestuR);

            if(disparity>=minD && disparity<maxD)
            {
                if(disparity<=0)
                {
                    disparity=0.01;
                    bestuR = uL-0.01;
                }
                mvDepth[iL]=mbf/disparity;
                mvuRight[iL] = bestuR;
                vDistIdx.push_back(pair<int,int>(bestDist,iL));
            }
        }
    }

    sort(vDistIdx.begin(),vDistIdx.end());
    const float median = vDistIdx[vDistIdx.size()/2].first;
    const float thDist = 1.5f*1.4f*median;

    for(int i=vDistIdx.size()-1;i>=0;i--)
    {
        if(vDistIdx[i].first<thDist)
            break;
        else
        {
            mvuRight[vDistIdx[i].second]=-1;
            mvDepth[vDistIdx[i].second]=-1;
        }
    }
}

};

static void require(bool ok,const char* reason) {if(!ok)throw std::runtime_error(reason);}
int main(int argc,char** argv) {
    cv::RNG rng(12345);
    cv::Mat left(480,640,CV_8UC1),right(480,640,CV_8UC1);
    rng.fill(left,cv::RNG::UNIFORM,0,256);rng.fill(right,cv::RNG::UNIFORM,0,256);
    for(int n=0;n<3000;++n) {
        int x=rng.uniform(0,620),y=rng.uniform(0,460),xr=rng.uniform(0,620),yr=rng.uniform(0,460);
        auto a=left(cv::Rect(x,y,11,11)),b=right(cv::Rect(xr,yr,11,11));
        require(stereo_detail::PatchL1(a.ptr<std::uint8_t>(),a.step,b.ptr<std::uint8_t>(),b.step)==int(cv::norm(a,b,cv::NORM_L1)),"patch L1 differs from OpenCV");
    }
    cv::Mat zero(11,11,CV_8UC1,cv::Scalar(0)),max(11,11,CV_8UC1,cv::Scalar(255));
    require(stereo_detail::PatchL1(zero.ptr<std::uint8_t>(),zero.step,max.ptr<std::uint8_t>(),max.step)==30855,"L1 overflow");
    cv::Mat k=(cv::Mat_<float>(3,3)<<400,0,320,0,400,240,0,0,1),dist=cv::Mat::zeros(4,1,CV_32F);
    Pinhole camera({400,400,320,240});
    ORBextractor le(1000,1.2,8,20,7),re(1000,1.2,8,20,7);
    std::vector<double> referenceMs,optimizedMs;
    int valid=0;
    for(int trial=0;trial<16;++trial) {
        rng.fill(left,cv::RNG::UNIFORM,0,256);
        right.setTo(0);int disparity=5+trial%15;
        left(cv::Rect(disparity,0,640-disparity,480)).copyTo(right(cv::Rect(0,0,640-disparity,480)));
        Frame actual(left,right,trial*.1,&le,&re,nullptr,k,dist,20,2.5,&camera);
        ReferenceFrame oracle(actual);
        auto t=std::chrono::steady_clock::now();oracle.ComputeReference();
        referenceMs.push_back(std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-t).count());
        t=std::chrono::steady_clock::now();actual.ComputeStereoMatches();
        optimizedMs.push_back(std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-t).count());
        require(actual.mvDepth.size()==oracle.mvDepth.size(),"depth size");
        require(std::memcmp(actual.mvDepth.data(),oracle.mvDepth.data(),actual.mvDepth.size()*sizeof(float))==0,"depth changed");
        require(std::memcmp(actual.mvuRight.data(),oracle.mvuRight.data(),actual.mvuRight.size()*sizeof(float))==0,"right subpixel coordinates changed");
        valid+=std::count_if(actual.mvDepth.begin(),actual.mvDepth.end(),[](float z){return z>0;});
    }
    require(valid>1000,"oracle fixture has too few matches");
    // Previously N>0 with no right descriptors indexed an empty median vector.
    right.setTo(0);Frame unmatched(left,right,2.,&le,&re,nullptr,k,dist,20,2.5,&camera);
    require(unmatched.N>0,"unmatched fixture has no left features");
    require(std::all_of(unmatched.mvDepth.begin(),unmatched.mvDepth.end(),[](float z){return z==-1;}),"unmatched depth not invalid");
    std::sort(referenceMs.begin(),referenceMs.end());std::sort(optimizedMs.begin(),optimizedMs.end());
    int recorded=0;
    if(argc==3) {
        cv::FileStorage fs(argv[1],cv::FileStorage::READ);
        cv::Mat mapX[2],mapY[2],matrix,p2;
        for(int eye=0;eye<2;++eye) {
            std::string n=std::to_string(eye+1);cv::Mat a,d,r,p;
            fs["K"+n]>>a;fs["D"+n]>>d;fs["R"+n]>>r;fs["P"+n]>>p;
            cv::initUndistortRectifyMap(a,d,r,p,cv::Size(1280,960),CV_16SC2,mapX[eye],mapY[eye]);
            if(!eye)matrix=p(cv::Rect(0,0,3,3)).clone();else p2=p;
        }
        matrix.convertTo(matrix,CV_32F);
        Pinhole measured({matrix.at<float>(0,0),matrix.at<float>(1,1),matrix.at<float>(0,2),matrix.at<float>(1,2)});
        std::vector<std::filesystem::path> files;
        for(auto& f:std::filesystem::directory_iterator(std::filesystem::path(argv[2])/"frames"))files.push_back(f.path());
        std::sort(files.begin(),files.end());
        for(size_t idx=0;idx<files.size() && recorded<20;idx+=std::max(size_t(1),files.size()/20)) {
            cv::Mat image=cv::imread(files[idx].string(),0),l,r;
            if(image.size()!=cv::Size(2720,960))continue;
            cv::remap(image(cv::Rect(160,0,1280,960)),l,mapX[0],mapY[0],cv::INTER_LINEAR);
            cv::remap(image(cv::Rect(1440,0,1280,960)),r,mapX[1],mapY[1],cv::INTER_LINEAR);
            Frame actual(l,r,idx*.1,&le,&re,nullptr,matrix,dist,float(-p2.at<double>(0,3)),2.5,&measured);
            // Compare both algorithms on the same fully initialized Frame.
            // The constructor initializes baseline fields after its first match.
            actual.ComputeStereoMatches();
            ReferenceFrame oracle(actual);oracle.ComputeReference();
            require(std::memcmp(actual.mvDepth.data(),oracle.mvDepth.data(),actual.mvDepth.size()*sizeof(float))==0,"recorded depth changed");
            require(std::memcmp(actual.mvuRight.data(),oracle.mvuRight.data(),actual.mvuRight.size()*sizeof(float))==0,"recorded right coordinates changed");
            ++recorded;
        }
        require(recorded>=15,"recorded equivalence coverage too small");
    }
    std::cout<<"RECORDED_STEREO_EQUIVALENCE frames="<<recorded<<std::endl;
    std::cout<<"STEREO_MATCH_REGRESSION_PASS patches=3000 frames=16 matched="<<valid
        <<" reference_median_ms="<<referenceMs[referenceMs.size()/2]
        <<" optimized_median_ms="<<optimizedMs[optimizedMs.size()/2]<<std::endl;
}
