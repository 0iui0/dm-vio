#ifndef ANDROID_DSO_VIEWER_H_
#define ANDROID_DSO_VIEWER_H_

#include "boost/thread.hpp"
#include "util/MinimalImage.h"
#include "IOWrapper/Output3DWrapper.h"
#include "KeyFrameDisplay.h"
#include "util/NumType.h"


namespace dso
{


namespace IOWrap
{

class AndroidOutput3DWrapper : public Output3DWrapper
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    AndroidOutput3DWrapper(int w, int h, bool startRunThread=true);
    virtual ~AndroidOutput3DWrapper();

    void run();
    void close();


    // ==================== Output3DWrapper Functionality ======================
    virtual void publishGraph(const std::map<uint64_t,Eigen::Vector2i> &connectivity);
    virtual void publishKeyframes( std::vector<FrameHessian*> &frames, bool final, CalibHessian* HCalib);
    virtual void publishCamPose(FrameShell* frame, CalibHessian* HCalib);


    virtual void pushLiveFrame(FrameHessian* image);
    virtual void pushDepthImage(MinimalImageB3* image);
    virtual bool needPushDepthImage();

    virtual void join();

    virtual void reset();
    
    SE3 currentCamPose();
    int getKeyframeCount();
    MinimalImageB3* cloneVideoImage();
    MinimalImageB3* cloneKeyframeImage();
    std::vector<std::pair<int, MyVertex*> > getVertices();
    

private:
    boost::thread runThread;
    bool running_;
    int w_, h_;

    // 3D model rendering
    boost::mutex model3DMutex_;
	KeyFrameDisplay* currentCam_;
	std::vector<KeyFrameDisplay*> keyframes_;
	std::map<int, KeyFrameDisplay*> keyframesByKFID_;

	// images rendering
	boost::mutex openImagesMutex_;
	MinimalImageB3* internalVideoImg_;
	MinimalImageB3* internalKFImg_;
	
};

}
}
#endif // ANDROID_DSO_VIEWER_H_
