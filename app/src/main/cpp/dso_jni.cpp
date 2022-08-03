#include <string.h>
#include <jni.h>
#include <boost/thread.hpp>
#include <locale.h>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <util/MainSettings.h>

#include <iostream>
#include <istream>
#include <sstream>
#include <fstream>

#include "IOWrapper/Output3DWrapper.h"
#include "IOWrapper/ImageDisplay.h"
#include "IOWrapper/Android/AndroidOutput3DWrapper.h"
#include "IOWrapper/Android/KeyFrameDisplay.h"

#include "util/settings.h"
#include "util/globalFuncs.h"
#include "util/DatasetReader.h"
#include "util/globalCalib.h"
#include "util/logger.h"

#include "util/NumType.h"
#include "FullSystem/FullSystem.h"
#include "OptimizationBackend/MatrixAccumulators.h"
#include "FullSystem/PixelSelector2.h"
#include "util/MinimalImage.h"
#include <jni.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>

// FIXME: remove hard code
#define IMAGE_DIR "/sdcard/dataset-outdoors3_512_16/dso/cam0/images"
std::string source = IMAGE_DIR;
std::string calib = "";
std::string gammaCalib = "";
std::string vignette = "";
std::string gtFile = "";
std::string imuFile = "";
int maxPreloadImages = 0; // If set we only preload if there are less images to be loade.


double rescale = 1;
bool reverse = false;
int start = 0;
int end = 100000;
bool prefetch = false;
float playbackSpeed = 0;    // 0 for linearize (play as fast as possible, while sequentializing tracking & mapping). otherwise, factor on timestamps.
bool preload = false;
bool useSampleOutput = false;
bool use16Bit = false;
bool linearizeOperation = (playbackSpeed == 0);

using namespace dso;
dmvio::MainSettings mainSettings;
dmvio::IMUCalibration imuCalibration;
dmvio::IMUSettings imuSettings;

class DSOSlamSystem {
public:
    DSOSlamSystem() {
        reader_ = new ImageFolderReader(source, mainSettings.calib, mainSettings.gammaCalib,
                                        mainSettings.vignette, use16Bit);
        reader_->loadIMUData(imuFile);
        reader_->setGlobalCalibration();
        undistort_ = Undistort::getUndistorterForFile(mainSettings.calib, mainSettings.gammaCalib,
                                                      mainSettings.vignette);
        imuCalibration.loadFromFile(mainSettings.imuCalibFile);
        fullSystem_ = new FullSystem(linearizeOperation, imuCalibration, imuSettings);
        fullSystem_->setGammaFunction(reader_->getPhotometricGamma());
        fullSystem_->linearizeOperation = (playbackSpeed == 0);

        outputWrapper_ = new IOWrap::AndroidOutput3DWrapper(wG[0], hG[0], false);
        fullSystem_->outputWrapper.push_back(outputWrapper_);

        frameId_ = start;
    }

    ~DSOSlamSystem() {
        if (reader_) {
            delete reader_;
            reader_ = NULL;
        }
        if (fullSystem_) {
            delete fullSystem_;
            fullSystem_ = NULL;
        }
        if (outputWrapper_) {
            delete outputWrapper_;
            outputWrapper_ = NULL;
        }
        if (undistort_) {
            delete undistort_;
            undistort_ = NULL;
        }
    }

    void onFrameByData(int width, int height, unsigned char *data) {
        // Ref. https://github.com/JakobEngel/dso_ros/blob/master/src/main.cpp vidCb function
        if (setting_fullResetRequested) {
            std::vector<IOWrap::Output3DWrapper *> wraps = fullSystem_->outputWrapper;
            delete fullSystem_;
            for (IOWrap::Output3DWrapper *ow: wraps) ow->reset();
            imuCalibration.loadFromFile(mainSettings.imuCalibFile);
            fullSystem_ = new FullSystem(linearizeOperation, imuCalibration, imuSettings);
            fullSystem_->linearizeOperation = false;
            fullSystem_->outputWrapper = wraps;
            if (undistort_->photometricUndist != 0)
                fullSystem_->setGammaFunction(undistort_->photometricUndist->getG());
            setting_fullResetRequested = false;
        }

        MinimalImageB minImg(width, height, data);
        auto timestamp = std::chrono::system_clock::now().time_since_epoch().count();
        ImageAndExposure *undistImg = undistort_->undistort<unsigned char>(
                &minImg, 1.0f, timestamp / 1000);
        fullSystem_->addActiveFrame(undistImg, frameId_, 0, 0);
        frameId_++;
        delete undistImg;
    }

    void onFrameByPath(std::string path) {
        ImageAndExposure *img = reader_->getImage(frameId_);
//        bool gtDataThere = reader_->loadGTData(gtFile);
//        dmvio::GTData data;
//        bool found = false;
//        if (gtDataThere) {
//            data = reader_->getGTData(frameId_, found);
//        }

        std::unique_ptr<dmvio::IMUData> imuData;
        if (setting_useIMU) {
            imuData = std::make_unique<dmvio::IMUData>(reader_->getIMUData(frameId_));
        }

        fullSystem_->addActiveFrame(img, frameId_, imuData.get(), 0);
//                                    (gtDataThere && found) ? &data : 0);
//        if (gtDataThere && found && !disableAllDisplay) {
////                TODO add gt data viewer
////                viewer->addGTCamPose(data.pose);
//        }
        delete img;
        if (fullSystem_->initFailed || setting_fullResetRequested) {
            if (frameId_ < 250 || setting_fullResetRequested) {
                        printf("RESETTING!\n");
                std::vector<IOWrap::Output3DWrapper *> wraps = fullSystem_->outputWrapper;
                delete fullSystem_;
                for (IOWrap::Output3DWrapper *ow: wraps) ow->reset();
                fullSystem_ = new FullSystem(linearizeOperation, imuCalibration, imuSettings);
                fullSystem_->setGammaFunction(reader_->getPhotometricGamma());
                fullSystem_->outputWrapper = wraps;
                setting_fullResetRequested = false;
            }
        }
        if (fullSystem_->isLost) {
                    printf("LOST!!\n");
        }
        frameId_++;
    }

    float fx() {
        return fullSystem_->getCalibHessian().fxl();
    }

    float fy() {
        return fullSystem_->getCalibHessian().fyl();
    }

    float cx() {
        return fullSystem_->getCalibHessian().cxl();
    }

    float cy() {
        return fullSystem_->getCalibHessian().cyl();
    }

    int width() {
        return wG[0];
    }

    int height() {
        return hG[0];
    }

    SE3 currentCameraPose() {
        return outputWrapper_->currentCamPose();
    }

    int getKeyframeCount() {
        return outputWrapper_->getKeyframeCount();
    }

    MinimalImageB3 *cloneKeyframeImage() {
        return outputWrapper_->cloneKeyframeImage();
    }

    std::vector<std::pair<int, IOWrap::MyVertex *> > getVertices() {
        return outputWrapper_->getVertices();
    }

private:
    ImageFolderReader *reader_;
    Undistort *undistort_;;
    FullSystem *fullSystem_;
    IOWrap::AndroidOutput3DWrapper *outputWrapper_;
    int frameId_;
};

static DSOSlamSystem *gSlamSystem = NULL;

extern "C" {
JavaVM *gJvm = NULL;

JNIEXPORT void JNICALL
Java_com_example_slamapp_TARNativeInterface_dsoInit(JNIEnv *env, jobject thiz, jobjectArray jargv) {
            printf("dsoInit\n");
    //jargv is a Java array of Java strings
    int argc = env->GetArrayLength(jargv);
    typedef char *pchar;
    pchar *argv = new pchar[argc];
    int i;
    for (i = 0; i < argc; i++) {
        jstring js = static_cast<jstring>(env->GetObjectArrayElement(jargv, i)); //A Java string
        const char *pjc = env->GetStringUTFChars(js, 0); //A pointer to a Java-managed char buffer
        size_t jslen = strlen(pjc);
        argv[i] = new char[jslen + 1]; //Extra char for the terminating null
        // Copy to *our* buffer. We could omit that, but IMHO this is cleaner. Also, const correctness.
        strcpy(argv[i], pjc);
        env->ReleaseStringUTFChars(js, pjc);
                printf("argv[%i]: %s\n", i, pjc);
    }
    env->GetJavaVM(&gJvm);

    auto settingsUtil = std::make_shared<dmvio::SettingsUtil>();
    // Create Settings files.
    imuSettings.registerArgs(*settingsUtil);
    imuCalibration.registerArgs(*settingsUtil);
    mainSettings.registerArgs(*settingsUtil);

    // Dataset specific arguments. For other commandline arguments check out MainSettings::parseArgument,
    // MainSettings::registerArgs, IMUSettings.h and IMUInitSettings.h
    settingsUtil->registerArg("files", source);
    settingsUtil->registerArg("start", start);
    settingsUtil->registerArg("end", end);
    settingsUtil->registerArg("imuFile", imuFile);
    settingsUtil->registerArg("gtFile", gtFile);
    settingsUtil->registerArg("sampleoutput", useSampleOutput);
    settingsUtil->registerArg("reverse", reverse);
    settingsUtil->registerArg("use16Bit", use16Bit);
    settingsUtil->registerArg("maxPreloadImages", maxPreloadImages);

    mainSettings.parseArguments(argc, argv, *settingsUtil);

    if (mainSettings.imuCalibFile != "") {
        imuCalibration.loadFromFile(mainSettings.imuCalibFile);
    }

    // Print settings to commandline and file.
            printf("Settings:\n");
    settingsUtil->printAllSettings(std::cout);
    {
        std::ofstream settingsStream;
        settingsStream.open(imuSettings.resultsPrefix + "usedSettingsdso.txt");
        settingsUtil->printAllSettings(settingsStream);
    }

    gSlamSystem = new DSOSlamSystem();
}

JNIEXPORT void JNICALL
Java_com_example_slamapp_TARNativeInterface_dsoRelease(JNIEnv *env, jobject thiz) {
            printf("dsoRelease\n");
    if (gSlamSystem) {
        delete gSlamSystem;
        gSlamSystem = NULL;
    }
}

JNIEXPORT int JNICALL
Java_com_example_slamapp_TARNativeInterface_dsoOnFrameByData(JNIEnv *env, jobject thiz, jint width,
                                                             jint height, jbyteArray frameData,
                                                             jint format) {

    unsigned char grayData[640 * 480] = {0};
    env->GetByteArrayRegion(frameData, 0, width * height, (jbyte *) grayData);
    gSlamSystem->onFrameByData(width, height, grayData);
    env->DeleteLocalRef(frameData);

    return 0;
}

JNIEXPORT int JNICALL
Java_com_example_slamapp_TARNativeInterface_dsoOnFrameByPath(JNIEnv *env, jobject thiz,
                                                             jstring path) {
    const char *imgFile = env->GetStringUTFChars(path, 0);
    gSlamSystem->onFrameByPath(imgFile);
    env->ReleaseStringUTFChars(path, imgFile);

    return 0;
}

JNIEXPORT jfloatArray JNICALL
Java_com_example_slamapp_TARNativeInterface_dsoGetIntrinsics(JNIEnv *env, jobject thiz) {
    jfloatArray result;
    result = env->NewFloatArray(4);
    if (result == NULL) {
        return NULL; /* out of memory error thrown */
    }

    jfloat array1[4];
    array1[0] = gSlamSystem->cx();
    array1[1] = gSlamSystem->cy();
    array1[2] = gSlamSystem->fx();
    array1[3] = gSlamSystem->fy();

    env->SetFloatArrayRegion(result, 0, 4, array1);
    return result;
}

JNIEXPORT jfloatArray JNICALL
Java_com_example_slamapp_TARNativeInterface_dsoGetCurrentPose(JNIEnv *env, jobject thiz) {
    jfloatArray result;
    int length = 16;
    result = env->NewFloatArray(length);
    if (result == NULL) {
        return NULL; /* out of memory error thrown */
    }

    jfloat mat4[length];
    Sophus::Matrix4f m = gSlamSystem->currentCameraPose().matrix().cast<float>();
    float *pose = m.data();
    memcpy(mat4, pose, sizeof(jfloat) * length);

    env->SetFloatArrayRegion(result, 0, length, mat4);
    return result;
}

JNIEXPORT jobject JNICALL
Java_com_example_slamapp_TARNativeInterface_dsoGetPointCloud(JNIEnv *env, jobject thiz) {
    int pointNum = 0;
    std::vector<std::pair<int, IOWrap::MyVertex *> > vertices = gSlamSystem->getVertices();
    for (std::vector<std::pair<int, IOWrap::MyVertex *> >::iterator it = vertices.begin();
         it != vertices.end(); ++it) {
        pointNum += it->first;
    }

    jfloat *points = new jfloat[pointNum * 3];
    jint *colors = new jint[pointNum];
    int points_offset = 0;
    int colors_offset = 0;
    for (std::vector<std::pair<int, IOWrap::MyVertex *> >::iterator it = vertices.begin();
         it != vertices.end(); ++it) {
        for (int i = 0; i < it->first; ++i) {
            memcpy(points + points_offset, it->second[i].point, 3 * sizeof(float));
            colors[colors_offset] =
                    (it->second[i].color[3] << 24) + (it->second[i].color[0] << 16) +
                    (it->second[i].color[1] << 8) + it->second[i].color[2];
            points_offset += 3;
            colors_offset++;
        }
    }

    jclass classKeyFrame = env->FindClass("com/example/slamapp/DSOPointCloud");

    // new DSOPointCloud object
    jmethodID initMethodID = env->GetMethodID(classKeyFrame, "<init>", "()V");
    assert (initMethodID != NULL);
    jobject pointCloudObject = env->NewObject(classKeyFrame, initMethodID);
    assert (pointCloudObject != NULL);

    // set pointCount
    jint pointCount = pointNum;
    jfieldID pointCountFieldID = env->GetFieldID(classKeyFrame, "pointCount", "I");
    assert (pointCountFieldID != NULL);
    env->SetIntField(pointCloudObject, pointCountFieldID, pointCount);

    // set points
    jfloatArray pointsArray = env->NewFloatArray(pointNum * 3);
    env->SetFloatArrayRegion(pointsArray, 0, pointNum * 3, points);
    jfieldID pointsFieldID = env->GetFieldID(classKeyFrame, "worldPoints", "[F");
    assert (pointsFieldID != NULL);
    env->SetObjectField(pointCloudObject, pointsFieldID, pointsArray);

    // set colors
    jintArray colorsArray = env->NewIntArray(pointNum);
    env->SetIntArrayRegion(colorsArray, 0, pointNum, colors);
    jfieldID colorsFieldID = env->GetFieldID(classKeyFrame, "colors", "[I");
    assert (colorsFieldID != NULL);
    env->SetObjectField(pointCloudObject, colorsFieldID, colorsArray);

    // Release
    env->DeleteLocalRef(classKeyFrame);
    delete[] points;
    delete[] colors;

    return pointCloudObject;
}

JNIEXPORT jint JNICALL
Java_com_example_slamapp_TARNativeInterface_dsoGetKeyFrameCount(JNIEnv *env, jobject thiz) {
    return gSlamSystem->getKeyframeCount();
}

JNIEXPORT jbyteArray JNICALL
Java_com_example_slamapp_TARNativeInterface_dsoGetCurrentImage(JNIEnv *env, jobject thiz) {
    MinimalImageB3 *minImg = gSlamSystem->cloneKeyframeImage();
    int width = gSlamSystem->width();
    int height = gSlamSystem->height();
    int imgSize = width * height * 4;
    unsigned char *imgData = new unsigned char[imgSize];

    for (int i = 0; i < width * height; ++i) {
        imgData[i * 4] = minImg->data[i][0];
        imgData[i * 4 + 1] = minImg->data[i][1];
        imgData[i * 4 + 2] = minImg->data[i][2];
        imgData[i * 4 + 3] = (unsigned char) 0xff;
    }

    jbyteArray byteArray = env->NewByteArray(imgSize);
    env->SetByteArrayRegion(byteArray, 0, imgSize, (jbyte *) imgData);

    delete minImg;
    delete[] imgData;
    return byteArray;
}
}
