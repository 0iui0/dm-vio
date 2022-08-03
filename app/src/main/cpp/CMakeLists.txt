# For more information about using CMake with Android Studio, read the
# documentation: https://d.android.com/studio/projects/add-native-code.html

# Sets the minimum version of CMake required to build the native library.

cmake_minimum_required(VERSION 3.18.1)

# Declares and names the project.

project("slamapp")
if (ANDROID)
    include(cmake/ndk-pkg.cmake)
endif()
find_package(yaml-cpp REQUIRED CONFIG)
find_package(boost REQUIRED CONFIG)
# configure import libs
set(distribution_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../../distribution)

######         opencv         ######
add_library( opencv-lib SHARED IMPORTED )
set_target_properties(opencv-lib PROPERTIES IMPORTED_LOCATION
        ${distribution_DIR}/opencv/lib/${ANDROID_ABI}/libopencv_java4.so)

######         gtsam         ######
add_library( gtsam-lib SHARED IMPORTED )
set_target_properties(gtsam-lib PROPERTIES IMPORTED_LOCATION
        ${distribution_DIR}/gtsam/lib/${ANDROID_ABI}/libgtsam.so)
######         gtsam         ######
add_library( gtsam-metis SHARED IMPORTED )
set_target_properties(gtsam-metis PROPERTIES IMPORTED_LOCATION
        ${distribution_DIR}/gtsam/lib/${ANDROID_ABI}/libmetis-gtsam.so)

include_directories(
        .
        dso
)
# DSO Source files
set(dso_SOURCE_FILES
        dso/FullSystem/FullSystem.cpp
        dso/FullSystem/FullSystemOptimize.cpp
        dso/FullSystem/FullSystemOptPoint.cpp
        dso/FullSystem/FullSystemDebugStuff.cpp
        dso/FullSystem/FullSystemMarginalize.cpp
        dso/FullSystem/Residuals.cpp
        dso/FullSystem/CoarseTracker.cpp
        dso/FullSystem/CoarseInitializer.cpp
        dso/FullSystem/ImmaturePoint.cpp
        dso/FullSystem/HessianBlocks.cpp
        dso/FullSystem/PixelSelector2.cpp
        dso/IOWrapper/ImageDisplay_dummy.cpp
#        dso/IOWrapper/ImageRW_dummy.cpp
#        dso/IOWrapper/OpenCV/ImageDisplay_OpenCV.cpp
        dso/IOWrapper/OpenCV/ImageRW_OpenCV.cpp
        dso/IOWrapper/Android/KeyFrameDisplay.cpp
        dso/IOWrapper/Android/AndroidOutput3DWrapper.cpp
        dso/OptimizationBackend/EnergyFunctional.cpp
        dso/OptimizationBackend/AccumulatedTopHessian.cpp
        dso/OptimizationBackend/AccumulatedSCHessian.cpp
        dso/OptimizationBackend/EnergyFunctionalStructs.cpp
        dso/util/settings.cpp
        dso/util/Undistort.cpp
        dso/util/globalCalib.cpp
        )
set(dmvio_SOURCE_FILES
        IMU/IMUIntegration.cpp
        GTSAMIntegration/Sim3GTSAM.cpp
        IMUInitialization/GravityInitializer.cpp
        IMU/IMUTypes.cpp
        IMU/IMUSettings.cpp
        util/TimeMeasurement.cpp
        util/SettingsUtil.cpp
        GTSAMIntegration/BAGTSAMIntegration.cpp
        IMU/CoarseIMULogic.cpp
        IMU/BAIMULogic.cpp
        GTSAMIntegration/PoseTransformation.cpp
        GTSAMIntegration/Marginalization.cpp
        GTSAMIntegration/PoseTransformationIMU.cpp
        GTSAMIntegration/PoseTransformationFactor.cpp
        IMUInitialization/CoarseIMUInitOptimizer.cpp
        IMUInitialization/IMUInitializer.cpp
        IMU/IMUUtils.cpp
        IMUInitialization/IMUInitSettings.cpp
        GTSAMIntegration/GTSAMUtils.cpp
        GTSAMIntegration/DelayedMarginalization.cpp
        IMUInitialization/PoseGraphBundleAdjustment.cpp
        GTSAMIntegration/FEJValues.cpp
        IMUInitialization/IMUInitializerStates.cpp
        IMUInitialization/IMUInitializerLogic.cpp
        IMUInitialization/IMUInitializerTransitions.cpp
        GTSAMIntegration/AugmentedScatter.cpp
        live/FrameContainer.cpp
        live/IMUInterpolator.cpp
        util/MainSettings.cpp
        live/FrameSkippingStrategy.cpp
        )


add_library( # Sets the name of the library.
        slamapp
        SHARED
        ${dso_SOURCE_FILES}
        ${dmvio_SOURCE_FILES}
        dso_jni.cpp
        )

target_include_directories(slamapp PRIVATE
        ${distribution_DIR}/opencv/include
        ${distribution_DIR}/suitesparse/include
        ${distribution_DIR}/sophus/include
        ${distribution_DIR}/eigen3/include
        ${distribution_DIR}/yaml/include
        ${distribution_DIR}/sse2neon/include
        ${distribution_DIR}/gtsam/include)
find_library( # Sets the name of the path variable.
        log-lib
        log)

target_link_libraries( # Specifies the target library.
        slamapp
        android
        opencv-lib
        gtsam-lib
        gtsam-metis
        boost::libboost_filesystem.so
        yaml-cpp::libyaml-cpp.so
        ${log-lib})