package com.example.slamapp;

/**
 * Created by aarontang on 2017/3/30.
 */

public class TARNativeInterface {
    public static final String TAG = TARNativeInterface.class.getSimpleName();
    public static native void dsoInit(String[] args);
    public static native void dsoRelease();
    public static native int dsoOnFrameByData(int width, int height, byte[] data, int format);
    public static native int dsoOnFrameByPath(String path);
    public static native float[] dsoGetIntrinsics();
    public static native float[] dsoGetCurrentPose();
    public static native com.example.slamapp.DSOPointCloud dsoGetPointCloud();
    public static native int dsoGetKeyFrameCount();
    public static native byte[] dsoGetCurrentImage();
}
