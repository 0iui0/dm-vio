package com.example.slamapp;

import android.Manifest;
import android.app.Activity;
import android.content.Context;
import android.content.pm.PackageManager;
import android.graphics.Bitmap;
import android.graphics.Color;
import android.os.Build;
import android.os.Bundle;
import android.os.Process;
import android.util.Log;
import android.view.KeyEvent;
import android.view.View;
import android.view.ViewGroup;
import android.widget.FrameLayout;
import android.widget.ImageView;
import android.widget.RelativeLayout;
import android.widget.TextView;
import android.widget.Toast;

import androidx.annotation.NonNull;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;

import com.hjq.permissions.OnPermissionCallback;
import com.hjq.permissions.XXPermissions;

import org.rajawali3d.renderer.Renderer;
import org.rajawali3d.view.ISurface;
import org.rajawali3d.view.SurfaceView;

import java.io.File;
import java.nio.ByteBuffer;
import java.util.Arrays;
import java.util.List;

public class DSOActivity extends AppCompatActivity implements com.example.slamapp.DSORenderer.RenderListener {

    public static final String TAG = DSOActivity.class.getSimpleName();
    private static final boolean LOCAL_MODE = true;
    static {
        System.loadLibrary("slamapp");
    }

    private String mFileDir;
    private RelativeLayout mLayout;
    private SurfaceView mRajawaliSurface;
    private Renderer mRenderer;
    private ImageView mImageView;
    private boolean mStopped = false;
    private VideoSource mVideoSource;
    private boolean mStarted = false;
    private TextView mTextView;
    private long mLastUpdateTime;
    private int mLastUpdateIndex;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        init();

        mRajawaliSurface = createSurfaceView();
        mRenderer = createRenderer();
        applyRenderer();

        mLayout = new RelativeLayout(this);
        FrameLayout.LayoutParams childParams = new FrameLayout.LayoutParams(ViewGroup.LayoutParams
                .MATCH_PARENT, ViewGroup.LayoutParams.MATCH_PARENT);
        mLayout.addView(mRajawaliSurface, childParams);

        mImageView = new ImageView(this);
        RelativeLayout.LayoutParams imageParams = new RelativeLayout.LayoutParams(480, 320);
        imageParams.addRule(RelativeLayout.ALIGN_PARENT_RIGHT);
        imageParams.addRule(RelativeLayout.ALIGN_PARENT_TOP);
        mLayout.addView(mImageView, imageParams);

        mTextView = new TextView(this);
        mTextView.setTextColor(Color.YELLOW);
        RelativeLayout.LayoutParams textParams = new RelativeLayout.LayoutParams(600, 100);
        textParams.addRule(RelativeLayout.ALIGN_PARENT_LEFT);
        textParams.addRule(RelativeLayout.ALIGN_PARENT_TOP);
        mLayout.addView(mTextView, textParams);

        mVideoSource = new VideoSource(this, com.example.slamapp.Constants.IN_WIDTH, com.example.slamapp.Constants.IN_HEIGHT);
        if (!LOCAL_MODE) {
            mVideoSource.start();
        }

        setContentView(mLayout);
    }

    private void init() {
        mFileDir = getExternalFilesDir(null).getAbsolutePath();
        com.example.slamapp.Utils.copyAssets(this, mFileDir);
        String[] args={
                "files=/sdcard/dataset-outdoors3_512_16/dso/cam0/images",
                "vignette=/sdcard/dataset-outdoors3_512_16/dso/cam0/vignette.png",
                "imuFile=/sdcard/dataset-outdoors3_512_16/dso/imu.txt",
                "gtFile=/sdcard/dataset-outdoors3_512_16/dso/gt_imu.csv",
                "calib=/sdcard/dm-vio/configs/tumvi_calib/camera02.txt",
                "gamma=/sdcard/dm-vio/configs/tumvi_calib/pcalib.txt",
                "imuCalib=/sdcard/dm-vio/configs/tumvi_calib/camchain.yaml",
                "mode=0",
                "useimu=1",
                "use16Bit=1",
                "preset=0",
                "nogui=1",
                "resultsPrefix=/sdcard/tmp/",
                "settingsFile=/sdcard/dm-vio/configs/tumvi.yaml",
                "start=2"
        };
        XXPermissions.with(this)
                .permission(Manifest.permission.CAMERA)
                .permission(Manifest.permission.MANAGE_EXTERNAL_STORAGE)
                .request(new OnPermissionCallback (){
                    @Override
                    public void onGranted(List<String> permissions, boolean all) {
                        if (all) {
                            Log.i(TAG, "获取相机和sd卡权限成功");
                            onPermissionsGranted();
                        } else {
                            Log.i(TAG, "获取部分权限成功，但部分权限未正常授予");
                        }
                    }
                    @Override
                    public void onDenied(List<String> permissions, boolean never) {
                        if (never) {
                            Log.e(TAG, "被永久拒绝授权，请手动授予相机和sd卡权限");
                            // 如果是被永久拒绝就跳转到应用权限系统设置页面
                            XXPermissions.startPermissionActivity(DSOActivity.this, permissions);
                        } else {
                            Log.e(TAG, "获取相机和sd卡权限失败");
                        }                    }
                });
        TARNativeInterface.dsoInit(args);
    }

    private void onPermissionsGranted() {
//        TARNativeInterface.stringFromJNI();
    }

    protected SurfaceView createSurfaceView() {
        SurfaceView view = new SurfaceView(this);
        view.setFrameRate(60);
        view.setRenderMode(ISurface.RENDERMODE_WHEN_DIRTY);
        return view;
    }

    protected Renderer createRenderer() {
        com.example.slamapp.DSORenderer renderer = new com.example.slamapp.DSORenderer(this);
        renderer.setRenderListener(this);
        return renderer;
    }

    protected void applyRenderer() {
        mRajawaliSurface.setSurfaceRenderer(mRenderer);
    }

    public View getView() {
        return mLayout;
    }

    private void start() {
        DataThread thread = new DataThread("DSO_DataThread");
        thread.setOSPriority(Process.THREAD_PRIORITY_URGENT_AUDIO);
        thread.start();
    }

    @Override
    public boolean onKeyDown(int keyCode, KeyEvent event) {
        if (keyCode == KeyEvent.KEYCODE_BACK) {
            mStopped = true;
            TARNativeInterface.dsoRelease();
            finish();
            System.exit(0);
        } else if (keyCode == KeyEvent.KEYCODE_VOLUME_UP) {
            Toast.makeText(this, "Start", Toast.LENGTH_SHORT).show();
            start();
            mStarted = true;
            return true;
        } else if (keyCode == KeyEvent.KEYCODE_VOLUME_DOWN) {
            // TODO: reset
            Toast.makeText(this, "Reset!", Toast.LENGTH_SHORT).show();
            return true;
        }
        return true;
    }

    @Override
    public void onRender() {
        if (mImageView == null)
            return;

        byte[] imgData;
        int imgDataWidth = 0;
        int imgDataHeight = 0;
        if (!mStarted) {
            byte[] frameData = mVideoSource.getFrame();     // YUV data
            if (frameData == null)
                return;

            imgData = new byte[com.example.slamapp.Constants.IN_WIDTH * com.example.slamapp.Constants.IN_HEIGHT * 4];
            for (int i = 0; i < imgData.length / 4; ++i) {
                imgData[i * 4] = frameData[i];
                imgData[i * 4 + 1] = frameData[i];
                imgData[i * 4 + 2] = frameData[i];
                imgData[i * 4 + 3] = (byte) 0xff;
            }
            imgDataWidth = com.example.slamapp.Constants.IN_WIDTH;
            imgDataHeight = com.example.slamapp.Constants.IN_HEIGHT;
        } else {

            imgData = TARNativeInterface.dsoGetCurrentImage();
            imgDataWidth = com.example.slamapp.Constants.OUT_WIDTH;
            imgDataHeight = com.example.slamapp.Constants.OUT_HEIGHT;
        }

        if (imgData == null)
            return;

        final Bitmap bm = Bitmap.createBitmap(imgDataWidth, imgDataHeight, Bitmap.Config.ARGB_8888);
        bm.copyPixelsFromBuffer(ByteBuffer.wrap(imgData));
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                mImageView.setImageBitmap(bm);
            }
        });
    }

    private void refreshFrameRate(final int frameIndex) {
        if (System.currentTimeMillis() - mLastUpdateTime < 1000) {
            return;
        }
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                int diffIndex = frameIndex - mLastUpdateIndex;
                float diffTime = (float)(System.currentTimeMillis() - mLastUpdateTime) / 1000;
                mTextView.setText("Frame Rate: " + (float)diffIndex / diffTime + " fps\ncurrent index: " + frameIndex);

                mLastUpdateTime = System.currentTimeMillis();
                mLastUpdateIndex = frameIndex;
            }
        });

    }

    private class DataThread extends Thread {
        private int mOSPriority = Process.THREAD_PRIORITY_DEFAULT;

        public DataThread(String threadName) {
            super(threadName);
        }

        public void setOSPriority(int priority) {
            mOSPriority = priority;
        }

        @Override
        public void run() {
            Process.setThreadPriority(mOSPriority);
            mLastUpdateTime = System.currentTimeMillis();

            int i = 0;
            String imgDir = "/sdcard/dataset-outdoors3_512_16/dso/cam0/images";

            if (LOCAL_MODE) {
                File directory = new File(imgDir);
                File[] files = directory.listFiles();
                Arrays.sort(files);
                for (final File file : files) {
                    if (mStopped)
                        return;
                    if (!file.getName().endsWith(".png"))
                        continue;
                    TARNativeInterface.dsoOnFrameByPath(imgDir + File.separator + file.getName());
                    refreshFrameRate(i++);
                }
            } else {
                while (!mStopped) {
                    byte[] frameData = mVideoSource.getFrame();     // YUV data
                    if (frameData != null) {
                        TARNativeInterface.dsoOnFrameByData(com.example.slamapp.Constants.IN_WIDTH, com.example.slamapp.Constants.IN_HEIGHT, frameData, 0);
                        refreshFrameRate(i++);
                    }
                }
            }
        }
    }
}
