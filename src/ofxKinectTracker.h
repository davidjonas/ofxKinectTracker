#pragma once

#include "ofxKinect.h"
#include "ofxOpenCv.h"
#include "ofxKinectBlob.h"

class ofxKinectTracker {
  private:
    ofxKinect * kinect;
    ofxCvColorImage colorImg;
    ofxCvGrayscaleImage depthImage;
    ofxCvGrayscaleImage background;
    ofxCvGrayscaleImage diff;
    ofxCvContourFinder contourFinder;

    int kinectIndex;
    int kinectAngle;

    //flags
    bool backgroundSubtract;
    bool blur;
    bool initialized;

    float blurAmount;
    float threshold;
    float maxDepth;
    float minDepth;
    float width;
    float height;
    float maxBlobs;
    float tolerance;
    float removeAfterSeconds;
    int idCounter;

  public:
    vector<ofxKinectBlob> blobs;

    ofxKinectTracker();
    ofxKinectTracker(int index);
    ofxKinectTracker(int index, float tolerance);
    ~ofxKinectTracker();

    //Getters and setters
    void setBackgroundSubtract(bool value);
    void setBlur(bool value);
    void setBlurAmount(float value);
    void setThreshold(float value);
    void setMaxDepth(float value);
    void setMinDepth(float value);
    void setTolerance(float value);
    void setRemoveAfterSeconds(float value);
    bool getBackgroundSubtract();
    bool getBlur();
    float getBlurAmount();
    float getThreshold();
    float getMaxDepth();
    float getMinDepth();
    float getTolerance();
    float getRemoveAfterSeconds();
    int getWidth();
    int getHeight();
    int getKinectIndex();

    //Action Methods
    void init(int index);
    void updateCameraTiltAngle();
    void update();
    void grabBackground();
    void subtractBackground();

    //The Tracker
    void matchAndUpdateBlobs();
    float getZHintForBlob(ofxCvBlob blob);

    //Image Getters
    ofxCvColorImage getColorImage();
    ofxCvGrayscaleImage getGrayImage();
    ofxCvGrayscaleImage getDepthImage();


    //Draw and debug methods
    void draw();
    void drawDepth(float x, float y);
    void drawRGB(float x, float y);
    void drawDepth(float x, float y, float scale);
    void drawRGB(float x, float y, float scale);
    void drawBlobPositions(float x, float y);
    void drawBlobPositions(float x, float y, float scale);
    void drawBackground(float x, float y);
    void drawBackground(float x, float y, float scale);
    void drawContours(float x, float y);
    void drawContours(float x, float y, float scale);
    void drawDiff(float x, float y);
    void drawDiff(float x, float y, float scale);
    void drawDebug(float x, float y);
    void drawDebug(float x, float y, float scale);

    //closing
    void close();
};
