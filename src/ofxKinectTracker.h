#pragma once

#include "ofxKinect.h"
#include "ofxOpenCv.h"
#include "ofxKinectBlob.h"
#include "ofxKinectArray.h"

#define TRACK_DEPTH 0
#define TRACK_COLOR 1

class ofxKinectTracker {
  private:
    ofxKinectArray kinect;
    ofxCvColorImage colorImg;
    ofxCvGrayscaleImage grayscale;
    ofxCvGrayscaleImage depthImage;
    ofxCvGrayscaleImage background;
    ofxCvGrayscaleImage diff;
    ofxCvContourFinder contourFinder;
    ofxCvGrayscaleImage depthThreshold;

    int numKinects;
    int kinectAngle;
    int mode;

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
    float minBlobSize;
    float edgeThreshold;

  public:
    vector<ofxKinectBlob> blobs;

    ofxKinectTracker();
    ~ofxKinectTracker();

    int numKinectsDetected();

    //Getters and setters
    void setBackgroundSubtract(bool value);
    void setBlur(bool value);
    void setBlurAmount(float value);
    void setThreshold(float value);
    void setMaxDepth(float value);
    void setMinDepth(float value);
    void setTolerance(float value);
    void setRemoveAfterSeconds(float value);
    void setEdgeThreshold(float value);
    void setMinBlobSize(float value);
    bool getBackgroundSubtract();
    bool getBlur();
    float getBlurAmount();
    float getThreshold();
    float getMaxDepth();
    float getMinDepth();
    float getTolerance();
    float getRemoveAfterSeconds();
    float getEdgeThreshold();
    int getWidth();
    int getHeight();
    float getMinBlobSize();
    int getMode();
    void setMode(int m);
    int getNumActiveBlobs();
    vector<ofxKinectBlob> getActiveBlobs();
    bool isOverlapCandidate(ofxKinectBlob blob);
    bool thereAreOverlaps();
    ofxKinectBlob * getOverlapBlob();

    //Action Methods
    void init();
    void updateCameraTiltAngle();
    void update();
    void grabBackground();
    void subtractBackground();

    //The Tracker
    void matchAndUpdateBlobs();
    float getZHintForBlob(ofxCvBlob blob);
    void clearBlobs();

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
    void drawEdgeThreshold(float x, float y);
    void drawEdgeThreshold(float x, float y, float scale);

    //Calibration
    void calibratePosition(int index, ofPoint p);

    //closing
    void close();
};
