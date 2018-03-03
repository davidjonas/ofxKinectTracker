#pragma once
#include "ofxOpenCv.h"

class ofxKinectBlob {
  private:
    float zHint;
    float tolerance;
    ofVec3f direction;
    bool active;
    float lastSeen;
    bool overlap;

  public:
    int id;
    ofxCvBlob blob;

    ofxKinectBlob(int id, ofxCvBlob blob, float zHint, float tolerance);
    ~ofxKinectBlob();

    void update(ofxCvBlob blob, float zHint);
    float difference(ofxCvBlob otherBlob, float zHint);
    void draw(float x, float y);
    void setActive(bool value);
    void setOverlap(bool value);
    bool isActive();
    bool isOverlapping();
    float timeSinceLastSeen();
};
