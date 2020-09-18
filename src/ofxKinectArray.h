#pragma once
#include "ofMain.h"
#include "ofxKinect.h"
#include "ofxOpenCv.h"

class ofxKinectImageCalibration
{
  private:
    int width;
    int height;
    ofVec2f position;
    ofVec2f scale;
    float rotation;

  public:
    ofxKinectImageCalibration() : width(640), height(480), position(0.0f, 0.0f), scale(1.0f, 1.0f), rotation(0) {
    }

    ofxKinectImageCalibration(int index) : width(640), height(480), position(640 * index, 0.0f), scale(1.0f, 1.0f), rotation(0) {
      ofLogNotice("ofxKinectImageCalibration") <<
        "Calibrating kinect " <<
        index <<
        " to position: (" <<
        position.x <<
        ", " <<
        position.y <<
        "), scale(" <<
        scale.x <<
        " ," <<
        scale.y<<
        "), rotation(" <<
        rotation <<
        ")";
    }

    ~ofxKinectImageCalibration(){

    }

    void setPosition(float x, float y)
    {
      position.x = x;
      position.y = y;
    }

    void setPosition(ofVec2f vec)
    {
      position = vec;
    }

    void setScale(float x, float y)
    {
      scale.x = x;
      scale.y = y;
    }

    void setScale(ofVec2f vec)
    {
      scale = vec;
    }

    void setRotation(float angle)
    {
      rotation = angle;
    }

    void setDimensions(int w, int h)
    {
      width = w;
      height = h;
    }

    ofVec2f getPosition(){
      return position;
    }

    ofVec2f getScale(){
      return scale;
    }

    float getRotation()
    {
      return rotation;
    }

    ofRectangle getBoundingRect()
    {
      if(rotation != 0)
      {
        float p1x, p1y, p2x, p2y, p3x, p3y, p4x, p4y, d, alpha, beta;

        alpha = ofDegToRad(rotation);
        d = sqrt((width*scale.x)*(width*scale.x) + (height*scale.y)*(height*scale.y));
        beta = acos((width*scale.x)/d);

        p1x = position.x;
        p1y = position.y;
        p2x = cos(alpha)*(width*scale.x) + p1x;
        p2y = sin(alpha)*(width*scale.x) + p1y;
        p3x = cos(alpha+beta)*d + p1x;
        p3y = sin(alpha+beta)*d + p1y;
        p4x = cos(alpha+HALF_PI)*(height*scale.y) + p1x;
        p4y = sin(alpha+HALF_PI)*(height*scale.y) + p1y;

        std::vector<float> xs = {p1x, p2x, p3x, p4x};
        std::vector<float> ys = {p1y, p2y, p3y, p4y};

        float minx = *std::min_element(xs.begin(), xs.end());
        float miny = *std::min_element(ys.begin(), ys.end());
        float maxx = *std::max_element(xs.begin(), xs.end());
        float maxy = *std::max_element(ys.begin(), ys.end());

        ofRectangle rect(minx, miny, maxx-minx, maxy-miny);

        return rect;
      }
      else{
        return ofRectangle(position.x, position.y, (width*scale.x), (height*scale.y));
      }
    }
};

class ofxKinectArray
{
  private:
    std::vector<ofxKinect *> kinects;
    std::vector<ofxKinectImageCalibration *> calibrations;
    bool registration;
    int detectedKinectsCache;
    ofFbo colorFbo;
    ofFbo depthFbo;
    ofPixels colorPixels;
    ofPixels depthPixels;
    ofxCvColorImage colorImg;
    ofxCvGrayscaleImage grayscale;

  public:
    int width;
    int height;

    ofxKinectArray() : registration(false), detectedKinectsCache(-1), width(0), height(0) {

    }

    ~ofxKinectArray(){

    }

    int numKinectsDetected()
    {
      if(detectedKinectsCache == -1)
      {
        ofxKinect kinectCounter;
        int detectedKinectsCache = kinectCounter.numTotalDevices();
        kinectCounter.close();
        return detectedKinectsCache;
      }
      else{
        return detectedKinectsCache;
      }
    }

    void setRegistration(bool value)
    {
      registration = value;

      for(uint8_t i=0; i<kinects.size(); i++)
      {
        kinects[i]->setRegistration(registration);
      }
    }

    void init()
    {
      ofLogNotice("ofxKinectArray::init") << "Initializing " << numKinectsDetected() << " kinects.";
      for(uint8_t i=0; i<numKinectsDetected(); i++)
      {
        ofxKinect * k = new ofxKinect();
        k->init();
        kinects.push_back(k);
        ofxKinectImageCalibration * c = new ofxKinectImageCalibration(i);
        calibrations.push_back(c);
      }
    }

    void open()
    {
      for(uint8_t i=0; i<kinects.size(); i++)
      {
        kinects[i]->open(i);
        width += kinects[i]->width;
        height = kinects[i]->height;
        calibrations[i]->setDimensions(kinects[i]->width, kinects[i]->height);
      }
      ofLogNotice("ofKinectArray") << "Alocating FBO of size: " << width << ", " << height;
      allocateImages();
    }

    void allocateImages()
    {
      colorFbo.allocate(width, height, GL_RGB);
      depthFbo.allocate(width, height, GL_RGB);
      colorPixels.allocate(width, height, GL_RGB);
      depthPixels.allocate(width, height, GL_RGB);
      colorImg.allocate(width, height);
      grayscale.allocate(width, height);
    }

    void open(int index)
    {
      if(index < kinects.size())
      {
        kinects[index]->open(index);
        width = kinects[index]->width;
        height = kinects[index]->height;
        allocateImages();
      }
      else
      {
        ofLogError("ofxKinectArray::open") << "There is no kinect with index " << index;
      }
    }

    void update()
    {
      for(uint8_t i=0; i<kinects.size(); i++)
      {
        kinects[i]->update();
      }
    }

    void close()
    {
      for(uint8_t i=0; i<kinects.size(); i++)
      {
        kinects[i]->close();
      }
    }

    void setCameraTiltAngle(int index, int kinectAngle)
    {
      if(index < kinects.size())
      {
        kinects[index]->setCameraTiltAngle(kinectAngle);
      }
      else
      {
        ofLogError("ofxKinectArray::setCameraTiltAngle") << "There is no kinect with index " << index;
      }
    }

    void setCameraTiltAngle(int kinectAngle)
    {
      for(uint8_t i=0; i<kinects.size(); i++)
      {
        kinects[i]->setCameraTiltAngle(kinectAngle);
      }
    }

    ofPixels & getPixels()
    {
      colorFbo.bind();
      glViewport(0, -height*2, width/(0.5*kinects.size()), height*2); //TODO: THis is really weird!! Why do I have to stretch the viewport 2x to fill up the buffer?
      ofClear(0, 0, 0, 255);
      ofSetColor(255);
      ofEnableBlendMode(OF_BLENDMODE_SCREEN);
      for(uint8_t i=0; i<kinects.size(); i++)
      {
        ofPushMatrix();
        ofTranslate(calibrations[i]->getPosition().x, calibrations[i]->getPosition().y);
        ofRotateDeg(calibrations[i]->getRotation());
        ofScale(calibrations[i]->getScale().x, -calibrations[i]->getScale().y);
        kinects[i]->draw(0,0);
        ofPopMatrix();
      }
      ofEnableBlendMode(OF_BLENDMODE_ALPHA);
      colorFbo.unbind();

      colorFbo.readToPixels(colorPixels);

      return colorPixels;
    }

    ofPixels & getDepthPixels()
    {
      depthFbo.bind();
      glViewport(0, -height*2, width/(0.5*kinects.size()), height*2); //TODO: THis is really weird!! Why do I have to stretch the viewport 2x to fill up the buffer?
      ofClear(0, 0, 0, 0);
      ofSetColor(255);
      ofEnableBlendMode(OF_BLENDMODE_SCREEN);
      for(uint8_t i=0; i<kinects.size(); i++)
      {
        ofPushMatrix();
        ofTranslate(calibrations[i]->getPosition().x, calibrations[i]->getPosition().y);
        ofRotateDeg(calibrations[i]->getRotation());
        ofScale(calibrations[i]->getScale().x, -calibrations[i]->getScale().y);
        kinects[i]->drawDepth(0,0);
        ofPopMatrix();
      }
      ofEnableBlendMode(OF_BLENDMODE_ALPHA);
      depthFbo.unbind();

      depthFbo.readToPixels(depthPixels);

      colorImg.setFromPixels(depthPixels);
      grayscale = colorImg;

      return grayscale.getPixels();
    }

    float getDistanceAt(ofPoint p)
    {
      return getDistanceAt(p.x, p.y);
    }

    float getDistanceAt(float x, float y)
    {
      for(uint8_t i=0; i<kinects.size(); i++)
      {
        ofRectangle bound = calibrations[i]->getBoundingRect();

        if(bound.inside(x, y))
        {
          return kinects[i]->getDistanceAt(calibrations[i]->getPosition().x + x, calibrations[i]->getPosition().y + y);
        }
      }
      return 0.0f;
    }

    void calibratePosition(int index, ofPoint p)
    {
      if(index < calibrations.size())
      {
        calibrations[index]->setPosition(p);
      }
      else
      {
        ofLogError("ofxKinectArray::calibratePosition") << "There is no kinect with index " << index;
      }
    }
};
