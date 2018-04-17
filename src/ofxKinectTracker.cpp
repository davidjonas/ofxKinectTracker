#include "ofxKinectTracker.h"

#define KINECTANGLEDEFAULT 0;

ofxKinectTracker::ofxKinectTracker() {
  kinectAngle = KINECTANGLEDEFAULT;
  initialized = false;
  tolerance = 50;
  edgeThreshold = 10.0f;
}

ofxKinectTracker::ofxKinectTracker(int index){
  init(index);
}

ofxKinectTracker::ofxKinectTracker(int index, float tolerance){
  init(index);
  this->tolerance = tolerance;
}

ofxKinectTracker::~ofxKinectTracker(){
  kinect.close();
}

int ofxKinectTracker::numKinectsDetected()
{
  return kinect.numKinectsDetected();
}

void ofxKinectTracker::init(int index){
  kinectIndex = index;
  kinect.setRegistration(true);
  kinect.init();
  kinect.open();

  width = kinect.width;
  height = kinect.height;

  kinectAngle = KINECTANGLEDEFAULT;
  //updateCameraTiltAngle();

  colorImg.allocate(width,height);
  grayscale.allocate(width, height);
  depthImage.allocate(width, height);
  background.allocate(width, height);
  diff.allocate(width, height);
  depthThreshold.allocate(width, height);
  threshold = 3;  //60
  blurAmount = 9;
  backgroundSubtract = false;
  blur = false;
  initialized = true;
  tolerance = 100;
  removeAfterSeconds = 5;
  idCounter = 0;
  maxDepth = 255;
  minDepth = 0;
  minBlobSize = 100;
  mode = TRACK_DEPTH;
}

//Getters and setters
void ofxKinectTracker::setBackgroundSubtract(bool value){
  backgroundSubtract = value;
}

void ofxKinectTracker::setBlur(bool value){
  blur = value;
}

void ofxKinectTracker::setBlurAmount(float value){
  if(value >= 1)
  {
    blurAmount = value;
  }
  else
  {
    blurAmount = 1;
  }
}

void ofxKinectTracker::setThreshold(float value){
  threshold = value;
  if(threshold > 255) threshold = 255;
  if(threshold < 0) threshold = 0;
}

void ofxKinectTracker::setMaxDepth(float value){
  maxDepth = value;
}

void ofxKinectTracker::setMinDepth(float value){
  minDepth = value;
}

void ofxKinectTracker::setTolerance(float value){
  tolerance = value;
  //Updating existing blobs.
  for(uint8_t i=0; i<blobs.size(); i++)
  {
    blobs[i].setTolerance(tolerance);
  }
}

void ofxKinectTracker::setRemoveAfterSeconds(float value){
  removeAfterSeconds = value;
}

void ofxKinectTracker::setEdgeThreshold(float value){
  edgeThreshold = value;
}

void ofxKinectTracker::setMinBlobSize(float value){
  minBlobSize = value;
}

bool ofxKinectTracker::getBackgroundSubtract(){
  return backgroundSubtract;
}

bool ofxKinectTracker::getBlur(){
  return blur;
}

float ofxKinectTracker::getBlurAmount(){
  return blurAmount;
}

float ofxKinectTracker::getThreshold(){
  return threshold;
}

float ofxKinectTracker::getMaxDepth(){
  return maxDepth;
}

float ofxKinectTracker::getMinDepth(){
  return minDepth;
}

float ofxKinectTracker::getTolerance(){
  return tolerance;
}

float ofxKinectTracker::getRemoveAfterSeconds(){
  return removeAfterSeconds;
}

float ofxKinectTracker::getEdgeThreshold(){
  return edgeThreshold;
}

float ofxKinectTracker::getMinBlobSize(){
  return minBlobSize;
}

int ofxKinectTracker::getWidth(){
  return width;
}

int ofxKinectTracker::getHeight(){
  return height;
}

int ofxKinectTracker::getKinectIndex(){
  return kinectIndex;
}

int ofxKinectTracker::getMode(){
  return mode;
}

void ofxKinectTracker::setMode(int m){
  mode = m;
}


int ofxKinectTracker::getNumActiveBlobs()
{
  uint8_t count = 0;
  for(uint8_t i=0; i<blobs.size(); i++)
  {
    if(blobs[i].isActive())
    {
      count++;
    }
  }
  return count;
}

vector<ofxKinectBlob> ofxKinectTracker::getActiveBlobs(){
  vector<ofxKinectBlob> vec;

  for(uint8_t i=0; i<blobs.size(); i++)
  {
    if(blobs[i].isActive())
    {
      vec.push_back(blobs[i]);
    }
  }

  return vec;
}

bool ofxKinectTracker::isOverlapCandidate(ofxKinectBlob blob){

  ofRectangle margin(edgeThreshold, edgeThreshold, width-edgeThreshold*2, height-edgeThreshold*2);
  ofRectangle inter = blob.blob.boundingRect.getIntersection(margin);
  return inter.getArea() >= blob.blob.boundingRect.getArea()/2;

  // bool result = true;
  // ofVec3f perdictedLocation = blob.blob.centroid + blob.direction;
  //
  // float distanceFromEdge = perdictedLocation.x; //left edge
  // if(perdictedLocation.y < distanceFromEdge) //Top edge
  // {
  //   distanceFromEdge = perdictedLocation.y;
  // }
  // if(width - perdictedLocation.x < distanceFromEdge) //Right edge
  // {
  //   distanceFromEdge = width - perdictedLocation.x;
  // }
  // if(height - perdictedLocation.y < distanceFromEdge) //Bottom edge
  // {
  //   distanceFromEdge = height - perdictedLocation.y;
  // }
  //
  // return distanceFromEdge > edgeThreshold;
}


void ofxKinectTracker::updateCameraTiltAngle(){
  kinect.setCameraTiltAngle(kinectAngle);
}

void ofxKinectTracker::update(){
  kinect.update();
  colorImg.setFromPixels(kinect.getPixels());
  depthImage.setFromPixels(kinect.getDepthPixels());
  grayscale = colorImg;

  if(blur)
  {
    depthImage.blurGaussian(blurAmount);
  }

  if(backgroundSubtract){
    subtractBackground();
    contourFinder.findContours(diff, minBlobSize, (width*height)/2, 20, false);
  }
  else {
    depthThreshold = depthImage;
    depthThreshold.threshold(threshold);
    //ofLogNotice("minimum blob size: ") << minBlobSize;
    contourFinder.findContours(depthThreshold, minBlobSize, (width*height)/2, 20, false);
  }

  matchAndUpdateBlobs();
}

void ofxKinectTracker::grabBackground() {
  if(mode == TRACK_DEPTH)
  {
    background.setFromPixels(depthImage.getPixels());
  }
  else {
    background.setFromPixels(grayscale.getPixels());
  }

  backgroundSubtract = true;
  clearBlobs();
}

void ofxKinectTracker::subtractBackground() {

  ofPixels & pix = depthImage.getPixels();
	ofPixels & bgPix = background.getPixels();
	ofPixels & d = diff.getPixels();

  if(mode == TRACK_COLOR)
  {
    pix = grayscale.getPixels();
  }

  //TODO: Better to do the threshold in the source depth image
  //like it is done in https://github.com/openframeworks/openFrameworks/blob/master/examples/computer_vision/kinectExample/src/ofApp.cpp

	int numPixels = pix.size();
	for(int i = 0; i < numPixels; i++) {
    if(pix[i] < 255-minDepth && pix[i] > 255-maxDepth){
  		if(abs(pix[i] - bgPix[i]) < threshold) {
  			d[i] = 0;
  		}
  		else
  		{
  			d[i] = 255;
  		}
    }
    else{
      d[i] = 0;
    }
	}

  diff.flagImageChanged();
}

//The Tracker
void ofxKinectTracker::matchAndUpdateBlobs()
{
  vector<ofxCvBlob> cvBlobs = contourFinder.blobs;
  vector<bool> trackedBlob(blobs.size(), false);
  vector<ofxCvBlob>::iterator currentBlob = cvBlobs.begin();
  vector<ofxCvBlob> newBlobs;

  while(currentBlob != cvBlobs.end())
  {
    int chosenMatch = -1;
    float minDifference = 10000; //TODO: this should be flagged instead of ridiculous value.

    for(int i=0; i<blobs.size(); i++)
    {
      float blobDiff = blobs[i].difference(*currentBlob, getZHintForBlob(*currentBlob));

      if(blobDiff != -1 && blobDiff <= minDifference)
      {
        if(blobDiff == minDifference)
        {
          //TODO: There are two blobs that match, this is really rare. How to decide which blob is which?
          //      right now the latest blob in the vector will be chosen.
          ofLog(OF_LOG_WARNING) << "Blob conflict found!!" << endl;
        }
        minDifference = blobDiff;
        chosenMatch = i;
      }
    }

    if(chosenMatch != -1)
    {
      blobs[chosenMatch].update(*currentBlob, getZHintForBlob(*currentBlob));
      trackedBlob[chosenMatch] = true;
    }
    else
    {
      bool isValid = true;
      // for(int i=0; i<blobs.size(); i++)
      // {
      //   float interArea = currentBlob->boundingRect.getIntersection(blobs[i].blob.boundingRect).getArea();
      //   if(interArea != 0 && (interArea >= currentBlob->boundingRect.getArea() * 0.9 || interArea >= blobs[i].blob.boundingRect.getArea() * 0.7))
      //   {
      //     isValid = false;
      //   }
      // }

      if(isValid) newBlobs.push_back(*currentBlob);
    }

    currentBlob++;
  }

  for(int i=0; i<newBlobs.size(); i++)
  {
    float zHint = getZHintForBlob(newBlobs[i]);
    ofxKinectBlob newBlob(++idCounter, newBlobs[i], zHint, tolerance);
    blobs.push_back(newBlob);
  }

  for(int i=0; i<trackedBlob.size(); i++)
  {
      if(!trackedBlob[i])
      {
        if(!blobs[i].isOverlapping() && blobs[i].timeSinceLastSeen() > removeAfterSeconds)
        {
            if(i < blobs.size()){
                blobs.erase(blobs.begin()+i);
            }
        }


        if((blobs[i].isActive() || blobs[i].isOverlapping()) && isOverlapCandidate(blobs[i]))
        {
          //TODO: Check for ovelapping blobs;
          int overlapIndex = -1;
          for(uint8_t b=0; b<blobs.size(); b++)
          {
            if(trackedBlob[b] && isOverlapCandidate(blobs[b]) && blobs[i].intersects(blobs[b]))
            {
              //BLOBS OVERLAP!
              blobs[i].setOverlap(true);
              blobs[b].setOverlap(true);
              overlapIndex = b;
              break;
            }
          }

          if(overlapIndex == -1)
          {
            blobs[i].setOverlap(false);
          }
        }
      }
      else {
        if(blobs[i].isOverlapping())
        {
          for(uint8_t b=0; b<blobs.size(); b++)
          {
            if(trackedBlob[b] && blobs[b].isOverlapping() && !blobs[i].intersects(blobs[b]))
            {
              //BLOBS STOPPED OVERLAPPING!
              blobs[i].setOverlap(false);
              blobs[b].setOverlap(false);
              break;
            }
          }
        }
      }
      blobs[i].setActive(trackedBlob[i]);
  }
}

float ofxKinectTracker::getZHintForBlob(ofxCvBlob blob)
{
  return kinect.getDistanceAt(blob.centroid);
}

void ofxKinectTracker::clearBlobs(){
  blobs.clear();
}


//Image Getters
ofxCvColorImage ofxKinectTracker::getColorImage(){
  return colorImg;
}

ofxCvGrayscaleImage ofxKinectTracker::getGrayImage(){
  return grayscale;
}

ofxCvGrayscaleImage ofxKinectTracker::getDepthImage(){
  return depthImage;
}

//Draw and debug methods
void ofxKinectTracker::drawDepth(float x, float y)
{
  depthImage.draw(x, y);
}

void ofxKinectTracker::drawDepth(float x, float y, float scale)
{
  depthImage.draw(x, y, width*scale, height*scale);
}

void ofxKinectTracker::drawRGB(float x, float y)
{
  colorImg.draw(x, y, width, height);
}

void ofxKinectTracker::drawRGB(float x, float y, float scale)
{
  colorImg.draw(x, y, width*scale, height*scale);
}

void ofxKinectTracker::drawBlobPositions(float x, float y){
  drawBlobPositions(x,y,1.0);
}

void ofxKinectTracker::drawBlobPositions(float x, float y, float scale){
  for (int i = 0; i < blobs.size(); i++){

    if(blobs[i].isActive())
    {
      ofFill();
      ofSetColor(255,0,0);
      ofCircle(x+ blobs[i].blob.centroid.x * scale,
             y+ blobs[i].blob.centroid.y * scale,
             10);

      ofSetColor(255);
      ofDrawBitmapString(ofToString(blobs[i].id),
            x+ blobs[i].blob.centroid.x * scale,
            y+ blobs[i].blob.centroid.y * scale);

      if(blobs[i].isOverlapping())
      {
        ofSetLineWidth(10);
        ofNoFill();
        ofSetColor(255,0,0);
        ofDrawRectangle(x+blobs[i].blob.boundingRect.x * scale,
                        y+blobs[i].blob.boundingRect.y * scale,
                        blobs[i].blob.boundingRect.width * scale,
                        blobs[i].blob.boundingRect.height * scale);
        ofSetLineWidth(1);
      }
    }
    else {
      ofNoFill();
      ofCircle(x+ blobs[i].blob.centroid.x * scale,
             y+ blobs[i].blob.centroid.y * scale,
             10);

      ofSetColor(255);
      ofDrawBitmapString(ofToString(blobs[i].id),
            x+ blobs[i].blob.centroid.x * scale,
            y+ blobs[i].blob.centroid.y * scale);
    }
  }
}

void ofxKinectTracker::drawBackground(float x, float y){
  background.draw(x,y);
}

void ofxKinectTracker::drawBackground(float x, float y, float scale){
  background.draw(x,y,width*scale, height*scale);
}

void ofxKinectTracker::drawContours(float x, float y){
  contourFinder.draw(x,y);
}

void ofxKinectTracker::drawContours(float x, float y, float scale){
  contourFinder.draw(x,y,width*scale,height*scale);
}

void ofxKinectTracker::drawDiff(float x, float y){
  diff.draw(x, y);
}

void ofxKinectTracker::drawDiff(float x, float y, float scale){
  diff.draw(x,y,width*scale,height*scale);
}

void ofxKinectTracker::drawDebug(float x, float y) {
  drawDebug(x, y, 1.0);
}


void ofxKinectTracker::drawDebug(float x, float y, float scale) {
  drawDepth(x,y,0.5 * scale);
  drawBackground(x+width*scale/2, y, 0.5 * scale);
  drawDiff(x, y+height*scale/2, 0.5 * scale);
  drawRGB(x+width*scale/2, y+height*scale/2, 0.5 * scale);
  drawContours(x+width*scale/2, y+height*scale/2, 0.5 * scale);
  drawBlobPositions(x+width*scale/2, y+height*scale/2, 0.5 * scale);
  drawEdgeThreshold(x+width*scale/2, y+height*scale/2, 0.5 * scale);
}

void ofxKinectTracker::drawEdgeThreshold(float x, float y)
{
  drawEdgeThreshold(x, y, 1.0);
}

void ofxKinectTracker::drawEdgeThreshold(float x, float y, float scale)
{
  ofNoFill();
  ofSetColor(255, 90, 90);
  ofDrawRectangle(x+(edgeThreshold * scale),y+(edgeThreshold * scale), (width*scale)-((edgeThreshold * scale)*2), (height*scale)-((edgeThreshold * scale)*2));
}

//Calibration
void ofxKinectTracker::calibratePosition(int index, ofPoint p){
  kinect.calibratePosition(index, p);
}

void ofxKinectTracker::close(){
  kinect.close();
}
