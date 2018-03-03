#include "ofxKinectTracker.h"

#define KINECTANGLEDEFAULT 0;

ofxKinectTracker::ofxKinectTracker() {
  kinectAngle = KINECTANGLEDEFAULT;
  initialized = false;
  tolerance = 50;
}

ofxKinectTracker::ofxKinectTracker(int index){
  init(index);
}

ofxKinectTracker::ofxKinectTracker(int index, float tolerance){
  init(index);
  this->tolerance = tolerance;
}

void ofxKinectTracker::init(int index){
  kinectIndex = index;
  kinect = new ofxKinect();
  kinect->setRegistration(true);
  kinect->init();
  kinect->open(kinectIndex);

  width = kinect->width;
  height = kinect->height;

  kinectAngle = KINECTANGLEDEFAULT;
  updateCameraTiltAngle();

  colorImg.allocate(width,height);
  depthImage.allocate(width, height);
  background.allocate(width, height);
  diff.allocate(width, height);
  threshold = 3;  //60
  blurAmount = 9;
  backgroundSubtract = false;
  blur = false;
  initialized = true;
  tolerance = 100;
  removeAfterSeconds = 10;
  idCounter = 0;
  maxDepth = 255;
  minDepth = 0;
}

ofxKinectTracker::~ofxKinectTracker(){
  kinect->close();
  delete kinect;
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
}

void ofxKinectTracker::setRemoveAfterSeconds(float value){
  removeAfterSeconds = value;
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

int ofxKinectTracker::getWidth(){
  return width;
}

int ofxKinectTracker::getHeight(){
  return height;
}

int ofxKinectTracker::getKinectIndex(){
  return kinectIndex;
}

void ofxKinectTracker::updateCameraTiltAngle(){
  kinect->setCameraTiltAngle(kinectAngle);
}

void ofxKinectTracker::update(){
  kinect->update();
  colorImg.setFromPixels(kinect->getPixels());
  depthImage.setFromPixels(kinect->getDepthPixels());
  if(blur)
  {
    depthImage.blurGaussian(blurAmount);
  }

  if(backgroundSubtract){
    subtractBackground();
    contourFinder.findContours(diff, 100, (width*height)/2, 20, false);
  }
  else {
    depthImage.threshold(threshold);
    contourFinder.findContours(depthImage, 100, (width*height)/2, 20, false);
  }

  matchAndUpdateBlobs();
}

void ofxKinectTracker::grabBackground() {
  background.setFromPixels(depthImage.getPixels());
  backgroundSubtract = true;
}

void ofxKinectTracker::subtractBackground() {
  ofPixels & pix = depthImage.getPixels();
	ofPixels & bgPix = background.getPixels();
	ofPixels & d = diff.getPixels();

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
          //TODO: There are two blobs that match, this is rare. How to decide which blob is which?
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
      newBlobs.push_back(*currentBlob);
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
        if(blobs[i].timeSinceLastSeen() > removeAfterSeconds)
        {
          blobs.erase(blobs.begin()+i);
        }

        //TODO: Check for ovelapping blobs;
      }

      blobs[i].setActive(trackedBlob[i]);
  }
}

float ofxKinectTracker::getZHintForBlob(ofxCvBlob blob)
{
  return kinect->getDistanceAt(blob.centroid);
}

//Image Getters
ofxCvColorImage ofxKinectTracker::getColorImage(){
  return colorImg;
}

ofxCvGrayscaleImage ofxKinectTracker::getGrayImage(){
  ofxCvGrayscaleImage grayscale;
  grayscale.allocate(width, height);
  grayscale.setFromPixels(colorImg.getPixels());
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
  kinect->draw(x, y, width, height);
}

void ofxKinectTracker::drawRGB(float x, float y, float scale)
{
  kinect->draw(x, y, width*scale, height*scale);
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
}

void ofxKinectTracker::close(){
  kinect->close();
  delete kinect;
}
