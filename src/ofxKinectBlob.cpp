#include "ofxKinectBlob.h"

ofxKinectBlob::ofxKinectBlob(int id, ofxCvBlob blob, float zHint, float tolerance){
  this->blob = blob;
  this->zHint = zHint;
  this->active = true;
  this->tolerance = tolerance;
  this->id = id;
  this->overlap = false;
}

ofxKinectBlob::~ofxKinectBlob(){

}

void ofxKinectBlob::update(ofxCvBlob blob, float zHint)
{
  direction.x = blob.centroid.x - this->blob.centroid.x;
  direction.y = blob.centroid.y - this->blob.centroid.y;
  direction.z = zHint - this->zHint;

  this->blob = blob;
  this->zHint = zHint;
  this->active = true;
}

float ofxKinectBlob::difference(ofxCvBlob otherBlob, float zHint){
  ofVec3f diff(
            otherBlob.centroid.x - blob.centroid.x,
            otherBlob.centroid.y - blob.centroid.y,
            zHint - this->zHint
  );

  float distance = diff.length();

  if (distance > tolerance)
  {
    return -1;
  }

  ofVec3f expectedLocationDiff = (blob.centroid + direction) - otherBlob.centroid;
  float deviation = expectedLocationDiff.length();
  float areaDiff = abs(otherBlob.area - blob.area);

  float result = distance + deviation/5 + areaDiff;

  return result;
}

void ofxKinectBlob::draw(float x, float y){
  if(active)
  {
    blob.draw(x,y);
  }
}

void ofxKinectBlob::setActive(bool value)
{
  active = value;
  if(active)
  {
    lastSeen = ofGetElapsedTimef();
  }
}

void ofxKinectBlob::setOverlap(bool value)
{
  overlap = value;
}

bool ofxKinectBlob::isActive()
{
  return active;
}

bool ofxKinectBlob::isOverlapping()
{
  return overlap;
}

float ofxKinectBlob::timeSinceLastSeen()
{
  return ofGetElapsedTimef() - lastSeen;
}
