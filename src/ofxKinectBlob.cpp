#include "ofxKinectBlob.h"

ofxKinectBlob::ofxKinectBlob(int id, ofxCvBlob blob, float zHint, float tolerance){
  this->blob = blob;
  this->zHint = zHint;
  this->active = true;
  this->tolerance = tolerance;
  this->id = id;
  this->overlap = false;
  speed = 0;
}

ofxKinectBlob::~ofxKinectBlob(){

}

void ofxKinectBlob::update(ofxCvBlob blob, float zHint)
{
  direction.x = blob.centroid.x - this->blob.centroid.x;
  direction.y = blob.centroid.y - this->blob.centroid.y;
  direction.z = zHint - this->zHint;

  speed = direction.length();

  this->blob = blob;
  this->zHint = zHint;
  this->active = true;
}

bool ofxKinectBlob::intersects(const ofxKinectBlob otherBlob){
  ofRectangle intersection = blob.boundingRect.getIntersection(otherBlob.blob.boundingRect);
  return intersection.width != 0 || intersection.height != 0 || intersection.x != 0 || intersection.y != 0;
}

ofRectangle ofxKinectBlob::getIntersection(ofxKinectBlob otherBlob) {
  return blob.boundingRect.getIntersection(otherBlob.blob.boundingRect);
}

float ofxKinectBlob::difference(ofxCvBlob otherBlob, float zHint){
  ofVec3f diff(
            otherBlob.centroid.x - blob.centroid.x,
            otherBlob.centroid.y - blob.centroid.y,
            zHint - this->zHint
  );

  if(this->blob.boundingRect.inside(otherBlob.centroid))
  {
    return 0;
  }

  float distance = diff.length();

  ofVec3f expectedLocationDiff = (blob.centroid + direction) - otherBlob.centroid;
  float deviation = expectedLocationDiff.length();
  float areaDiff = abs(otherBlob.area - blob.area);

  float result = distance + deviation/5 + areaDiff;

  if (distance > tolerance)
  {
    return -1;
  }
  else
  {
    return result;
  }
}

void ofxKinectBlob::setTolerance(float value)
{
  tolerance = value;
}

float ofxKinectBlob::getTolerance()
{
  return tolerance;
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
