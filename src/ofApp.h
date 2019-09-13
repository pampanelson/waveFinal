#pragma once

#include "ofMain.h"
#include "ofxGui.h"
#include "ofxCv.h"
#include "ofxOpenCv.h"
#include "ofxOsc.h"
#include "ofxSyphon.h"
#include "ofxKinect.h"

// send host (aka ip address)
#define HOST "localhost"
//#define HOST "192.168.0.174"

/// send port
#define PORT 8000
using namespace ofxCv;
using namespace cv;
class ofApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();

		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void mouseEntered(int x, int y);
		void mouseExited(int x, int y);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);
    float myPosToAngle(float x,float y,float cX,float cY);

    
    
    
    ofxOscSender sender;
    
    

    
    int                 width;
    int                 height;
    
    ofVideoGrabber vidGrabber;
    ofPixels videoInverted;
    ofTexture videoTexture;
    ofImage   vidImg;
    int camWidth;
    int camHeight;
    
    
    ofxKinect kinect;

    
    ofxCvColorImage     colorImage;
    ofxCvGrayscaleImage grayImage; // grayscale depth image
    ofxCv::ContourFinder contourFinder;
    ofxCvGrayscaleImage grayThreshNear; // the near thresholded image
    ofxCvGrayscaleImage grayThreshFar; // the far thresholded image
    
    ofPixels previous;
    ofPixels strench;


    
    
    
    ofxPanel gui;
    ofParameter<bool> bThreshWithOpenCV;
    ofParameter<float> minAreaRadius;
    ofParameter<float> maxAreaRadius;
    ofParameter<float> trackingThreshold;
    ofParameter<float> angle;
    ofParameter<int> nearThreshold;
    ofParameter<int> farThreshold;
    ofParameter<int> outRadius;
    ofParameter<int> inRadius;
    
    ofParameter<int> detectCircleCenterX;
    ofParameter<int> detectCircleCenterY;
    
    ofParameter<int> detectStrenchrX;
    
    ofParameter<bool> bSendingOSC;
    ofParameter<bool> bTracking;
    ofParameter<bool> bFlip;

    
    
    
    int trackingDataSize = 32;
    vector<float> trackingData;
    
    
    
};
