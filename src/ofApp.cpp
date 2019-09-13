#include "ofApp.h"



float ofApp::myPosToAngle(float x,float y,float centerX,float centerY){
    float res;
    // center on (320,480)
    res = atan(
               (centerY - y) // always > 0 for top lefter (0,0)
               /
               (centerX - x) // if > 0 means point on the left of center
               // if < 0 point on the right of center
               
               )/PI;
    
    
    // clock direction of angle from -x axis ///////////////  *********** IMPORTANT **********
    // so , res should always < 1 aka. PI
    if(res < 0){
        res = 1. + res;
    }
    
    return res;
}




//--------------------------------------------------------------
void ofApp::setup(){
    ofSetFrameRate(30);
    ofSetVerticalSync(true);
    
    ofSetBackgroundColor(0, 0, 0);
    
    
    // open an outgoing connection to HOST:PORT
    sender.setup(HOST, PORT);
    
    
    camWidth = 640;  // try to grab at this size.
    camHeight = 480;
    
    //get back a list of devices.
//    vector<ofVideoDevice> devices = vidGrabber.listDevices();
//
//    for(size_t i = 0; i < devices.size(); i++){
//        if(devices[i].bAvailable){
//            //log the device
//            ofLogNotice() << devices[i].id << ": " << devices[i].deviceName;
//        }else{
//            //log the device and note it as unavailable
//            ofLogNotice() << devices[i].id << ": " << devices[i].deviceName << " - unavailable ";
//        }
//    }
//
//    vidGrabber.setDeviceID(0);
//    vidGrabber.setDesiredFrameRate(30);
//    vidGrabber.initGrabber(camWidth, camHeight);
//
//    vidImg.allocate(camWidth, camHeight, OF_IMAGE_COLOR);

    
    
    // enable depth->video image calibration
    kinect.setRegistration(true);
    
    kinect.init();
    //kinect.init(true); // shows infrared instead of RGB video image
    //kinect.init(false, false); // disable video image (faster fps)
    
    kinect.open();        // opens first available kinect
    //kinect.open(1);    // open a kinect by id, starting with 0 (sorted by serial # lexicographically))
    //kinect.open("A00362A08602047A");    // open a kinect using it's unique serial #
    
    //    cout << kinect.width << "," << kinect.height << endl;
    grayImage.allocate(kinect.width, kinect.height);
    grayThreshNear.allocate(kinect.width, kinect.height);
    grayThreshFar.allocate(kinect.width, kinect.height);
    
    
    imitate(previous, grayImage);
    imitate(strench, grayImage);
    
    nearThreshold = 230;
    farThreshold = 70;
    bThreshWithOpenCV = true;
    
    //    ofSetFrameRate(60);
    
    // zero the tilt on startup
    angle = 0;
    kinect.setCameraTiltAngle(angle);
    
    // print the intrinsic IR sensor values
    if(kinect.isConnected()) {
        ofLogNotice() << "sensor-emitter dist: " << kinect.getSensorEmitterDistance() << "cm";
        ofLogNotice() << "sensor-camera dist:  " << kinect.getSensorCameraDistance() << "cm";
        ofLogNotice() << "zero plane pixel size: " << kinect.getZeroPlanePixelSize() << "mm";
        ofLogNotice() << "zero plane dist: " << kinect.getZeroPlaneDistance() << "mm";
    }
    

//
//    colorImage.allocate(camWidth,camHeight);
//
//    grayImage.allocate(camWidth, camHeight);
//    grayThreshNear.allocate(camWidth, camHeight);
//    grayThreshFar.allocate(camWidth, camHeight);
//
    
    
    
    
    width = 3840;
    height = 960;
    
    
    
    
    for (int i = 0; i < trackingDataSize; i++) {
        trackingData.push_back(-0.1);
    }
    
    
    
    gui.setup();
    
    gui.add(bSendingOSC.set("Sending osc",false));
    gui.add(bTracking.set("Tracking",false));
    
    gui.add(minAreaRadius.set("min area",1,1,300));
    gui.add(maxAreaRadius.set("max area",10,1,800));
    gui.add(trackingThreshold.set("tracking thresh",1,1,100));
    gui.add(angle.set("angle",10,1,180));

    gui.add(nearThreshold.set("near",230,1,255));
    gui.add(farThreshold.set("far",70,1,255));
    gui.add(bThreshWithOpenCV.set("use opencv", false));
    
    gui.add(detectCircleCenterX.set("circle x",0,0,640));
    gui.add(detectCircleCenterY.set("circle y",0,0,640));
    gui.add(detectStrenchrX.set("strench",0,0,640));
    gui.add(outRadius.set("out r",10,1,480));
    gui.add(inRadius.set("in r",0,1,480));
    
    
    if (!ofFile("settings.xml"))
        gui.saveToFile("settings.xml");
    
    gui.loadFromFile("settings.xml");
    
}

//--------------------------------------------------------------
void ofApp::update(){
    for (int i = 0; i < trackingDataSize; i++) {
        trackingData[i] = -0.1;
    }
    
    
    
    kinect.setCameraTiltAngle(angle);
    
    
    kinect.update();
    
    // there is a new frame and we are connected
    if(kinect.isFrameNew()) {
        
        // load grayscale depth image from the kinect source
        grayImage.setFromPixels(kinect.getDepthPixels());
        
        
        
        if(bFlip){
            
            grayImage.mirror(false, true);
        }
        
        
        
        // we do two thresholds - one for the far plane and one for the near plane
        // we then do a cvAnd to get the pixels which are a union of the two thresholds
        if(bThreshWithOpenCV) {
            grayThreshNear = grayImage;
            grayThreshFar = grayImage;
            grayThreshNear.threshold(nearThreshold, true);
            grayThreshFar.threshold(farThreshold);
            cvAnd(grayThreshNear.getCvImage(), grayThreshFar.getCvImage(), grayImage.getCvImage(), NULL);
        } else {
            
            // or we do it ourselves - show people how they can work with the pixels
            ofPixels & pix = grayImage.getPixels();
            int numPixels = pix.size();
            for(int i = 0; i < numPixels; i++) {
                if(pix[i] < nearThreshold && pix[i] > farThreshold) {
                    pix[i] = 255;
                } else {
                    pix[i] = 0;
                }
            }
            
        }
        
        // update the cv images
        grayImage.flagImageChanged();
        // get diff
        //        absdiff(grayImage, previous, diff);
        //        diff.update();
        //        copy(grayImage, previous);
        //        blur(diff,10);
        
        grayImage.blur();
        
        contourFinder.setMinAreaRadius(minAreaRadius);
        contourFinder.setMaxAreaRadius(maxAreaRadius);
        contourFinder.setThreshold(trackingThreshold);
        // wait for half a second before forgetting something
        //        contourFinder.getTracker().setPersistence(trackingPersistence);
        // an object can move up to 32 pixels per frame
        //        contourFinder.getTracker().setMaximumDistance(maxDistance);
        
        
        
        // tracking
        // judege if need to sending signal for make waves
        
        // get all tracking contour centroid average value
        contourFinder.findContours(grayImage);
        
        
 
        // update the cv images
        grayImage.flagImageChanged();
        
        // get diff
        //    absdiff(grayImage, previous, diff);
        //    diff.update();
        copy(grayImage, previous);
        //    blur(diff,10);
        
        
        
        grayImage.blur();
        
        
        for (int i = 0; i < previous.getWidth(); i++) {
            for (int j = 0; j < previous.getHeight(); j++) {
                ofColor c = previous.getColor(i, j);
                
                int x = i;
                int y = j;
                int strenchedX;
                if(x > detectCircleCenterX && x + detectStrenchrX < 640){
                    strenchedX = x + detectStrenchrX;
                    
                }
                
                
                if(x < detectCircleCenterX && x - detectStrenchrX > 0){
                    strenchedX = x - detectStrenchrX;
                }
                
                
                
                // drop aren between strenched
                if(abs(x - detectCircleCenterX) <= detectStrenchrX){
                    strench.setColor(x, y, ofColor(0));
                    
                }
                else{
                    strench.setColor(strenchedX, y, c);
                    
                }
                
                
                
            }
        }
        
        
        // remove out of detecting area
        if(bTracking){
            for (int i = 0; i < strench.getWidth(); i++) {
                for (int j = 0; j < strench.getHeight(); j++) {
                    //            // detect to remove other then circle base original coordination
                    float dist = sqrt((i - detectCircleCenterX)*(i - detectCircleCenterX) + (j-detectCircleCenterY)*(j-detectCircleCenterY));
                    
                    if(dist > inRadius && dist < outRadius){
                        ofColor c = strench.getColor(i, j);
                        
                        //                cout << ofToString(c) << endl;
                        if(c.r < 100){
                            float angle = myPosToAngle(i,j,detectCircleCenterY,detectCircleCenterY);
                            
                            int angleIndex = floor((trackingDataSize - 1)*angle);
                            trackingData[angleIndex] = 1.0;
                        }
                    }
                    
                    
                    
                }
            }
            
        }

        grayImage.setFromPixels(strench);
        grayImage.flagImageChanged();
        
    
    
 

    if(bSendingOSC){
        // prepare data for osc send ----------------------------------------
        
        ofxOscMessage m;
        ofxOscMessage m1;
        //        m.setAddress("/composition/selectedclip/video/effects/pwl00/effect/float1");
        //        m.addFloatArg(ofMap(ofGetMouseX(), 0, ofGetWidth(), 0.f, 1.f, true));
        //        //    m.addFloatArg(ofMap(ofGetMouseY(), 0, ofGetHeight(), 0.f, 1.f, true));
        //        sender.sendMessage(m, false);
        //        m.clear();
        
        string data;
        
        for(int i = 0;i<trackingDataSize;i++){
            //            oscTrackingData[i] = -0.1;
            //            oscTrackingData[i] += i * 0.1;
            data += ofToString(trackingData[i]);
            if(i != trackingDataSize - 1){
                data += ",";
            }
            
        }
        
        
//        cout << data << endl;
        
        // debug ================
        //        m.setAddress("/composition/selectedclip/video/effects/pwaveword/effect/osctextdata0");
        m.setAddress("/composition/selectedclip/video/effects/pwaveword/effect/osctextdata0");
        m.addStringArg(data);
        sender.sendMessage(m,false);
        
        
        m1.setAddress("/composition/selectedclip/video/effects/pwaveline/effect/oscdataline0");
        m1.addStringArg(data);
        sender.sendMessage(m1,false);
        
        m1.clear();
        
        
    }
        
        //
    }
}

//--------------------------------------------------------------
void ofApp::draw(){
        grayImage.draw(0,0,640,480);
    //    diff.draw(640, 480);
    
    //    vidGrabber.getTexture().draw(0, 0);
    //    contourFinder.draw();
    //    colorImage.draw(0, 480);
    
    
    ofSetColor(0, 0, 255,50);
    ofDrawCircle(detectCircleCenterX, detectCircleCenterY, inRadius);
    
    ofSetColor(255, 0,0,50);
    ofDrawCircle(detectCircleCenterX, detectCircleCenterY, outRadius);
    
    ofSetColor(255);
    gui.draw();

}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){

}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}
