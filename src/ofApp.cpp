#include "ofApp.h"



float ofApp::myPosToAngle(float x,float y,float centerX,float centerY){
    float res;
    
    res = atan(
               (y - centerY) // always > 0 for top lefter (0,0)
               /
               (centerX - x) // if > 0 means point on the left of center
               // if < 0 point on the right of center
               
               )/PI;
    
    
    // clock direction of angle from -x axis ///////////////  *********** IMPORTANT **********
    // so , res should always < 1 aka. PI
    if(res < 0){
        res = 1. + res;
    }
    
    res = ofClamp(res, 0, 1);
    return res;
}




//--------------------------------------------------------------
void ofApp::setup(){
    ofSetFrameRate(30);
    ofSetVerticalSync(true);
    
    ofSetBackgroundColor(0, 0, 0);
    
    
    // open an outgoing connection to HOST:PORT
    sender.setup(HOST, PORT);
    
    
    // enable depth->video image calibration
    kinect.setRegistration(true);
    
    kinect.init();
    //kinect.init(true); // shows infrared instead of RGB video image
    kinect.init(false, false); // disable video image (faster fps)

    
    kinect.open();        // opens first available kinect
    //kinect.open(1);    // open a kinect by id, starting with 0 (sorted by serial # lexicographically))
    //kinect.open("A00362A08602047A");    // open a kinect using it's unique serial #
    
    //    cout << kinect.width << "," << kinect.height << endl;
    grayImage.allocate(kinect.width, kinect.height);
    grayImage1.allocate(kinect.width, kinect.height);
    grayThreshNear.allocate(kinect.width, kinect.height);
    grayThreshFar.allocate(kinect.width, kinect.height);
    
    
    imitate(previous, grayImage);
    imitate(strench, grayImage);
    
    nearThreshold = 230;
    farThreshold = 70;
    bThreshWithOpenCV = true;
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

    
    
    for (int i = 0; i < trackingDataSize; i++) {
        trackingData.push_back(0.0);
        oscData.push_back(0.0);
    }
    
    
    
    gui.setup();
    
    gui.add(bSendingOSC.set("Sending osc",false));
    gui.add(bTracking.set("Tracking",false));
    gui.add(bFlip.set("Flip",false));

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
    
    gui.add(wavePowerMax.set("wave power max",0,.1,5));
    gui.add(wavePowerDelta.set("wave power delta",0,0.01,2));
    gui.add(waveThreshold.set("wave threshold",10,1,2000));
    gui.add(bUseRandomShape.set("random shape",false));
    gui.add(randomWavePowerMaxMin.set("random power max min",0,.1,5));
    gui.add(randomWavePowerDeltaMin.set("random power delta minn",0,0.01,1));

    
    if (!ofFile("settings.xml"))
        gui.saveToFile("settings.xml");
    
    gui.loadFromFile("settings.xml");
    
}

//--------------------------------------------------------------
void ofApp::update(){
    for (int i = 0; i < trackingDataSize; i++) {
        trackingData[i] = 0.0;
    }
    
    changingWaveShapeMarker = 0.0;
    
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
        
//        contourFinder.setMinAreaRadius(minAreaRadius);
//        contourFinder.setMaxAreaRadius(maxAreaRadius);
//        contourFinder.setThreshold(trackingThreshold);
        // wait for half a second before forgetting something
        //        contourFinder.getTracker().setPersistence(trackingPersistence);
        // an object can move up to 32 pixels per frame
        //        contourFinder.getTracker().setMaximumDistance(maxDistance);
        
    
        // get all tracking contour centroid average value
//        contourFinder.findContours(grayImage);
        
    
        
        // get diff
        //    absdiff(grayImage, previous, diff);
        //    diff.update();
        copy(grayImage, previous);
        grayImage1 = grayImage;
        //    blur(diff,10);
        
        
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
                        if(c.r > 10){
                            
                            // if on right strneched area, angle to center x + strenchX
                            // if on left strenched arean , angle to center x - strechanx
                            
                            
                            int cX ;
                            if(i > detectCircleCenterX){
                                cX = detectCircleCenterX + detectStrenchrX;
                            }
                            if(i < detectCircleCenterX){
                                cX = detectCircleCenterX - detectStrenchrX;
                            }
                            
                            
                            float angle = myPosToAngle(i,j,cX,detectCircleCenterY);
                            angle = ofClamp(angle, 0, 1);
                            
                            angle = 1. - angle;// why???????????????? but work
                            int angleIndex = floor(trackingDataSize*angle);
                            angleIndex = ofClamp(angleIndex, 0, trackingDataSize-1);
                            trackingData[angleIndex] += 1;
                        }
                    }



                }
            }
            
        }

        grayImage1.setFromPixels(strench);
        grayImage1.flagImageChanged();
        
    
    
        // analyse end, process and prepare data to send
        
        
        for (int i = 0; i < trackingDataSize; i++) {
            if(trackingData[i] > waveThreshold){
                oscData[i] += currentWaveDelta;
                
            }else{
                oscData[i] -= currentWaveDelta;
            }
            
            if(oscData[i] > currentWavePowerMax){
                oscData[i] = currentWavePowerMax;
            }
            
            if(oscData[i] < 0){
                oscData[i] = 0;
            }
            
            //summup value for check if no wave on , and have chance to change wavw shape
            
            changingWaveShapeMarker += oscData[i];
            
        }
        
        
        // if change wave shape when have chance ======================
        
        if(changingWaveShapeMarker == 0 && bUseRandomShape){
            
            currentWaveDelta = ofRandom(randomWavePowerDeltaMin, wavePowerDelta);
            currentWavePowerMax = ofRandom(randomWavePowerMaxMin,wavePowerMax);
        }
        else{
            currentWavePowerMax = wavePowerMax;
            currentWaveDelta = wavePowerDelta;
        }

        
        
        
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
            data += ofToString(oscData[i]);
            if(i != trackingDataSize - 1){
                data += ",";
            }
            
        }
        
        
//        cout << data << endl;
        
        // debug ================
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
    grayImage1.draw(0,0,640,480);
    grayImage.draw(640,0,640,480);
    //    diff.draw(640, 480);
    
    
    
    ofSetColor(0, 0, 255,50);
    ofDrawCircle(detectCircleCenterX, detectCircleCenterY, inRadius);
    
    ofSetColor(255, 0,0,50);
    ofDrawCircle(detectCircleCenterX, detectCircleCenterY, outRadius);
    
    
    // draw osc data for monitor
    for (int i = 0; i < trackingDataSize; i++) {
        ofSetColor(oscData[i]*30 + 10, 0, 0);
        int w = 30;
        int h = 40;
        
        ofDrawRectangle(10 + i*w, 515, w, h);
        ofSetColor(255, 0, 0);
        ofDrawBitmapString(ofToString(i),10 + i*w,500);

    }
    
    
    
    ofSetColor(255);
    gui.draw();

}
//--------------------------------------------------------------
void ofApp::exit(){
    
    kinect.setCameraTiltAngle(0); // zero the tilt on exit
    kinect.close();
    
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
