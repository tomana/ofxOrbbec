#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup() {
    // Set to verbose logging for debugging
    ofSetLogLevel(OF_LOG_WARNING);
    ofSetFrameRate(30);  // Limit frame rate to match camera
    ofSetVerticalSync(true);  // Enable vertical sync
    
    ofLogNotice("ofApp::setup") << "Starting setup...";

    // Log which GPU/driver OpenGL is using
    const GLubyte* glVendor   = glGetString(GL_VENDOR);
    const GLubyte* glRenderer = glGetString(GL_RENDERER);
    const GLubyte* glVersion  = glGetString(GL_VERSION);
    const GLubyte* glslVer    = glGetString(GL_SHADING_LANGUAGE_VERSION);
    ofLogNotice("GPU") << "GL_VENDOR   : " << (glVendor   ? reinterpret_cast<const char*>(glVendor)   : "?");
    ofLogNotice("GPU") << "GL_RENDERER : " << (glRenderer ? reinterpret_cast<const char*>(glRenderer) : "?");
    ofLogNotice("GPU") << "GL_VERSION  : " << (glVersion  ? reinterpret_cast<const char*>(glVersion)  : "?");
    if(glslVer){
        ofLogNotice("GPU") << "GLSL        : " << reinterpret_cast<const char*>(glslVer);
    }

    auto deviceInfo = ofxOrbbecCamera::getDeviceList(true); 
    
    if (deviceInfo.empty()) {
        ofLogError("ofApp::setup") << "No Orbbec devices found!";
        return;
    }
    ofLogNotice("ofApp::setup") << "Found " << deviceInfo.size() << " device(s)";

    settings.bColor = true; 
    settings.bDepth = true; 
    settings.bPointCloud = true; 
    
    // Femto Mega depth settings
    settings.depthFrameSize.format = OB_FORMAT_Y16;
    settings.depthFrameSize.requestWidth = 640;  // NFOV mode (640x576)
    settings.depthFrameSize.frameRate = 30;
    
    // Femto Mega color settings (2MP)
    settings.colorFrameSize.format = OB_FORMAT_YUYV;  // Use native YUYV format instead of MJPG
    settings.colorFrameSize.requestWidth = 1280;  // Lower resolution for stability
    settings.colorFrameSize.frameRate = 30;  // Match depth frame rate
    
    // Temporarily disable RGB point cloud to test depth-only mode
    settings.bPointCloudRGB = false;  // Testing depth-only point cloud first
    
    try {
        if(!orbbecCam.open(settings)) {
            ofLogError("ofApp::setup") << "Failed to open Femto Mega camera";
            return;
        }
        
        ofLogNotice("ofApp::setup") << "Camera opened successfully";
        
        // Wait a bit for the camera to fully initialize
        ofSleepMillis(1000);
        
        // Do a test update to ensure camera is working
        orbbecCam.update();
        
        if (!orbbecCam.isFrameNewColor() && !orbbecCam.isFrameNewDepth()) {
            ofLogWarning("ofApp::setup") << "Camera opened but no frames received in first update";
        }
        
    } catch(const ob::Error &e) {
        ofLogError("ofApp::setup") << "Femto Mega error: " << e.getMessage() << "\nTip: Femto Mega requires matching depth/color framerates";
    } catch(const std::exception &e) {
        ofLogError("ofApp::setup") << "Error: " << e.what();
    }
}

//--------------------------------------------------------------
void ofApp::update() {
    if (!bAcquisitionActive) {
        return; // Skip update when acquisition is stopped
    }

    static bool firstUpdate = true;
    try {
        if (firstUpdate) {
            ofLogNotice("ofApp::update") << "First update called";
            firstUpdate = false;
        }

        ofLogVerbose("ofApp::update") << "Starting camera update";
        orbbecCam.update();
        ofLogVerbose("ofApp::update") << "Camera update complete";
        
        if (orbbecCam.isFrameNewColor()) {
            ofLogVerbose("ofApp::update") << "New color frame detected";
            auto pix = orbbecCam.getColorPixels();
            if (pix.isAllocated()) {
                if (!outputTex.isAllocated() || 
                    outputTex.getWidth() != pix.getWidth() || 
                    outputTex.getHeight() != pix.getHeight()) {
                    ofLogNotice("ofApp::update") << "Allocating color texture: " << pix.getWidth() << "x" << pix.getHeight();
                    outputTex.allocate(pix);
                }
                outputTex.loadData(pix);
            } else {
                ofLogWarning("ofApp::update") << "Color pixels not allocated";
            }
        }

        static int checkCounter = 0;
        if(checkCounter++ < 5) {
            std::cerr << ">>>>> isFrameNewDepth: " << orbbecCam.isFrameNewDepth() << std::endl;
        }
        
        if (orbbecCam.isFrameNewDepth()) {
            //std::cerr << ">>>>> New depth frame detected!" << std::endl;
            ofLogVerbose("ofApp::update") << "New depth frame detected";
            auto depthPix = orbbecCam.getDepthPixels();
            if (depthPix.isAllocated()) {
                if (!outputTexDepth.isAllocated() || 
                    outputTexDepth.getWidth() != depthPix.getWidth() || 
                    outputTexDepth.getHeight() != depthPix.getHeight()) {
                    ofLogNotice("ofApp::update") << "Allocating depth texture: " << depthPix.getWidth() << "x" << depthPix.getHeight();
                    outputTexDepth.allocate(depthPix);
                }
                outputTexDepth.loadData(depthPix);
            } else {
                ofLogWarning("ofApp::update") << "Depth pixels not allocated";
            }

            try {
                ofLogVerbose("ofApp::update") << "Getting point cloud mesh";
                mPointCloudMesh = orbbecCam.getPointCloudMesh();
                static int logCounter = 0;
                if(logCounter++ < 10) {
                    std::cerr << "##### Point cloud mesh vertices: " << mPointCloudMesh.getNumVertices() 
                              << " indices: " << mPointCloudMesh.getNumIndices() << std::endl;
                }
            } catch (const std::exception& e) {
                ofLogError("ofApp::update") << "Error getting point cloud: " << e.what();
            }
        }
    } catch (const std::exception& e) {
        ofLogError("ofApp::update") << "Error in update: " << e.what();
    }
    
    ofLogVerbose("ofApp::update") << "Update complete";
}

//--------------------------------------------------------------
void ofApp::exit(){
    try {
        ofLogNotice("ofApp::exit") << "Closing camera...";
        orbbecCam.close();
        ofLogNotice("ofApp::exit") << "Camera closed";
        
        // Clear textures
        if (outputTex.isAllocated()) {
            outputTex.clear();
        }
        if (outputTexDepth.isAllocated()) {
            outputTexDepth.clear();
        }
        
        // Clear mesh
        mPointCloudMesh.clear();
        
    } catch (const std::exception& e) {
        ofLogError("ofApp::exit") << "Error during cleanup: " << e.what();
    }
}

//--------------------------------------------------------------
void ofApp::draw(){
    static bool firstDraw = true;
    try {
        if (firstDraw) {
            ofLogNotice("ofApp::draw") << "First draw called";
            firstDraw = false;
        }

        ofBackground(10); 

        if (!ofGetMousePressed()) {
            ofSetColor(255, 255);
            
            // Draw color texture if available
            if (outputTex.isAllocated()) {
                ofLogVerbose("ofApp::draw") << "Drawing color texture: " << outputTex.getWidth() << "x" << outputTex.getHeight();
                ofPushMatrix();
                ofScale(0.25, 0.25);  // More precise scaling
                outputTex.draw(0, 0);
                ofPopMatrix();
            }
            
            // Draw depth texture if available
            if (outputTexDepth.isAllocated()) {
                ofLogVerbose("ofApp::draw") << "Drawing depth texture: " << outputTexDepth.getWidth() << "x" << outputTexDepth.getHeight();
                float yPos = outputTex.isAllocated() ? outputTex.getHeight()/4 : 0;
                ofPushMatrix();
                ofTranslate(0, yPos);
                ofScale(0.5, 0.5);  // More precise scaling
                outputTexDepth.draw(0, 0);
                ofPopMatrix();
            }
        }

        // Draw point cloud if available and valid
        if (mPointCloudMesh.getNumVertices() > 0 && mPointCloudMesh.getNumIndices() > 0) {
            ofLogVerbose("ofApp::draw") << "Drawing point cloud with " << mPointCloudMesh.getNumVertices() << " vertices";
            ofSetColor(255);
            ofEnableDepthTest();
            mCam.begin();
            try {
                ofPushMatrix();
                ofTranslate(0, -300, 1000);
                mPointCloudMesh.drawVertices();  // Draw only vertices to be safer
                ofPopMatrix();
            } catch (const std::exception& e) {
                ofLogError("ofApp::draw") << "Error drawing point cloud: " << e.what();
            }
            mCam.end();
            ofDisableDepthTest();
        }
    } catch (const std::exception& e) {
        ofLogError("ofApp::draw") << "Error in draw: " << e.what();
    }

    ofLogVerbose("ofApp::draw") << "Draw complete";
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    if( key == 'c' ){
        orbbecCam.close();
    }
    else if( key == ' ' ){
        // Toggle acquisition on/off
        bAcquisitionActive = !bAcquisitionActive;
        ofLogNotice("ofApp::keyPressed") << "Acquisition " << (bAcquisitionActive ? "started" : "stopped");
    }
    else if( key == OF_KEY_RETURN ){
        // Save screenshot of entire window
        ofImage screenshot;
        screenshot.grabScreen(0, 0, ofGetWidth(), ofGetHeight());
        string timestamp = ofGetTimestampString();
        string filename = "screenshot_" + timestamp + ".png";
        screenshot.save(filename);
        ofLogNotice("ofApp::keyPressed") << "Saved screenshot: " << filename;
    }
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
