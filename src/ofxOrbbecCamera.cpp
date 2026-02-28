#include "ofxOrbbecCamera.h"
#include <libobsensor/hpp/Utils.hpp>

std::vector < std::shared_ptr<ob::DeviceInfo> > ofxOrbbecCamera::getDeviceList(bool bNetIncDevices){
    std::vector<std::shared_ptr<ob::DeviceInfo> > dInfo; 

    auto tCtx = std::make_shared<ob::Context>(); //ofxOrbbecCamera::getContext();
    if( bNetIncDevices ){
		tCtx->enableNetDeviceEnumeration(true); 
	}

    // Query the list of connected devices
    auto devList = tCtx->queryDeviceList();
    
    // Get the number of connected devices
    int devCount = devList->deviceCount();
    ofLogNotice("ofxOrbbecCamera::getDeviceList()") << "Found " << devCount << " devices" << std::endl;
    
    // traverse the device list and create a pipe
    for(int i = 0; i < devCount; i++) {
        auto dev  = devList->getDevice(i);
        auto info = dev->getDeviceInfo();

        ofLogNotice("ofxOrbbecCamera::getDeviceList()") << "["<< i <<"] device is " << info->name() << " serial: " << info->serialNumber() << std::endl; 

        dInfo.push_back(info);
    }

    return dInfo; 
}

//Class functions 
ofxOrbbecCamera::~ofxOrbbecCamera(){
    close();
    #ifdef OFXORBBEC_DECODE_H264_H265
        if(bInitOneTime){
            avcodec_close(codecContext264);
            av_free(codecContext264);

            avcodec_close(codecContext265);
            av_free(codecContext265);
            bInitOneTime = false; 
        }
    #endif 
}

void ofxOrbbecCamera::close(){
    clear();
}

void ofxOrbbecCamera::clear(){

    if( isThreadRunning() ){
        waitForThread(true, 2000); 
    }
    try{
		if( mPipe ){	
			mPipe->stop();
			mPipe.reset();
			pointCloud.reset();
        }
    }catch(ob::Error &e) {
        ofLogError("ofxOrbbecCamera::clear") << "function:" << e.getName() << "\nargs:" << e.getArgs() << "\nmessage:" << e.getMessage() << "\ntype:" << e.getExceptionType();
    }

    mTemporalFilter.reset();
    mSpatialFilter.reset();
    mHoleFillingFilter.reset();
    mNoiseRemovalFilter.reset();

    mCurrentSettings = ofxOrbbec::Settings();
    bNewFrameColor = bNewFrameDepth = bNewFrameIR = false;
    mInternalColorFrameNo = mInternalDepthFrameNo = 0;
    mExtColorFrameNo = mExtDepthFrameNo = 0;

    mPipe.reset();
	ctxLocal.reset();
    bConnected = false; 
    mTimeSinceFrame = 0.0;
}

bool ofxOrbbecCamera::open(ofxOrbbec::Settings aSettings){
    clear(); 
	
	ob::Context::setLoggerToFile(OB_LOG_SEVERITY_OFF, "log.txt");
	ob::Context::setLoggerToConsole(OB_LOG_SEVERITY_INFO);

	ctxLocal = std::make_shared<ob::Context>();
    auto tCtx = ctxLocal;

    std::shared_ptr<ob::Device> device;

    //need depth frames for point cloud
    if( aSettings.bPointCloud && !aSettings.bDepth ){
        aSettings.bDepth = true; 
    }

    mCurrentSettings = aSettings; 

    if( aSettings.ip != ""){
        try{
            device = tCtx->createNetDevice(aSettings.ip.c_str(), 8090);
        }catch(ob::Error &e) {
            ofLogError("ofxOrbbecCamera::open") << "function:" << e.getName() << "\nargs:" << e.getArgs() << "\nmessage:" << e.getMessage() << "\ntype:" << e.getExceptionType();
        }
        if(!device){
            return false; 
        }
    }else{

        // Query the list of connected devices
         auto devList = tCtx->queryDeviceList();
        
        // Get the number of connected devices
        int devCount = devList->deviceCount();

        bool openWithSerial = aSettings.deviceSerial != "";

        // traverse the device list and create a pipe
        std::ostringstream os{""};
        for(int i = 0; i < devCount; i++) {
            // Get the device and create the pipeline
            auto dev  = devList->getDevice(i);
            auto info = dev->getDeviceInfo();
            os << "[" << i << "] device is " << info->name() << " serial: " << info->serialNumber() << std::endl;

            if( openWithSerial ){
                std::string serialStr(info->serialNumber());
                if( aSettings.deviceSerial == serialStr ){
                    device = dev;
                    break; 
                }
            }else{
                if( aSettings.deviceID == i ){
                    device = dev; 
                    break; 
                }
            }
        }
        ofLogNotice("ofxOrbbecCamera::open") << "\n" << os.str();
    }

    if( device ){
        // pass in device to create pipeline
        mPipe = std::make_shared<ob::Pipeline>(device);
    
        if( mPipe ){

             // Create Config for configuring Pipeline work
            std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();

            std::shared_ptr<ob::StreamProfile> depthProfile;
            std::shared_ptr<ob::StreamProfile> colorProfile;

            if( aSettings.bDepth ){
                // Get the depth camera configuration list
                auto depthProfileList = mPipe->getStreamProfileList(OB_SENSOR_DEPTH);

                if( mCurrentSettings.depthFrameSize.requestWidth > 0){
                    try {
                        auto requestType = mCurrentSettings.depthFrameSize;

                        // Find the corresponding profile according to the specified format
                        depthProfile = depthProfileList->getVideoStreamProfile(requestType.requestWidth, requestType.requestHeight, requestType.format, requestType.frameRate);
                    }
                    catch(ob::Error &e) {
                        ofLogWarning("ofxOrbbecCamera::open") << " couldn't open depth with requested dimensions / format - using default "; 
                        depthProfile = depthProfileList->getProfile(0);
                    }

                }else{  
                    depthProfile = depthProfileList->getProfile(0);
                }
                
                // enable depth stream
                config->enableStream(depthProfile);
            }

            if( aSettings.bColor ){
                // Get the color camera configuration list
                auto colorProfileList = mPipe->getStreamProfileList(OB_SENSOR_COLOR);

                if( mCurrentSettings.colorFrameSize.requestWidth > 0){
                    try {
                        auto requestType = mCurrentSettings.colorFrameSize;

                        // Find the corresponding profile according to the specified format
                        colorProfile = colorProfileList->getVideoStreamProfile(requestType.requestWidth, requestType.requestHeight, requestType.format, requestType.frameRate);
                    }
                    catch(ob::Error &e) {
                        ofLogWarning("ofxOrbbecCamera::open") << " couldn't open color with requested dimensions / format - using default "; 
                        colorProfile = colorProfileList->getProfile(0);
                    }

                }else{  
                    colorProfile = colorProfileList->getProfile(0);
                }
                
                // enable color stream
                config->enableStream(colorProfile);
            }

            // Enable D2C alignment when requested by bPointCloudRGB or bAlignD2C
            bool needD2C = (aSettings.bPointCloud && aSettings.bColor && aSettings.bPointCloudRGB)
                         || (aSettings.bAlignD2C && aSettings.bColor);
            if( needD2C ){
                // Try find supported depth to color align hardware mode profile
                auto depthProfileList = mPipe->getD2CDepthProfileList(colorProfile, ALIGN_D2C_HW_MODE);
                if(depthProfileList->count() > 0) {
                    config->setAlignMode(ALIGN_D2C_HW_MODE);
                    ofLogNotice("ofxOrbbecCamera") << "D2C alignment: hardware mode";
                }
                else {
                    // Try find supported depth to color align software mode profile
                    auto depthProfileList = mPipe->getD2CDepthProfileList(colorProfile, ALIGN_D2C_SW_MODE);
                    if(depthProfileList->count() > 0) {
                        config->setAlignMode(ALIGN_D2C_SW_MODE);
                        ofLogNotice("ofxOrbbecCamera") << "D2C alignment: software mode";
                    }else{
                        config->setAlignMode(ALIGN_DISABLE);
                        ofLogWarning("ofxOrbbecCamera") << "D2C alignment: no supported mode found";
                    }
                }
            } else if( aSettings.bPointCloud ){
                config->setAlignMode(ALIGN_DISABLE);
            }
            
            if( aSettings.bIMU ) {
                auto gyroSensor = device->getSensorList()->getSensor(OB_SENSOR_GYRO);
                if(gyroSensor) {
                    auto profile = gyroSensor->getStreamProfileList()->getProfile(OB_PROFILE_DEFAULT);
                    config->enableStream(profile);
                } else {
                    ofLogWarning("ofxOrbbecCamera::open") << "IMU flag is true, but gyro sensor not supported";
                }
                auto accelSensor = device->getSensorList()->getSensor(OB_SENSOR_ACCEL);
                if(accelSensor) {
                    auto profile = accelSensor->getStreamProfileList()->getProfile(OB_PROFILE_DEFAULT);
                    config->enableStream(profile);
                } else {
                    ofLogWarning("ofxOrbbecCamera::open") << "IMU flag is true, but accel sensor not supported";
                }
            }

            // Ensure depth and color frame rates match if both are enabled
            if (aSettings.bDepth && aSettings.bColor && depthProfile && colorProfile) {
                auto depthVsp = depthProfile->as<ob::VideoStreamProfile>();
                auto colorVsp = colorProfile->as<ob::VideoStreamProfile>();
                if (depthVsp && colorVsp && depthVsp->fps() != colorVsp->fps()) {
                    ofLogNotice("ofxOrbbecCamera::open") << "Syncing depth and color frame rates to " << colorVsp->fps() << " fps";
                    // Try to match depth to color frame rate
                    auto depthProfileList = mPipe->getStreamProfileList(OB_SENSOR_DEPTH);
                    try {
                        depthProfile = depthProfileList->getVideoStreamProfile(
                            depthVsp->width(), 
                            depthVsp->height(),
                            depthVsp->format(),
                            colorVsp->fps()
                        );
                        config->enableStream(depthProfile); // Override previous depth profile
                    } catch(ob::Error &e) {
                        ofLogError("ofxOrbbecCamera::open") << "Could not sync depth frame rate: " << e.getMessage();
                        return false;
                    }
                }
            }

            // Pass in the configuration and start the pipeline
            try {
                mPipe->start(config);
            } catch(ob::Error &e) {
                ofLogError("ofxOrbbecCamera::open") << "Pipeline start failed: " << e.getMessage();
                return false;
            }
            
            if (device->isPropertySupported(OB_PROP_DEPTH_ROTATE_INT, OB_PERMISSION_WRITE)) {
                device->setIntProperty(OB_PROP_DEPTH_ROTATE_INT, aSettings.rotation);
            }

            // --- XY unprojection table setup ---
            // Required for GPU mesh (bDepthMesh) and legacy CPU point cloud (bPointCloud/bPointCloudRGB).
            if( aSettings.bDepthMesh || aSettings.bPointCloud || aSettings.bPointCloudRGB ){
                auto param = mPipe->getCalibrationParam(config);
                // When D2C is enabled, depth frame is warped to color camera space,
                // so xyTable must use color sensor intrinsics + resolution.
                OBSensorType xySensor = (needD2C) ? OB_SENSOR_COLOR : OB_SENSOR_DEPTH;
                uint32_t xyWidth, xyHeight;
                if (needD2C && colorProfile) {
                    auto cvsp = colorProfile->as<ob::VideoStreamProfile>();
                    xyWidth  = cvsp->width();
                    xyHeight = cvsp->height();
                } else {
                    auto vsp = depthProfile->as<ob::VideoStreamProfile>();
                    xyWidth  = vsp->width();
                    xyHeight = vsp->height();
                }
                uint32_t tableSize = xyWidth * xyHeight * 2 * sizeof(float);
                xyTableData.resize(tableSize);
                if(!ob::CoordinateTransformHelper::transformationInitXYTables(param, xySensor, &xyTableData[0], &tableSize, &xyTables)) {
                    ofLogError("ofxOrbbecCamera::open") << "couldn't init xyTables for " << (needD2C ? "color (D2C)" : "depth");
                } else {
                    ofLogNotice("ofxOrbbecCamera::open") << "xyTables built for " << (needD2C ? "color (D2C)" : "depth")
                        << " sensor: " << xyWidth << "x" << xyHeight;
                }
            }

            // --- Legacy CPU PointCloudFilter (only when explicitly requested) ---
            if( aSettings.bPointCloud || aSettings.bPointCloudRGB ){
                auto cameraParam = mPipe->getCameraParam();
                pointCloud = std::make_shared<ob::PointCloudFilter>();
                pointCloud->setCameraParam(cameraParam);

                if( aSettings.bPointCloudRGB ){
                    pointCloud->setCreatePointFormat(OB_FORMAT_RGB_POINT);
                    // RGB point cloud needs color-sensor xyTables (overwrites depth tables set above)
                    auto param2 = mPipe->getCalibrationParam(config);
                    auto vsp2 = colorProfile->as<ob::VideoStreamProfile>();
                    uint32_t colorWidth  = vsp2->width();
                    uint32_t colorHeight = vsp2->height();
                    uint32_t tableSize2  = colorWidth * colorHeight * 2 * sizeof(float);
                    xyTableData.resize(tableSize2);
                    if(!ob::CoordinateTransformHelper::transformationInitXYTables(param2, OB_SENSOR_COLOR, &xyTableData[0], &tableSize2, &xyTables)) {
                        ofLogError("ofxOrbbecCamera::open") << "couldn't init xyTables for color";
                    }
                }else{
                    pointCloud->setCreatePointFormat(OB_FORMAT_POINT);
                    // Depth xyTables already set up above
                }
            }

            // Create depth post-processing filters
            if (aSettings.bDepth) {
                setupDepthFilters();
            }

            ob::Context::setLoggerSeverity(OB_LOG_SEVERITY_ERROR);
            bConnected = true;
            startThread();

        }else{
            return false; 
        }

    }

    return true; 
}

bool ofxOrbbecCamera::isConnected() const {
	if( mPipe && mPipe->getDevice() ){
        return bConnected; 
	}
	return false;
}

const ofPixels &ofxOrbbecCamera::getDepthPixels() const {
    mExtDepthFrameNo = mInternalDepthFrameNo;
    return mDepthPixels;
}

const ofFloatPixels &ofxOrbbecCamera::getDepthPixelsF() const {
    mExtDepthFrameNo = mInternalDepthFrameNo;
    return mDepthPixelsF;
} 

const ofPixels &ofxOrbbecCamera::getColorPixels() const {
    mExtColorFrameNo = mInternalColorFrameNo;
    return mColorPixels;
}

const ofPixels &ofxOrbbecCamera::getIRPixels() const {
    mExtIRFrameNo = mInternalIRFrameNo;
    return mIRPixels;
}

const ofShortPixels &ofxOrbbecCamera::getIRPixelsS() const {
    mExtIRFrameNo = mInternalIRFrameNo;
    return mIRPixelsS;
}

const std::vector <glm::vec3> &ofxOrbbecCamera::getPointCloud() const {
    mExtDepthFrameNo = mInternalDepthFrameNo;
    return mPointCloudPtsLocal;
} 

const ofMesh &ofxOrbbecCamera::getPointCloudMesh() const {
    mExtDepthFrameNo = mInternalDepthFrameNo;
    return mPointCloudMeshLocal;
}

void ofxOrbbecCamera::update(){
    if( mPipe ){
        bNewFrameDepth = bNewFrameColor = bNewFrameIR = false; 
        if( mInternalDepthFrameNo > mExtDepthFrameNo ){
            bNewFrameDepth = true; 
            bNewFrameIR = true; 
        }
        if( mInternalColorFrameNo > mExtColorFrameNo ){
            bNewFrameColor = true; 
        }
        if( bNewFrameColor || bNewFrameDepth || bNewFrameIR ){
            mTimeSinceFrame = 0; 
            bConnected = true; 
        }else{
            mTimeSinceFrame += ofClamp(ofGetLastFrameTime(), 1.0 / 250.0, 1.0 / 5.0);
        }
        if( bConnected && mTimeSinceFrame > 5.0 ){
            bConnected = false; 
        }
    }
    while(!gyroQueue.empty()) {
        gyroQueue.receive(gyro);
    }
    while(!accelQueue.empty()) {
        accelQueue.receive(accel);
    }
}

void ofxOrbbecCamera::threadedFunction(){
    while(isThreadRunning()){
        if( mPipe ){
            auto frameSet = mPipe->waitForFrames(20);
            if(frameSet) {
                if( mCurrentSettings.bDepth ) {
                    auto depthFrame = frameSet->getFrame(OB_FRAME_DEPTH);
                    if(depthFrame) {
                        // Apply post-processing filters to raw depth frame
                        depthFrame = applyDepthFilters(depthFrame);

                        auto tmpDepthPixels = processFrame(depthFrame);
                        ofFloatPixels tmpDepthPixelsF;
                        if (mCurrentSettings.bDepthFloat) {
                            tmpDepthPixelsF = processFrameFloatPixels(depthFrame);
                        }
                        // Lock while swapping pixel buffers — main thread reads these
                        lock();
                        mDepthPixels = std::move(tmpDepthPixels);
                        if (mCurrentSettings.bDepthFloat) {
                            mDepthPixelsF = std::move(tmpDepthPixelsF);
                        }
                        unlock();
                        if( mCurrentSettings.bPointCloud && !mCurrentSettings.bPointCloudRGB ){
                            try {
                                std::shared_ptr<ob::Frame> pointCloudFrame = pointCloud->process(frameSet);
                                pointCloudToMesh(frameSet->depthFrame());
                                mInternalDepthFrameNo++;  // Increment frame counter for point cloud
                            }
                            catch(std::exception &e) {
                                ofLogError("ofxOrbbecCamera::threadedFunction") << "Get point cloud failed";
                            };
                        }else{
                            mInternalDepthFrameNo++; 
                        }
                    }
                } // if( mCurrentSettings.bDepth )

                if( mCurrentSettings.bColor ) {
                    auto colorFrame = frameSet->getFrame(OB_FRAME_COLOR);
                    static int noFrameCounter = 0;
                    if(!colorFrame && noFrameCounter++ < 5) {
                        std::cerr << "##### NO COLOR FRAME RECEIVED! #####" << std::endl;
                        std::cerr.flush();
                    }
                    if(colorFrame) {
                        auto videoFrame = colorFrame->as<ob::VideoFrame>();
                        static int logCounter = 0;
                        if(logCounter++ < 5) {  // Log first 5 frames only
                            std::cerr << ">>>>> Color frame format: " << videoFrame->format() 
                                << " size: " << videoFrame->width() << "x" << videoFrame->height() 
                                << " dataSize: " << videoFrame->dataSize() << std::endl;
                            std::cerr.flush();
                        }
                        
                        auto tmpColorPixels = processFrame(colorFrame);
                        lock();
                        mColorPixels = std::move(tmpColorPixels);
                        unlock();

                        static int pixelLogCounter = 0;
                        if(pixelLogCounter++ < 5) {
                            std::cerr << ">>>>> mColorPixels allocated: " << mColorPixels.isAllocated() 
                                << " size: " << mColorPixels.getWidth() << "x" << mColorPixels.getHeight() << std::endl;
                            std::cerr.flush();
                        }

                        if( mCurrentSettings.bPointCloudRGB ){
                            // Try RGB point cloud if color frame was successfully processed
                            if(frameSet != nullptr && frameSet->depthFrame() != nullptr && frameSet->colorFrame() != nullptr && mColorPixels.isAllocated()) {
                                // point position value multiply depth value scale to convert uint to millimeter (for some devices, the default depth value uint is not
                                // millimeter)
                                auto depthValueScale = frameSet->depthFrame()->getValueScale();
                                pointCloud->setPositionDataScaled(depthValueScale);
                                try {
                                    std::shared_ptr<ob::Frame> pointCloudFrame = pointCloud->process(frameSet);
                                    pointCloudToMesh(frameSet->depthFrame(), frameSet->colorFrame());
                                }
                                catch(std::exception &e) {
                                    ofLogError("ofxOrbbecCamera::threadedFunction") << "Get RGB point cloud failed: " << e.what();
                                }
                            } else {
                                // Fall back to depth-only point cloud if color processing failed
                               static bool loggedFallback = false;
                                if(!loggedFallback) {
                                    ofLogWarning("ofxOrbbecCamera::threadedFunction") << "Color frame unavailable, falling back to depth-only point cloud";
                                    loggedFallback = true;
                                }
                                if(frameSet->depthFrame()) {
                                    try {
                                        pointCloudToMesh(frameSet->depthFrame());
                                    }
                                    catch(std::exception &e) {
                                        ofLogError("ofxOrbbecCamera::threadedFunction") << "Get depth point cloud failed: " << e.what();
                                    }
                                }
                            }
                        }else{
                            //In case h264 and we can't decode - pixels will be empty 
                            if(mColorPixels.getWidth()){
                                mInternalColorFrameNo++; 
                            }
                        }
                    }
                } // if(mCurrentSettings.bColor)

                if( mCurrentSettings.bIR ) {
                    auto irFrame = frameSet->irFrame();
                    if(irFrame) {
                        mIRPixels = processFrame(irFrame);
                        mIRPixelsS = processFrameShortPixels(irFrame);
                        mInternalIRFrameNo++;
                    } else {
                        ofLogVerbose("ofxOrbbecCamera::threadedFunction") << "ir frame is null";
                    }
                }
                
                if( mCurrentSettings.bIMU ) {
                    {
                        auto frame = frameSet->getFrame(OB_FRAME_GYRO);
                        if(frame) {
                            auto gyroFrame = frame->as<ob::GyroFrame>();
                            if(gyroFrame){
                                glm::vec3 gyro;
                                auto value = gyroFrame->value();
                                gyro.x = value.x;
                                gyro.y = value.y;
                                gyro.z = value.z;
                                gyroQueue.send(gyro);
                            }
                        } else {
                            ofLogVerbose("ofxOrbbecCamera::threadedFunction") << "gyro frame is null";
                        }
                    }
                    {
                        auto frame = frameSet->getFrame(OB_FRAME_ACCEL);
                        if(frame) {
                            auto accelFrame = frame->as<ob::AccelFrame>();
                            if(accelFrame){
                                glm::vec3 accel;
                                auto value = accelFrame->value();
                                accel.x = value.x;
                                accel.y = value.y;
                                accel.z = value.z;
                                accelQueue.send(accel);
                            }
                        } else {
                            ofLogVerbose("ofxOrbbecCamera::threadedFunction") << "acceleration frame is null";
                        }
                    }
                } // if( mCurrentSettings.bIMU )
            }
        }

    }
}
        
bool ofxOrbbecCamera::isFrameNew() const {
    return bNewFrameColor || bNewFrameDepth || bNewFrameIR;
}

bool ofxOrbbecCamera::isFrameNewDepth() const {
    return bNewFrameDepth;
}

bool ofxOrbbecCamera::isFrameNewColor() const {
    return bNewFrameColor;
}


#ifdef OFXORBBEC_DECODE_H264_H265

void ofxOrbbecCamera::initH26XCodecs(){
    if( !bInitOneTime ){
        // Initialize FFmpeg libraries
        av_register_all();
        avcodec_register_all();
        bInitOneTime = true; 

        // Allocate an AVCodecContext and set its codec
        codec264 = avcodec_find_decoder(AV_CODEC_ID_H264);
        codecContext264 = avcodec_alloc_context3(codec264);
        avcodec_open2(codecContext264, codec264, NULL);

        // // Allocate an AVCodecContext and set its codec
        codec265 = avcodec_find_decoder(AV_CODEC_ID_H265);
        codecContext265 = avcodec_alloc_context3(codec265);
        avcodec_open2(codecContext265, codec265, NULL);
    }
}

ofPixels ofxOrbbecCamera::decodeH26XFrame(uint8_t * myData, int dataSize, bool bH264){
    initH26XCodecs();
    
    AVPacket packet; 
    av_init_packet(&packet);
    packet.data = myData;
    packet.size = dataSize;

    // Allocate an AVFrame for decoded data
    AVFrame* frame = av_frame_alloc();
    
    ofPixels pix;

    auto codecContext = codecContext264;
    if( !bH264 ){
        codecContext = codecContext265; 
    }

    int ret = avcodec_send_packet(codecContext, &packet);
    if (ret < 0) {
        ofLogError("ofxOrbbecCamera::decodeH26XFrame") << "Error sending a packet for decoding";
        return pix;
    }
 
    int frameDecoded = avcodec_receive_frame(codecContext, frame);

    if( frameDecoded == 0 ){

        // Allocate an AVFrame for RGB data
        AVFrame* rgbFrame = av_frame_alloc();

        rgbFrame->format = AV_PIX_FMT_RGB24; 
        rgbFrame->width = codecContext->width; 
        rgbFrame->height = codecContext->height; 

        av_frame_get_buffer(rgbFrame, 0);

        // Create a sws context for RGB conversion
        swsContext = sws_getCachedContext(swsContext, codecContext->width, codecContext->height, codecContext->pix_fmt,
            codecContext->width, codecContext->height, (AVPixelFormat)rgbFrame->format, SWS_BILINEAR, NULL, NULL, NULL);
        
        // Convert the decoded frame to RGB
        int result = sws_scale(swsContext, frame->data, frame->linesize, 0, frame->height, rgbFrame->data, rgbFrame->linesize);
        pix.setFromPixels((unsigned char * )rgbFrame->data[0], codecContext->width, codecContext->height, 3); 

        av_frame_free(&rgbFrame);
    }

    // Clean up and free allocated memory
    av_frame_free(&frame);

    return pix; 
}

#endif 


ofPixels ofxOrbbecCamera::processFrame(std::shared_ptr<ob::Frame> frame){

    ofPixels pix; 
    cv::Mat imuMat;
    cv::Mat rstMat;

    try{
        
        if( !frame ){
            return pix; 
        }

        if(frame->type() == OB_FRAME_COLOR) {
            auto videoFrame = frame->as<ob::VideoFrame>();
            switch(videoFrame->format()) {
            case OB_FORMAT_H264:

                #ifdef OFXORBBEC_DECODE_H264_H265 
                    pix = decodeH26XFrame((uint8_t*)videoFrame->data(), videoFrame->dataSize(), true);
                #else
                    ofLogError("ofxOrbbecCamera::processFrame") << " h264 / h265 not enabled. Define OFXORBBEC_DECODE_H264_H265 or set color format to OB_FORMAT_RGB " << std::endl;
                #endif

            break; 
            case OB_FORMAT_H265:

                #ifdef OFXORBBEC_DECODE_H264_H265 
                    pix = decodeH26XFrame((uint8_t*)videoFrame->data(), videoFrame->dataSize(), false);
                #else
                    ofLogError("ofxOrbbecCamera::processFrame") << " h264 / h265 not enabled. Define OFXORBBEC_DECODE_H264_H265 or set color format to OB_FORMAT_RGB " << std::endl;
                #endif

            break; 
            case OB_FORMAT_MJPG: {

#if !defined(TARGET_OSX) && !defined(TARGET_WIN32)
                cv::Mat rawMat(1, videoFrame->dataSize(), CV_8UC1, videoFrame->data());
                rstMat = cv::imdecode(rawMat, 1);
                cv::cvtColor(rstMat, rstMat, cv::COLOR_BGR2RGB);
#else
                ofLogError("ofxOrbbecCamera::processFrame") << " MJPG not supported - set color format to OB_FORMAT_RGB " << std::endl;
#endif

            } break;
            case OB_FORMAT_NV21: {
                cv::Mat rawMat(videoFrame->height() * 3 / 2, videoFrame->width(), CV_8UC1, videoFrame->data());
                cv::cvtColor(rawMat, rstMat, cv::COLOR_YUV2RGB_NV21);
            } break;
            case OB_FORMAT_YUYV:
            case OB_FORMAT_YUY2: {
                cv::Mat rawMat(videoFrame->height(), videoFrame->width(), CV_8UC2, videoFrame->data());
                cv::cvtColor(rawMat, rstMat, cv::COLOR_YUV2RGB_YUYV);
            } break;
            case OB_FORMAT_RGB: {
                cv::Mat rawMat(videoFrame->height(), videoFrame->width(), CV_8UC3, videoFrame->data());
                rstMat = rawMat;
            } break;
            case OB_FORMAT_UYVY: {
                cv::Mat rawMat(videoFrame->height(), videoFrame->width(), CV_8UC2, videoFrame->data());
                cv::cvtColor(rawMat, rstMat, cv::COLOR_YUV2RGB_UYVY);
            } break;
            default:
                break;
            }
            if(!rstMat.empty()) {
                pix.setFromPixels(rstMat.ptr(), rstMat.cols, rstMat.rows, rstMat.channels());
            }
        }
        else if(frame->type() == OB_FRAME_DEPTH) {
            auto videoFrame = frame->as<ob::VideoFrame>();
            if(videoFrame->format() == OB_FORMAT_Y16) {
                cv::Mat cvtMat;
                cv::Mat rawMat = cv::Mat(videoFrame->height(), videoFrame->width(), CV_16UC1, videoFrame->data());
                // depth frame pixel value multiply scale to get distance in millimeter
                float scale = videoFrame->as<ob::DepthFrame>()->getValueScale();

                // threshold to 5.46m
                cv::threshold(rawMat, cvtMat, 5460.0f / scale, 0, cv::THRESH_TRUNC);
                cvtMat.convertTo(cvtMat, CV_8UC1, scale * 0.05);
                rstMat = cvtMat;//cv::applyColorMap(cvtMat, rstMat, cv::COLORMAP_JET);
            }
            if(!rstMat.empty()) {
                pix.setFromPixels(rstMat.ptr(), rstMat.cols, rstMat.rows, rstMat.channels());
            }
        }
        else if(frame->type() == OB_FRAME_IR || frame->type() == OB_FRAME_IR_LEFT || frame->type() == OB_FRAME_IR_RIGHT) {
            auto videoFrame = frame->as<ob::VideoFrame>();
            if(videoFrame->format() == OB_FORMAT_Y16) {
                cv::Mat cvtMat;
                cv::Mat rawMat = cv::Mat(videoFrame->height(), videoFrame->width(), CV_16UC1, videoFrame->data());
                rawMat.convertTo(cvtMat, CV_8UC1, 1.0 / 16.0f);
                rstMat = rawMat;//cv::cvtColor(cvtMat, rstMat, cv::COLOR_GRAY2RGB);
            }
            else if(videoFrame->format() == OB_FORMAT_Y8) {
                cv::Mat rawMat = cv::Mat(videoFrame->height(), videoFrame->width(), CV_8UC1, videoFrame->data());
                rstMat = rawMat;//cv::cvtColor(rawMat * 2, rstMat, cv::COLOR_GRAY2RGB);
            }
            else if(videoFrame->format() == OB_FORMAT_MJPG) {
#if !defined(TARGET_OSX) && !defined(TARGET_WIN32)
                cv::Mat rawMat(1, videoFrame->dataSize(), CV_8UC1, videoFrame->data());
                rstMat = cv::imdecode(rawMat, 1);
                rstMat = rawMat;//cv::cvtColor(rstMat * 2, rstMat, cv::COLOR_GRAY2RGB);
#else
                ofLogError("ofxOrbbecCamera::processFrame") << " MJPG not supported - set IR format to OB_FORMAT_Y16 or OB_FORMAT_Y8 " << std::endl;
#endif
            }
            if(!rstMat.empty()) {
                pix.setFromPixels(rstMat.ptr(), rstMat.cols, rstMat.rows, rstMat.channels());
            }
        }
    } catch(const cv::Exception& ex) {
        ofLogError("processFrame") << "OpenCV exception: " << ex.what() << std::endl; 
    } catch(const std::exception& ex) {
        ofLogError("processFrame") << "Exception: " << ex.what() << std::endl; 
    }
    return pix; 
}

ofFloatPixels ofxOrbbecCamera::processFrameFloatPixels(std::shared_ptr<ob::Frame> frame) {
    ofFloatPixels pix;
    cv::Mat imuMat;
    cv::Mat rstMat;

    try{
        if( !frame ){
            return pix;
        }

        if(frame->type() == OB_FRAME_DEPTH) {
            auto videoFrame = frame->as<ob::VideoFrame>();
            if(videoFrame->format() == OB_FORMAT_Y16) {
                std::vector<float> raw_pixels;
                raw_pixels.resize(videoFrame->width() * videoFrame->height());
                float scale = videoFrame->as<ob::DepthFrame>()->getValueScale();
                
                // Copy Y16 data (unsigned short) to float buffer, then apply scale
                const uint16_t* depthData = static_cast<const uint16_t*>(videoFrame->data());
                for(size_t i = 0; i < raw_pixels.size(); i++) {
                    raw_pixels[i] = static_cast<float>(depthData[i]) * scale;
                }
                
                pix.setFromPixels(raw_pixels.data(), videoFrame->width(), videoFrame->height(), 1);
            }
        }
    } catch(const cv::Exception& ex) {
        ofLogError("processFrameFloatPixels") << "OpenCV exception: " << ex.what() << std::endl;
    } catch(const std::exception& ex) {
        ofLogError("processFrameFloatPixels") << "Exception: " << ex.what() << std::endl;
    }
    return pix;
}

ofShortPixels ofxOrbbecCamera::processFrameShortPixels(std::shared_ptr<ob::Frame> frame) {
    ofShortPixels pix;
    cv::Mat imuMat;
    cv::Mat rstMat;

    try{
        if( !frame ){
            return pix;
        }

        if(frame->type() == OB_FRAME_DEPTH
           || frame->type() == OB_FRAME_IR
           || frame->type() == OB_FRAME_IR_LEFT
           || frame->type() == OB_FRAME_IR_RIGHT)
        {
            auto videoFrame = frame->as<ob::VideoFrame>();
            if(videoFrame->format() == OB_FORMAT_Y16) {
                pix.setFromPixels((unsigned short *)videoFrame->data(), videoFrame->width(), videoFrame->height(), 1);
            }
        }
    } catch(const cv::Exception& ex) {
        ofLogError("processFrameShortPixels") << "OpenCV exception: " << ex.what() << std::endl;
    } catch(const std::exception& ex) {
        ofLogError("processFrameShortPixels") << "Exception: " << ex.what() << std::endl;
    }
    return pix;
}


void ofxOrbbecCamera::pointCloudToMesh(std::shared_ptr<ob::DepthFrame> depthFrame, std::shared_ptr<ob::ColorFrame> colorFrame){
    if( depthFrame ){
    
		bool bRGB = false;
		if(colorFrame){
			bRGB = true;
		}

        int numPoints = 0;
		uint32_t pointcloudSize = 0;
		
		 if(bRGB){
			numPoints = colorFrame->width() * colorFrame->height();
            pointcloudSize = numPoints * sizeof(OBColorPoint);
        }else{
			numPoints = depthFrame->width() * depthFrame->height();
            pointcloudSize = numPoints * sizeof(OBPoint);
        }
		
        std::vector <uint8_t> pointcloudData;
		if( mPointcloudData.size() != pointcloudSize){
			mPointcloudData.resize(pointcloudSize);
		}

        mPointCloudMesh = ofMesh();
        mPointCloudPts.clear();
        mPointCloudPts.reserve(numPoints);
        mPointCloudMesh.setMode(OF_PRIMITIVE_POINTS);

        if( bRGB ){
			// Validate color frame data before processing
			if (!colorFrame || !colorFrame->data() || colorFrame->dataSize() == 0) {
				ofLogError("pointCloudToMesh") << "Invalid color frame data";
				return;
			}
			
			// Validate depth frame data
			if (!depthFrame->data() || depthFrame->dataSize() == 0) {
				ofLogError("pointCloudToMesh") << "Invalid depth frame data";
				return;
			}
			
			OBColorPoint *point = (OBColorPoint *)&mPointcloudData[0];
			
			try {
				ob::CoordinateTransformHelper::transformationDepthToRGBDPointCloud(&xyTables, depthFrame->data(), colorFrame->data(), point);
			} catch (const std::exception& e) {
				ofLogError("pointCloudToMesh") << "Error in transformationDepthToRGBDPointCloud: " << e.what();
				return;
			}

			point = (OBColorPoint *)&mPointcloudData[0];

            std::vector <ofFloatColor> tColors;
            tColors.reserve(numPoints);

            for(int i = 0; i < numPoints; i++) {
                auto pt = glm::vec3(point->x, -point->y, -point->z);

                mPointCloudPts.push_back(pt);
                tColors.push_back(ofColor((int)point->r, (int)point->g, (int)point->b, 255));

                point++;
            }
            mPointCloudMesh.addColors(tColors);

        }else{
			OBPoint *point = (OBPoint *)&mPointcloudData[0];
			ob::CoordinateTransformHelper::transformationDepthToPointCloud(&xyTables, depthFrame->data(), point);

			point = (OBPoint *)&mPointcloudData[0];

            for(int i = 0; i < numPoints; i++) {
                auto pt = glm::vec3(point->x, -point->y, -point->z);

                mPointCloudPts.push_back(pt);
                point++;
            }
        }

        mPointCloudMesh.addVertices(mPointCloudPts);
        // No setupIndicesAuto() - OF_PRIMITIVE_POINTS doesn't need an index buffer

        if( lock() ){
            std::swap(mPointCloudMeshLocal, mPointCloudMesh);  // swap avoids a full mesh copy
            std::swap(mPointCloudPtsLocal, mPointCloudPts);
            if( bRGB ){
                mInternalColorFrameNo++;
            }else{
                mInternalDepthFrameNo++;
            }
            unlock();
        }
    }
}

void ofxOrbbecCamera::setOrbbecLogLevel(OBLogSeverity level) {
    ob::Context::setLoggerSeverity(level);
}

// --- Depth post-processing filters ---

void ofxOrbbecCamera::setupDepthFilters() {
    try {
        mTemporalFilter = std::make_shared<ob::TemporalFilter>();
        mTemporalFilter->enable(mCurrentSettings.bTemporalFilter);
        ofLogNotice("ofxOrbbecCamera") << "TemporalFilter created (enabled=" << mCurrentSettings.bTemporalFilter << ")";
    } catch (const std::exception& e) {
        ofLogWarning("ofxOrbbecCamera") << "TemporalFilter not available: " << e.what();
    }

    try {
        mSpatialFilter = std::make_shared<ob::SpatialAdvancedFilter>();
        mSpatialFilter->enable(mCurrentSettings.bSpatialFilter);
        ofLogNotice("ofxOrbbecCamera") << "SpatialAdvancedFilter created (enabled=" << mCurrentSettings.bSpatialFilter << ")";
    } catch (const std::exception& e) {
        ofLogWarning("ofxOrbbecCamera") << "SpatialAdvancedFilter not available: " << e.what();
    }

    try {
        mHoleFillingFilter = std::make_shared<ob::HoleFillingFilter>();
        mHoleFillingFilter->enable(mCurrentSettings.bHoleFillingFilter);
        ofLogNotice("ofxOrbbecCamera") << "HoleFillingFilter created (enabled=" << mCurrentSettings.bHoleFillingFilter << ")";
    } catch (const std::exception& e) {
        ofLogWarning("ofxOrbbecCamera") << "HoleFillingFilter not available: " << e.what();
    }

    try {
        mNoiseRemovalFilter = std::make_shared<ob::NoiseRemovalFilter>();
        mNoiseRemovalFilter->enable(mCurrentSettings.bNoiseRemovalFilter);
        ofLogNotice("ofxOrbbecCamera") << "NoiseRemovalFilter created (enabled=" << mCurrentSettings.bNoiseRemovalFilter << ")";
    } catch (const std::exception& e) {
        ofLogWarning("ofxOrbbecCamera") << "NoiseRemovalFilter not available: " << e.what();
    }

}

std::shared_ptr<ob::Frame> ofxOrbbecCamera::applyDepthFilters(std::shared_ptr<ob::Frame> frame) {
    // Apply filters in recommended order: spatial → temporal → noise removal → hole filling
    if (mSpatialFilter && mSpatialFilter->isEnabled()) {
        try { frame = mSpatialFilter->process(frame); } catch (...) {}
    }
    if (mTemporalFilter && mTemporalFilter->isEnabled()) {
        try { frame = mTemporalFilter->process(frame); } catch (...) {}
    }
    if (mNoiseRemovalFilter && mNoiseRemovalFilter->isEnabled()) {
        try { frame = mNoiseRemovalFilter->process(frame); } catch (...) {}
    }
    if (mHoleFillingFilter && mHoleFillingFilter->isEnabled()) {
        try { frame = mHoleFillingFilter->process(frame); } catch (...) {}
    }
    return frame;
}

void ofxOrbbecCamera::enableTemporalFilter(bool enable) {
    if (mTemporalFilter) mTemporalFilter->enable(enable);
}
void ofxOrbbecCamera::enableSpatialFilter(bool enable) {
    if (mSpatialFilter) mSpatialFilter->enable(enable);
}
void ofxOrbbecCamera::enableHoleFillingFilter(bool enable) {
    if (mHoleFillingFilter) mHoleFillingFilter->enable(enable);
}
void ofxOrbbecCamera::enableNoiseRemovalFilter(bool enable) {
    if (mNoiseRemovalFilter) mNoiseRemovalFilter->enable(enable);
}
void ofxOrbbecCamera::setTemporalFilterWeight(float weight) {
    if (mTemporalFilter) mTemporalFilter->setWeight(weight);
}
void ofxOrbbecCamera::setTemporalFilterDiffScale(float diffScale) {
    if (mTemporalFilter) mTemporalFilter->setDiffScale(diffScale);
}
void ofxOrbbecCamera::setSpatialFilterParams(OBSpatialAdvancedFilterParams params) {
    if (mSpatialFilter) mSpatialFilter->setFilterParams(params);
}
void ofxOrbbecCamera::setHoleFillingMode(OBHoleFillingMode mode) {
    if (mHoleFillingFilter) mHoleFillingFilter->setFilterMode(mode);
}

bool ofxOrbbecCamera::isTemporalFilterEnabled() const {
    return mTemporalFilter && mTemporalFilter->isEnabled();
}
bool ofxOrbbecCamera::isSpatialFilterEnabled() const {
    return mSpatialFilter && mSpatialFilter->isEnabled();
}
bool ofxOrbbecCamera::isHoleFillingFilterEnabled() const {
    return mHoleFillingFilter && mHoleFillingFilter->isEnabled();
}
bool ofxOrbbecCamera::isNoiseRemovalFilterEnabled() const {
    return mNoiseRemovalFilter && mNoiseRemovalFilter->isEnabled();
}

