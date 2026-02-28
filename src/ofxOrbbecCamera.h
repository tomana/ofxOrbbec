#include "ofMain.h"

#include "libobsensor/ObSensor.hpp"
#include "libobsensor/hpp/Error.hpp"
#include "libobsensor/hpp/Filter.hpp"
#include <opencv2/opencv.hpp>


//If you have ffmpeg / libavcodec included in your project uncomment below 
//You can easily get the required libs from ofxFFmpegRTSP addon ( if you add it to your project )
//#define OFXORBBEC_DECODE_H264_H265

// this allows us to decode the color video streams from Femto Mega over IP connection 
#ifdef OFXORBBEC_DECODE_H264_H265
    extern "C" {
    #include <libavcodec/avcodec.h>
    #include <libavformat/avformat.h>
    #include <libswscale/swscale.h>
    #include <libavutil/imgutils.h>
    }
#endif 

namespace ofxOrbbec{

struct Settings{

    struct FrameType{
        int requestWidth = 0; //needed
        int requestHeight = 0;//usually not needed, width is enough 
        OBFormat format = OB_FORMAT_UNKNOWN; //DEPTH: OB_FORMAT_Y16 COLOR: OB_FORMAT_RGB or OB_FORMAT_MJPG 
        int frameRate = 30;
    };

    std::string ip = "";
    int deviceID = 0;
    std::string deviceSerial = "";

    OBRotateDegreeType rotation = OB_ROTATE_DEGREE_0;

    FrameType depthFrameSize;
    FrameType colorFrameSize;
    FrameType irFrameSize;

    bool bColor = false;
    bool bDepth = false;
    bool bIR = false;
    bool bPointCloud = false;
    bool bPointCloudRGB = false;
    bool bDepthMesh = false;  // Build xyTables for GPU mesh without PointCloudFilter overhead
    bool bAlignD2C = false;   // Enable depth-to-color alignment (HW then SW fallback) without CPU point cloud

    bool bIMU = false;
    bool bDepthFloat = false;  // Enable getDepthPixelsF(); off by default (saves ~1.4 MB alloc/frame)

    // Depth post-processing filters (applied in order before pixel conversion)
    bool bTemporalFilter = false;          // Smooth depth across frames
    bool bSpatialFilter = false;           // Edge-preserving spatial smoothing
    bool bHoleFillingFilter = false;       // Fill missing depth pixels
    bool bNoiseRemovalFilter = false;      // Remove small noise scatter
    bool bEdgeNoiseRemovalFilter = false;  // Remove noise at depth edges
};

};


class ofxOrbbecCamera : public ofThread{
    public:

        ofxOrbbecCamera() = default; 
        ofxOrbbecCamera(const ofxOrbbecCamera &) = delete;
        ~ofxOrbbecCamera();

        bool open(ofxOrbbec::Settings aSettings);
        bool isConnected() const;
        void close();
        void update();

        static std::vector < std::shared_ptr<ob::DeviceInfo> > getDeviceList(bool bIncludeNetworkDevices); 

        //any frame
        bool isFrameNew() const;
        bool isFrameNewDepth() const;
        bool isFrameNewColor() const;
        bool isFrameNewIR() const;

        const ofPixels &getDepthPixels() const;
        const ofFloatPixels &getDepthPixelsF() const;
        
        const ofPixels &getColorPixels() const;
        
        const ofPixels &getIRPixels() const;
        const ofShortPixels &getIRPixelsS() const;

        const std::vector <glm::vec3> &getPointCloud() const;
        const ofMesh &getPointCloudMesh() const;

        // XY unprojection tables — available after open() when bDepthMesh or bPointCloud is set.
        // xTable[row*width+col] and yTable[row*width+col] give the per-pixel ray directions so that:
        //   X = xTable[i] * depth_mm,  Y = yTable[i] * depth_mm,  Z = depth_mm
        bool           isXYTableReady()    const { return xyTables.xTable != nullptr && !xyTableData.empty(); }
        const float*   getXYTableX()       const { return xyTables.xTable; }
        const float*   getXYTableY()       const { return xyTables.yTable; }
        int            getXYTableWidth()   const { return xyTables.width; }
        int            getXYTableHeight()  const { return xyTables.height; }
        
        glm::vec3 getGyro() const {
            return gyro;
        }
        glm::vec3 getAcceleration() const {
            return accel;
        }
        
        static void setOrbbecLogLevel(OBLogSeverity level);

        // --- Depth post-processing filter control ---
        // These can be called at any time to enable/disable/configure filters.
        // Filters are applied to raw depth frames before pixel conversion.
        void enableTemporalFilter(bool enable);
        void enableSpatialFilter(bool enable);
        void enableHoleFillingFilter(bool enable);
        void enableNoiseRemovalFilter(bool enable);
        void enableEdgeNoiseRemovalFilter(bool enable);

        // Fine-tuning (call after open())
        void setTemporalFilterWeight(float weight);       // 0-1: blend factor (higher = more smoothing)
        void setTemporalFilterDiffScale(float diffScale);  // sensitivity to depth changes
        void setSpatialFilterParams(OBSpatialAdvancedFilterParams params);
        void setHoleFillingMode(OBHoleFillingMode mode);

        // Query filter state
        bool isTemporalFilterEnabled() const;
        bool isSpatialFilterEnabled() const;
        bool isHoleFillingFilterEnabled() const;
        bool isNoiseRemovalFilterEnabled() const;
        bool isEdgeNoiseRemovalFilterEnabled() const;

    protected:
        void threadedFunction() override; 
        void clear(); 
        
        ofPixels processFrame(std::shared_ptr<ob::Frame> frame);
        ofFloatPixels processFrameFloatPixels(std::shared_ptr<ob::Frame> frame);
        ofShortPixels processFrameShortPixels(std::shared_ptr<ob::Frame> frame);
		void pointCloudToMesh(std::shared_ptr<ob::DepthFrame> depthFrame, std::shared_ptr<ob::ColorFrame> colorFrame = std::shared_ptr<ob::ColorFrame>() );

        ofxOrbbec::Settings mCurrentSettings;
        
        bool bNewFrameColor = false;
        bool bNewFrameDepth = false;
        bool bNewFrameIR = false;
        
        size_t mInternalDepthFrameNo = 0;
        mutable size_t mExtDepthFrameNo = 0;
    
        size_t mInternalColorFrameNo = 0;
        mutable size_t mExtColorFrameNo = 0;

        size_t mInternalIRFrameNo = 0;
        mutable size_t mExtIRFrameNo = 0;

        ofPixels mDepthPixels;
        ofFloatPixels mDepthPixelsF;
        
        ofPixels mColorPixels;
        
        ofPixels mIRPixels;
        ofShortPixels mIRPixelsS;

        ofMesh mPointCloudMesh;
        ofMesh mPointCloudMeshLocal;
        std::vector <glm::vec3> mPointCloudPts;
        std::vector <glm::vec3> mPointCloudPtsLocal;

		std::shared_ptr <ob::Pipeline> mPipe;
   		std::shared_ptr <ob::PointCloudFilter> pointCloud;
   		std::shared_ptr <ob::Context> ctxLocal;
    
        #ifdef OFXORBBEC_DECODE_H264_H265

            bool bInitOneTime = false; 

            AVCodec* codec264 = nullptr;
            AVCodecContext* codecContext264 = nullptr;

            AVCodec* codec265 = nullptr;
            AVCodecContext* codecContext265 = nullptr;

            SwsContext* swsContext = nullptr;

            void initH26XCodecs();
            ofPixels decodeH26XFrame(uint8_t * myData, int dataSize, bool bH264);

        #endif
        
        OBXYTables xyTables;
        std::vector <float> xyTableData;
        std::vector <uint8_t> mPointcloudData;
        bool bConnected = false;
        float mTimeSinceFrame = 0;

        // Depth post-processing filters
        std::shared_ptr<ob::TemporalFilter>       mTemporalFilter;
        std::shared_ptr<ob::SpatialAdvancedFilter> mSpatialFilter;
        std::shared_ptr<ob::HoleFillingFilter>     mHoleFillingFilter;
        std::shared_ptr<ob::NoiseRemovalFilter>    mNoiseRemovalFilter;
        std::shared_ptr<ob::EdgeNoiseRemovalFilter> mEdgeNoiseRemovalFilter;
        void setupDepthFilters();
        std::shared_ptr<ob::Frame> applyDepthFilters(std::shared_ptr<ob::Frame> frame);
        glm::vec3 gyro;
        glm::vec3 accel;
        ofThreadChannel<glm::vec3> gyroQueue;
        ofThreadChannel<glm::vec3> accelQueue;
};
