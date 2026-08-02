// Include file 
#ifndef COMMON_HPP  // Header guard to prevent multiple inclusions
#define COMMON_HPP

// C++ includes
#include <iostream> // The iostream library is an object-oriented library that provides input and output functionality using streams
#include <algorithm> // The header <algorithm> defines a collection of functions especially designed to be used on ranges of elements.
#include <fstream> // Input/output stream class to operate on files.
#include <chrono> // c++ timekeeper library
#include <vector> // vectors are sequence containers representing arrays that can change in size.
#include <queue>
#include <thread> // class to represent individual threads of execution.
#include <mutex> // A mutex is a lockable object that is designed to signal when critical sections of code need exclusive access, preventing other threads with the same protection from executing concurrently and access the same memory locations.
#include <cstdlib> // to find home directory
#include <set> // Addded : (Needed to keep track of distinict maps seen in each tick)

#include <cstring>
#include <sstream> // String stream processing functionalities
#include <iomanip> //for forced 9 decimal precision

//* ROS2 includes
//* std_msgs in ROS 2 https://docs.ros2.org/foxy/api/std_msgs/index-msg.html
#include "rclcpp/rclcpp.hpp"

// #include "your_custom_msg_interface/msg/custom_msg_field.hpp" // Example of adding in a custom message
#include <std_msgs/msg/header.hpp>
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include "sensor_msgs/msg/image.hpp"

//Added: (Hamdan)
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <nav_msgs/msg/path.hpp>

using std::placeholders::_1; //* TODO why this is suggested in official tutorial

// Include Eigen
// Quick reference: https://eigen.tuxfamily.org/dox/group__QuickRefPage.html
#include <Eigen/Dense> // Includes Core, Geometry, LU, Cholesky, SVD, QR, and Eigenvalues header file

// Include cv-bridge
#include <cv_bridge/cv_bridge.h>

// Include OpenCV computer vision library
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp> // Image processing tools
#include <opencv2/highgui/highgui.hpp> // GUI tools
#include <opencv2/core/eigen.hpp>
#include <image_transport/image_transport.h>

//* ORB SLAM 3 includes
#include "System.h" //* Also imports the ORB_SLAM3 namespace

//* Gobal defs
#define pass (void)0 // Python's equivalent of "pass" i.e. no operation


//* Node specific definitions
class MonocularMode : public rclcpp::Node
{   
    //* This slam node inherits from both rclcpp and ORB_SLAM3::System classes
    //* public keyword needs to come before the class constructor and anything else
    public:
    std::string experimentConfig = ""; // String to receive settings sent by the python driver
    double timeStep; // Timestep data received from the python node
    std::string receivedConfig = "";

    //* Class constructor
    MonocularMode(); // Constructor 

    ~MonocularMode(); // Destructor
        
    private:
        
        // Class internal variables
        std::string homeDir = "";
        std::string packagePath = "autonomous_drone/ros2_test/src/ros2_orb_slam3/"; //! Change to match path to your workspace
        std::string OPENCV_WINDOW = ""; // Set during initialization
        std::string nodeName = ""; // Name of this node
        std::string vocFilePath = ""; // Path to ORB vocabulary provided by DBoW2 package
        std::string settingsFilePath = ""; // Path to settings file provided by ORB_SLAM3 package
        bool bSettingsFromPython = false; // Flag set once when experiment setting from python node is received
        bool bVSLAMInitialized = false;
        
        std::string subexperimentconfigName = ""; // Subscription topic name
        std::string pubconfigackName = ""; // Publisher topic name
        std::string subImgMsgName = ""; // Topic to subscribe to receive RGB images from a python node
        std::string subTimestepMsgName = ""; // Topic to subscribe to receive the timestep related to the 
        std::string pubKeyframeTimestampsName = ""; //Added
        std::string pubMapTopologyChangedName = ""; //Added
        //Added: (Hamdan)
        std::string pubCurrentPoseRawName = "";
        std::string pubCurrentPointsRawName = "";
        std::string pubKeyframePointsName = "";   // REPLACES the live CSV
        std::string pubTrajectoryName = "";       // NEW - fixes the multi-map trajectory bug

        //* Definitions of publisher and subscribers
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr expConfig_subscription_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr configAck_publisher_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subImgMsg_subscription_;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subTimestepMsg_subscription_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr keyframeTimestamps_publisher_; // Added: (Hamdan), To publish time stamps
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr mapTopologyChanged_publisher_; //Added: Fires whenever the set of active maps_ids changes (new map, or merge/loop closure) //Hamdan
        //Added: (Hamdan)
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr currentPoseRaw_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr currentPointsRaw_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr keyframePoints_publisher_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_publisher_;

        //* ORB_SLAM3 related variables
        ORB_SLAM3::System* pAgent; // pointer to a ORB SLAM3 object
        ORB_SLAM3::System::eSensor sensorType;
        bool enablePangolinWindow = false; // Shows Pangolin window output
        bool enableOpenCVWindow = false; // Shows OpenCV window output
        
        //* Live-CSV related, ADDED (Hamdan)
        rclcpp::TimerBase::SharedPtr liveCsvTimer_;   // fires every ~1s, rewrites live_sparse_map_points.csv from scratch

        std::set<long unsigned int> prevMapIds_; //Added: maps_ids seen on the prev LIveCsvTimer_callback tick (Hamdan)

        //* ROS callbacks
        void experimentSetting_callback(const std_msgs::msg::String& msg); // Callback to process settings sent over by Python node
        void Timestep_callback(const std_msgs::msg::Float64& time_msg); // Callback to process the timestep for this image
        void Img_callback(const sensor_msgs::msg::Image& msg); // Callback to process RGB image and semantic matrix sent by Python node
        void LiveCsvTimer_callback(); // ADDED: periodic full-rewrite of live CSV(Changed) + publish keyframe timestamps (Hamdan)
        
        //* Helper functions
        // ORB_SLAM3::eigenMatXf convertToEigenMat(const std_msgs::msg::Float32MultiArray& msg); // Helper method, converts semantic matrix eigenMatXf, a Eigen 4x4 float matrix
        void initializeVSLAM(std::string& configString); //* Method to bind an initialized VSLAM framework to this node

        //We need to compute depth manually as KeyFrame member mvDepth is negative for moncular only slam i.e grabage we need
        //to manually caalculate it (Hamdan)
        //Function Defination
        
        // Added: (Hamdan)
        void WriteKeyframeDataToFile(const std::string& filepath, std::vector<double>& outTimestamps,std::set<long unsigned int>& outMapIds);
        
        void SaveSparseMapPointsPerKeyframe(const std::string& filepath);

        //Added: (Hamdan)
        void PublishCurrentPoseAndPoints(const Sophus::SE3f& Tcw); // per-frame, called from Img_callback
        void PublishLiveMapData();  // periodic, called from LiveCsvTimer_callback - REPLACES the CSV write


};

#endif
