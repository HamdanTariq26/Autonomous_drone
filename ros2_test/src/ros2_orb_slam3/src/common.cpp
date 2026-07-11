/*

A bare-bones example node demonstrating the use of the Monocular mode in ORB-SLAM3

Author: Azmyin Md. Kamal
Date: 01/01/24

REQUIREMENTS
* Make sure to set path to your workspace in common.hpp file

*/

//* Includes
#include "ros2_orb_slam3/common.hpp"

//* Constructor
MonocularMode::MonocularMode() :Node("mono_node_cpp")
{
    // Declare parameters to be passsed from command line
    // https://roboticsbackend.com/rclcpp-params-tutorial-get-set-ros2-params-with-cpp/
    
    //* Find path to home directory
    homeDir = getenv("HOME");
    // std::cout<<"Home: "<<homeDir<<std::endl;
    
    // std::cout<<"VLSAM NODE STARTED\n\n";
    RCLCPP_INFO(this->get_logger(), "\nORB-SLAM3-V1 NODE STARTED");

    this->declare_parameter("node_name_arg", "not_given"); // Name of this agent 
    this->declare_parameter("voc_file_arg", "file_not_set"); // Needs to be overriden with appropriate name  
    this->declare_parameter("settings_file_path_arg", "file_path_not_set"); // path to settings file  
    
    //* Watchdog, populate default values
    nodeName = "not_set";
    vocFilePath = "file_not_set";
    settingsFilePath = "file_not_set";

    //* Populate parameter values
    rclcpp::Parameter param1 = this->get_parameter("node_name_arg");
    nodeName = param1.as_string();
    
    rclcpp::Parameter param2 = this->get_parameter("voc_file_arg");
    vocFilePath = param2.as_string();

    rclcpp::Parameter param3 = this->get_parameter("settings_file_path_arg");
    settingsFilePath = param3.as_string();

    // rclcpp::Parameter param4 = this->get_parameter("settings_file_name_arg");
    
  
    //* HARDCODED, set paths
    if (vocFilePath == "file_not_set" || settingsFilePath == "file_not_set")
    {
        pass;
        vocFilePath = homeDir + "/" + packagePath + "orb_slam3/Vocabulary/ORBvoc.txt.bin";
        settingsFilePath = homeDir + "/" + packagePath + "orb_slam3/config/Monocular/";
    }

    // std::cout<<"vocFilePath: "<<vocFilePath<<std::endl;
    // std::cout<<"settingsFilePath: "<<settingsFilePath<<std::endl;
    
    
    //* DEBUG print
    RCLCPP_INFO(this->get_logger(), "nodeName %s", nodeName.c_str());
    RCLCPP_INFO(this->get_logger(), "voc_file %s", vocFilePath.c_str());
    // RCLCPP_INFO(this->get_logger(), "settings_file_path %s", settingsFilePath.c_str());
    
    subexperimentconfigName = "/mono_py_driver/experiment_settings"; // topic that sends out some configuration parameters to the cpp ndoe
    pubconfigackName = "/mono_py_driver/exp_settings_ack"; // send an acknowledgement to the python node
    subImgMsgName = "/mono_py_driver/img_msg"; // topic to receive RGB image messages
    subTimestepMsgName = "/mono_py_driver/timestep_msg"; // topic to receive RGB image messages
    pubKeyframeTimestampsName = "/mono_py_driver/keyframe_timestamps"; //ADDED (Hamdan)
    pubMapTopologyChangedName = "/mono_py_driver/map_topology_changed"; //Added (Hamdan)

    //* subscribe to python node to receive settings
    expConfig_subscription_ = this->create_subscription<std_msgs::msg::String>(subexperimentconfigName, 1, std::bind(&MonocularMode::experimentSetting_callback, this, _1));

    //* publisher to send out acknowledgement
    configAck_publisher_ = this->create_publisher<std_msgs::msg::String>(pubconfigackName, 10);

    //* subscrbite to the image messages coming from the Python driver node
    subImgMsg_subscription_= this->create_subscription<sensor_msgs::msg::Image>(subImgMsgName, 1, std::bind(&MonocularMode::Img_callback, this, _1));

    //* subscribe to receive the timestep
    subTimestepMsg_subscription_= this->create_subscription<std_msgs::msg::Float64>(subTimestepMsgName, 1, std::bind(&MonocularMode::Timestep_callback, this, _1));
    
    //ADDED: Publisher for the live keyframe timestamp list (Python side will subscribe for grace-period frame deletion) (Hamdan)
    keyframeTimestamps_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(pubKeyframeTimestampsName,10);

    //Added: Publisher when map changes (Hamdan)
    mapTopologyChanged_publisher_ = this->create_publisher<std_msgs::msg::Bool>(pubMapTopologyChangedName, 10);


    
    RCLCPP_INFO(this->get_logger(), "Waiting to finish handshake ......");
    
}

//* Destructor
MonocularMode::~MonocularMode()
{   
    
    // Stop all threads
    if (pAgent != nullptr) {
        pAgent->SaveKeyFrameTrajectoryTUM(homeDir + "/KeyFrameTrajectory.txt");
        SaveSparseMapPointsPerKeyframe(homeDir + "/SparseMapPoints.csv");
	    pAgent->Shutdown();
    }
    // Release resources and cleanly shutdown
    pass;

}

//* Callback which accepts experiment parameters from the Python node
void MonocularMode::experimentSetting_callback(const std_msgs::msg::String& msg){
    
    // std::cout<<"experimentSetting_callback"<<std::endl;
    bSettingsFromPython = true;
    experimentConfig = msg.data.c_str();
    // receivedConfig = experimentConfig; // Redundant
    
    RCLCPP_INFO(this->get_logger(), "Configuration YAML file name: %s", this->receivedConfig.c_str());

    //* Publish acknowledgement
    auto message = std_msgs::msg::String();
    message.data = "ACK";
    
    std::cout<<"Sent response: "<<message.data.c_str()<<std::endl;
    configAck_publisher_->publish(message);

    //* Wait to complete VSLAM initialization
    if (!bVSLAMInitialized) {
        bVSLAMInitialized = true;
        bSettingsFromPython = true;
        initializeVSLAM(experimentConfig);
    }

}

//* Method to bind an initialized VSLAM framework to this node
void MonocularMode::initializeVSLAM(std::string& configString){
    
    // Watchdog, if the paths to vocabular and settings files are still not set
    if (vocFilePath == "file_not_set" || settingsFilePath == "file_not_set")
    {
        RCLCPP_ERROR(get_logger(), "Please provide valid voc_file and settings_file paths");       
        rclcpp::shutdown();
    } 
    
    //* Build .yaml`s file path
    
    settingsFilePath = settingsFilePath.append(configString);
    settingsFilePath = settingsFilePath.append(".yaml"); // Example ros2_ws/src/orb_slam3_ros2/orb_slam3/config/Monocular/TUM2.yaml

    RCLCPP_INFO(this->get_logger(), "Path to settings file: %s", settingsFilePath.c_str());
    
    // NOTE if you plan on passing other configuration parameters to ORB SLAM3 Systems class, do it here
    // NOTE you may also use a .yaml file here to set these values
    sensorType = ORB_SLAM3::System::MONOCULAR; 
    enablePangolinWindow = true; // Shows Pangolin window output
    enableOpenCVWindow = true; // Shows OpenCV window output
    
    pAgent = new ORB_SLAM3::System(vocFilePath, settingsFilePath, sensorType, enablePangolinWindow);
    std::cout << "MonocularMode node initialized" << std::endl; // TODO needs a better message
    
    //ADDED: live csv path + start the periodic rewrite timer, now that pAgent exists.
//Created here (not in the constructor) so it can never fire before pAgent is valid.

liveFilePath = homeDir + "/live_sparse_map_points.csv";
liveCsvTimer_ = this->create_wall_timer(
	std::chrono::seconds(1),
	std::bind(&MonocularMode::LiveCsvTimer_callback,this)
);
}



//* Callback that processes timestep sent over ROS
void MonocularMode::Timestep_callback(const std_msgs::msg::Float64& time_msg){
    // timeStep = 0; // Initialize
    timeStep = time_msg.data;
}

//* Callback to process image message and run SLAM node
void MonocularMode::Img_callback(const sensor_msgs::msg::Image& msg)
{
    // Initialize
    cv_bridge::CvImagePtr cv_ptr; //* Does not create a copy, memory efficient
    
    //* Convert ROS image to openCV image
    try
    {
        //cv::Mat im =  cv_bridge::toCvShare(msg.img, msg)->image;
        cv_ptr = cv_bridge::toCvCopy(msg); // Local scope
        
        // DEBUGGING, Show image
        // Update GUI Window
        // cv::imshow("test_window", cv_ptr->image);
        // cv::waitKey(3);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(),"Error reading image");
        return;
    }
    
    // std::cout<<std::fixed<<"Timestep: "<<timeStep<<std::endl; // Debug
    
    //* Perform all ORB-SLAM3 operations in Monocular mode
    //! Pose with respect to the camera coordinate frame not the world coordinate frame
    Sophus::SE3f Tcw = pAgent->TrackMonocular(cv_ptr->image, timeStep); 
    
    //* An example of what can be done after the pose w.r.t camera coordinate frame is computed by ORB SLAM3
    //Sophus::SE3f Twc = Tcw.inverse(); //* Pose with respect to global image coordinate, reserved for future use

}

//Added: (Hamdan)
// UNCHANGED signature, now just a thin wrapper around the shared helper 
void MonocularMode::SaveSparseMapPointsPerKeyframe(const std::string& filepath) { 
    std::vector<double> timestamps; // unused here; final save doesn't need to publish anything 
    std::set<long unsigned int> mapIds; //unused here
    WriteKeyframeDataToFile(filepath, timestamps,mapIds); RCLCPP_INFO(this->get_logger(), "Saved sparse map points to %s", filepath.c_str()); }

// ADDED: shared helper, does a full rewrite of filepath each call, returns the
// mTimeStamp of every valid (non-bad) keyframe seen this pass. (Hamdan)
// Update: Now will are also saving map id so that evey key frame can be identified from which map id are they from
void MonocularMode::WriteKeyframeDataToFile(const std::string& filepath, std::vector<double>& outTimestamps,std::set<long unsigned int>& outMapIds)
{
    std::ofstream f(filepath);
    f << std::fixed << std::setprecision(6);
    f << "keyframe_id,timestamp,map_id,pixel_u,pixel_v,depth_camera_frame\n";

    std::vector<ORB_SLAM3::KeyFrame*> keyframes = pAgent->GetAllKeyFrames();

    outTimestamps.clear();
    outTimestamps.reserve(keyframes.size());
    outMapIds.clear();

    for (auto* kf : keyframes)
    {
        if (kf->isBad()) continue;

        outTimestamps.push_back(kf->mTimeStamp); // recorded even if this keyframe has zero usable map points below

        long unsigned int mapId = kf->GetMap()->GetId();
        outMapIds.insert(mapId);

        Sophus::SE3f Tcw = kf->GetPose();
        std::vector<ORB_SLAM3::MapPoint*> mapPoints = kf->GetMapPointMatches();

        for (size_t i = 0; i < mapPoints.size() && i < kf->mvKeysUn.size(); i++)
        {
            ORB_SLAM3::MapPoint* mp = mapPoints[i];
            if (!mp || mp->isBad()) continue;

            Eigen::Vector3f worldPos = mp->GetWorldPos();
            Eigen::Vector3f camPos = Tcw * worldPos;
            float depth = camPos.z();

            if (depth <= 0) continue;

            f << kf->mnId << "," << kf->mTimeStamp << "," << mapId << ","
              << kf->mvKeysUn[i].pt.x << "," << kf->mvKeysUn[i].pt.y << ","
              << depth << "\n";
        }
    }
    f.close();
}

// ADDED: fires every ~1s once VSLAM is running. Full rewrite each tick, so a
// loop-closure correction to an earlier keyframe's pose is reflected on the
// very next tick instead of leaving a stale row behind. (Hamdan)
void MonocularMode::LiveCsvTimer_callback()
{
    if (pAgent == nullptr) return; // watchdog - shouldn't fire before init, but safe to check

    std::vector<double> timestamps;
    std::set<long unsigned int> currentMapIds;
    WriteKeyframeDataToFile(liveFilePath, timestamps,currentMapIds);

    auto msg = std_msgs::msg::Float64MultiArray();
    msg.data = timestamps;
    keyframeTimestamps_publisher_->publish(msg);

    //Detect any change in the set of active maps_ids since last tick
    if (currentMapIds != prevMapIds_){
        auto topologyMsg = std_msgs::msg::Bool();
        topologyMsg.data = true;
        mapTopologyChanged_publisher_->publish(topologyMsg);
        RCLCPP_INFO(this->get_logger(),"Map topology changed: now %zu distinct map_id(s)",currentMapIds.size());
        prevMapIds_ = currentMapIds;
    }
}


