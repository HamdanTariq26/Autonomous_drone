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
    //Added: (Hamdan)
    pubCurrentPoseRawName = "/tello_autonomy/current_pose_raw";
    pubCurrentPointsRawName = "/tello_autonomy/current_points_raw";
    pubKeyframePointsName = "/tello_autonomy/keyframe_points";
    pubTrajectoryName = "/tello_autonomy/trajectory";

    // ADDED: handshake QoS must match middleware/topic_manager.py's (Hamdan)
    // HANDSHAKE_QOS exactly (RELIABLE + TRANSIENT_LOCAL + depth 1) on
    // BOTH ends of both handshake topics. QoS durability is negotiated -
    // a TRANSIENT_LOCAL requester can't match a VOLATILE-publishing
    // offerer, so this side must declare the same durability the Python
    // side already does, or the ack subscription can never match at all.
    rclcpp::QoS handshake_qos(1);
    handshake_qos.reliable();
    handshake_qos.transient_local();

//* subscribe to python node to receive settings
expConfig_subscription_ = this->create_subscription<std_msgs::msg::String>(subexperimentconfigName, handshake_qos, std::bind(&MonocularMode::experimentSetting_callback, this, _1));

//* publisher to send out acknowledgement
configAck_publisher_ = this->create_publisher<std_msgs::msg::String>(pubconfigackName, handshake_qos);

    //* subscrbite to the image messages coming from the Python driver node
    subImgMsg_subscription_= this->create_subscription<sensor_msgs::msg::Image>(subImgMsgName, 1, std::bind(&MonocularMode::Img_callback, this, _1));

    //* subscribe to receive the timestep
    subTimestepMsg_subscription_= this->create_subscription<std_msgs::msg::Float64>(subTimestepMsgName, 1, std::bind(&MonocularMode::Timestep_callback, this, _1));
    
    //ADDED: Publisher for the live keyframe timestamp list (Python side will subscribe for grace-period frame deletion) (Hamdan)
    keyframeTimestamps_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(pubKeyframeTimestampsName,10);

    //Added: Publisher when map changes (Hamdan)
    mapTopologyChanged_publisher_ = this->create_publisher<std_msgs::msg::Int32>(pubMapTopologyChangedName, 10);

    //Added: (Hamdan)
    currentPoseRaw_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(pubCurrentPoseRawName, 10);
    currentPointsRaw_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(pubCurrentPointsRawName, 10);
    keyframePoints_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(pubKeyframePointsName, 10);
    trajectory_publisher_ = this->create_publisher<nav_msgs::msg::Path>(pubTrajectoryName, 10);


    
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
    PublishCurrentPoseAndPoints(Tcw);   // ADDED: (Hamdan)
    
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

    std::vector<ORB_SLAM3::KeyFrame*> keyframes;
    std::vector<ORB_SLAM3::Map*> allMaps = pAgent->GetAtlas()->GetAllMaps();

    for (auto* pMap : allMaps){
        if(!pMap) continue;

        std::vector<ORB_SLAM3::KeyFrame*> mapKFs = pMap->GetAllKeyFrames();
        keyframes.insert(keyframes.end(),mapKFs.begin(),mapKFs.end());
    }

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

// ADDED: 
void MonocularMode::LiveCsvTimer_callback()
{
    if (pAgent == nullptr) return;

    std::vector<double> timestamps;
    std::set<long unsigned int> currentMapIds;
    WriteKeyframeDataToFile("/dev/null", timestamps, currentMapIds);  // CHANGED: only used now to collect timestamps/map_ids for the checks below - no longer writes a real file. See note below.

    PublishLiveMapData();  // ADDED - replaces what used to be the CSV write

    auto msg = std_msgs::msg::Float64MultiArray();
    msg.data = timestamps;
    keyframeTimestamps_publisher_->publish(msg);

    if (currentMapIds != prevMapIds_) {
        auto topologyMsg = std_msgs::msg::Int32();
        topologyMsg.data = static_cast<int32_t>(currentMapIds.size());
        mapTopologyChanged_publisher_->publish(topologyMsg);
        RCLCPP_INFO(this->get_logger(), "Map topology changed: now %zu distinct map_id(s)", currentMapIds.size());
        prevMapIds_ = currentMapIds;
    }
}


//Added:
void MonocularMode::PublishCurrentPoseAndPoints(const Sophus::SE3f& Tcw)
{
    if (pAgent == nullptr) return;

    long unsigned int mapId = pAgent->GetAtlas()->GetCurrentMap()->GetId();
    std::string frameIdStr = "slam_map_" + std::to_string(mapId);  // must match config.constants.SLAM_MAP_FRAME_ID_PREFIX

    // ---- Pose: Twc (camera position/orientation in world frame) ----
    Sophus::SE3f Twc = Tcw.inverse();
    Eigen::Vector3f t = Twc.translation();
    Eigen::Quaternionf q = Twc.unit_quaternion();

    geometry_msgs::msg::PoseStamped poseMsg;
    poseMsg.header.stamp = this->now();
    poseMsg.header.frame_id = frameIdStr;
    poseMsg.pose.position.x = t.x();
    poseMsg.pose.position.y = t.y();
    poseMsg.pose.position.z = t.z();
    poseMsg.pose.orientation.x = q.x();
    poseMsg.pose.orientation.y = q.y();
    poseMsg.pose.orientation.z = q.z();
    poseMsg.pose.orientation.w = q.w();
    currentPoseRaw_publisher_->publish(poseMsg);

    // ---- Currently tracked map points, camera-frame 3D (arbitrary SLAM units) ----
    std::vector<ORB_SLAM3::MapPoint*> trackedPoints = pAgent->GetTrackedMapPoints();
    std::vector<Eigen::Vector3f> camPoints;
    camPoints.reserve(trackedPoints.size());
    for (auto* mp : trackedPoints)
    {
        if (!mp || mp->isBad()) continue;
        Eigen::Vector3f camPos = Tcw * mp->GetWorldPos();
        if (camPos.z() <= 0) continue;  // behind camera - invalid
        camPoints.push_back(camPos);
    }

    sensor_msgs::msg::PointCloud2 cloudMsg;
    cloudMsg.header.stamp = this->now();
    cloudMsg.header.frame_id = frameIdStr;
    cloudMsg.height = 1;
    cloudMsg.is_dense = true;
    cloudMsg.is_bigendian = false;

    sensor_msgs::PointCloud2Modifier modifier(cloudMsg);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(camPoints.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloudMsg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloudMsg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloudMsg, "z");
    for (const auto& p : camPoints)
    {
        *iter_x = p.x(); *iter_y = p.y(); *iter_z = p.z();
        ++iter_x; ++iter_y; ++iter_z;
    }
    currentPointsRaw_publisher_->publish(cloudMsg);
}


//Added
void MonocularMode::PublishLiveMapData()
{
    if (pAgent == nullptr) return;

    // CRITICAL: loop every map in the Atlas, NOT GetAllKeyframePoses()/
    // Atlas::GetAllKeyFrames() - both of those silently only return the
    // CURRENTLY ACTIVE map, the exact bug already found and fixed once
    // before in WriteKeyframeDataToFile(). Do not reintroduce it here.
    std::vector<ORB_SLAM3::Map*> allMaps = pAgent->GetAtlas()->GetAllMaps();

    for (auto* pMap : allMaps)
    {
        if (!pMap) continue;
        long unsigned int mapId = pMap->GetId();
        std::string frameIdStr = "slam_map_" + std::to_string(mapId);

        std::vector<ORB_SLAM3::KeyFrame*> keyframes = pMap->GetAllKeyFrames();
        std::sort(keyframes.begin(), keyframes.end(), ORB_SLAM3::KeyFrame::lId);

        // ---- keyframe_points: (keyframe_id, keyframe_timestamp, pixel_u, pixel_v, depth) per valid map point ----
        std::vector<std::array<double,5>> rows;
        for (auto* kf : keyframes)
        {
            if (kf->isBad()) continue;
            Sophus::SE3f Tcw = kf->GetPose();
            std::vector<ORB_SLAM3::MapPoint*> mapPoints = kf->GetMapPointMatches();

            for (size_t i = 0; i < mapPoints.size() && i < kf->mvKeysUn.size(); i++)
            {
                ORB_SLAM3::MapPoint* mp = mapPoints[i];
                if (!mp || mp->isBad()) continue;
                Eigen::Vector3f camPos = Tcw * mp->GetWorldPos();
                float depth = camPos.z();
                if (depth <= 0) continue;

                rows.push_back({
                    static_cast<double>(kf->mnId),
                    kf->mTimeStamp,
                    static_cast<double>(kf->mvKeysUn[i].pt.x),
                    static_cast<double>(kf->mvKeysUn[i].pt.y),
                    static_cast<double>(depth)
                });
            }
        }

        sensor_msgs::msg::PointCloud2 cloudMsg;
        cloudMsg.header.stamp = this->now();
        cloudMsg.header.frame_id = frameIdStr;
        cloudMsg.height = 1;
        cloudMsg.is_dense = true;
        cloudMsg.is_bigendian = false;

        sensor_msgs::PointCloud2Modifier modifier(cloudMsg);
        modifier.setPointCloud2Fields(5,
            "keyframe_id", 1, sensor_msgs::msg::PointField::FLOAT64,
            "keyframe_timestamp", 1, sensor_msgs::msg::PointField::FLOAT64,
            "pixel_u", 1, sensor_msgs::msg::PointField::FLOAT64,
            "pixel_v", 1, sensor_msgs::msg::PointField::FLOAT64,
            "depth", 1, sensor_msgs::msg::PointField::FLOAT64);
        modifier.resize(rows.size());

        sensor_msgs::PointCloud2Iterator<double> it_kfid(cloudMsg, "keyframe_id");
        sensor_msgs::PointCloud2Iterator<double> it_ts(cloudMsg, "keyframe_timestamp");
        sensor_msgs::PointCloud2Iterator<double> it_u(cloudMsg, "pixel_u");
        sensor_msgs::PointCloud2Iterator<double> it_v(cloudMsg, "pixel_v");
        sensor_msgs::PointCloud2Iterator<double> it_depth(cloudMsg, "depth");

        for (const auto& row : rows)
        {
            *it_kfid = row[0]; *it_ts = row[1]; *it_u = row[2]; *it_v = row[3]; *it_depth = row[4];
            ++it_kfid; ++it_ts; ++it_u; ++it_v; ++it_depth;
        }
        keyframePoints_publisher_->publish(cloudMsg);

        // ---- trajectory: nav_msgs/Path, fixes the GetAllKeyframePoses() single-map bug ----
        nav_msgs::msg::Path pathMsg;
        pathMsg.header.stamp = this->now();
        pathMsg.header.frame_id = frameIdStr;

        for (auto* kf : keyframes)
        {
            if (kf->isBad()) continue;
            Sophus::SE3f Twc = kf->GetPoseInverse();
            Eigen::Vector3f t = Twc.translation();
            Eigen::Quaternionf q = Twc.unit_quaternion();

            geometry_msgs::msg::PoseStamped ps;
            ps.header.stamp = rclcpp::Time(static_cast<int64_t>(kf->mTimeStamp * 1e9));
            ps.header.frame_id = frameIdStr;
            ps.pose.position.x = t.x();
            ps.pose.position.y = t.y();
            ps.pose.position.z = t.z();
            ps.pose.orientation.x = q.x();
            ps.pose.orientation.y = q.y();
            ps.pose.orientation.z = q.z();
            ps.pose.orientation.w = q.w();
            pathMsg.poses.push_back(ps);
        }
        trajectory_publisher_->publish(pathMsg);
    }
}


