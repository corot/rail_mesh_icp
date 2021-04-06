#include <experimental/filesystem>

#include "rail_mesh_icp/TemplateMatching.h"

TemplateMatcher::TemplateMatcher(ros::NodeHandle& pnh, std::string& matching_frame, std::string& pcl_topic,
                                 std::string& template_files_path, tf::Transform& initial_estimate,
                                 tf::Transform& template_offset, std::string& template_frame, bool visualize,
                                 bool debug, bool latch, bool pre_processed_cloud, const ICPMatcher& icp_matcher)
               : icp_matcher_(icp_matcher), thread_pool_(12),
                 as_(pnh, "match_template", boost::bind(&TemplateMatcher::matchTemplateGoalCB, this, _1), false)
{
    matching_frame_ = matching_frame;
    pcl_topic_ = pcl_topic;
    pre_processed_cloud_ = pre_processed_cloud;
    initial_estimate_ = initial_estimate;
    latched_initial_estimate_ = latch;
    template_offset_ = template_offset;
    template_frame_ = template_frame;
    debug_ = debug;
    viz_ = visualize;

    // load template clouds
    ROS_INFO("Loaded templates: ");
    for (const auto& template_file : std::experimental::filesystem::directory_iterator(template_files_path))
    {
      if (template_file.path().extension() == ".pcd")
      {
        std::string template_name = template_file.path().stem().string();
        ROS_INFO_STREAM(" - " << template_name);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr template_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
        if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(template_file.path().string(), *template_cloud) < 0)
        {
          ROS_WARN_STREAM("Could not load template PCD " << template_file.path());
        }
        template_clouds_.emplace_back(std::make_pair(template_name, template_cloud));
      }
    }

    // visualization publishers
    pub_temp_ = pnh.advertise<sensor_msgs::PointCloud2>("/template_matcher/template_points",0);
    pub_targ_ = pnh.advertise<sensor_msgs::PointCloud2>("/template_matcher/target_points",0);
    pub_mtemp_ = pnh.advertise<sensor_msgs::PointCloud2>("/template_matcher/matched_points",0);

    // creates service handler for template matching
    //pose_srv_ = pnh.advertiseService("match_template", &TemplateMatcher::handle_match_template, this);
    //as_.registerGoalCallback();
    ///as_.registerPreemptCallback(boost::bind(&TemplateMatcher::preemptCB, this));
 /// boost::bind(&TemplateMatcher::matchTemplateGoalCB, this)
    as_.start();

    ROS_INFO("Template matcher ready; %lu templated loaded", template_clouds_.size());
}

void TemplateMatcher::matchTemplateGoalCB(MatchTemplateActionServer::GoalHandle goal_handle)
{
  ////thread_pool_.submit(boost::bind(&TemplateMatcher::matchTemplate, this, _1), goal_handle);
  ////std::async(&TemplateMatcher::matchTemplate, this, goal_handle);


// enqueue and store future
  //auto result =
      thread_pool_.enqueue(boost::bind(&TemplateMatcher::matchTemplate, this, _1), goal_handle);

// get result from future
  //std::cout << result.get() << std::endl;
}

void TemplateMatcher::matchTemplate(MatchTemplateActionServer::GoalHandle goal_handle)
{
  const rail_mesh_icp::MatchTemplateGoal& goal = *goal_handle.getGoal();

//  goal_handle.setAccepted();

  ROS_ERROR_STREAM("   >>>   MATCH " << ros::this_node::getName() << " STARTED   "<<goal.target_cloud.height << "   " <<goal.target_cloud.width
                   << "   "<<   goal_handle.isValid() << "   "<<   (int)goal_handle.getGoalStatus().status);
   // rail_mesh_icp::MatchTemplateGoal goal = *as_.acceptNewGoal();
    goal_handle.setAccepted();
  ROS_ERROR_STREAM( "               "<<   (int)goal_handle.getGoalStatus().status);
    rail_mesh_icp::MatchTemplateResult result;

    // declare data structures
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
              ////       pcl::PointCloud<pcl::PointXYZRGB>::Ptr matched_template_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    tf::Transform initial_estimate;
    if (latched_initial_estimate_) {
        initial_estimate = initial_estimate_;
    } else {
        tf::transformMsgToTF(goal.initial_estimate,initial_estimate);
    }

    // acquire target cloud from camera
    sensor_msgs::PointCloud2 target_cloud_msg;
    if (!pre_processed_cloud_) {
        ros::Time request_time = ros::Time::now();
        ros::Time point_cloud_time = request_time - ros::Duration(0.1);

        while (point_cloud_time < request_time)
        {
            // gets current point cloud from pcl_topic_
            boost::shared_ptr<sensor_msgs::PointCloud2 const> sharedMsg;
            sharedMsg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(pcl_topic_);
            if(sharedMsg != NULL){
                point_cloud_time = sharedMsg->header.stamp;
                target_cloud_msg = *sharedMsg;
            } else {
                ROS_ERROR("Could not get point cloud message from topic. Boost shared pointer is NULL.");
                goal_handle.setAborted(result, "Could not get point cloud message from topic");
                return;
            }
        }
    } else {
        // gets pre-processed point cloud from template match request
        target_cloud_msg = goal.target_cloud;
    }
    pcl::fromROSMsg(target_cloud_msg,*target_cloud);

    // transforms point cloud to the matching frame
    //if (matching_frame_ != target_cloud_msg.header.frame_id) {
    pcl_ros::transformPointCloud(matching_frame_, ros::Time(0), *target_cloud, target_cloud_msg.header.frame_id,
                                     *target_cloud, tf_);
    //}


    std::vector<double> match_errors(template_clouds_.size());
    std::vector<geometry_msgs::Transform> match_tfs(template_clouds_.size());
    std::vector<sensor_msgs::PointCloud2> match_clouds(template_clouds_.size());

    // Call template matching services in parallel with the object pointcloud, one call per template
    // Note that we log all errors multiplied by 1e6, as the values provided by the template matcher are very low
    #pragma omp parallel for    // NOLINT
    for (size_t i = 0; i < template_clouds_.size(); ++i)
    {
     // const auto& template_name = template_clouds_[i].first;
      const auto& template_cloud = template_clouds_[i].second;

      // prepares point cloud for matching by transforming by initial_estimate
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr _transformed_template_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl_ros::transformPointCloud(*template_cloud, *_transformed_template_cloud, initial_estimate);

      // prepares sensor_msgs to make ICP request
      sensor_msgs::PointCloud2 template_msg;
      sensor_msgs::PointCloud2 target_msg;
      pcl::toROSMsg(*_transformed_template_cloud,template_msg);
      pcl::toROSMsg(*target_cloud,target_msg);

      // makes ICP request
//      geometry_msgs::Transform match_tf;
//      double template_matching_error;
      if (!icp_matcher_.matchClouds(template_msg, target_msg, match_clouds[i], match_tfs[i], match_errors[i]))
      {
        match_errors[i] = std::numeric_limits<double>::max();
        continue;
      }

      #pragma omp critical
      // visualizes the transformed point cloud and estimated template pose
      if (debug_) {
        template_msg.header.frame_id = matching_frame_;
        target_msg.header.frame_id = matching_frame_;
        pub_temp_.publish(template_msg);
        pub_targ_.publish(target_msg);
      }

//
//      // provide center as initial estimate for position, but use 0, 0, 0 for orientation,
//      // as it's a good guideline only for elongated objects where PCA is meaningful
//      rail_mesh_icp::MatchTemplateGoal goal;
//      goal.initial_estimate.translation.x = obj.center.x;
//      goal.initial_estimate.translation.y = obj.center.y;
//      goal.initial_estimate.translation.z = obj.center.z - obj.height / 2.0; // templates' base is at z = 0
//      goal.initial_estimate.rotation.w = 1.0;
//      goal.target_cloud = obj.point_cloud;
//      if (match_acs_[obj_type]->sendGoalAndWait(goal, ros::Duration(2)) == actionlib::SimpleClientGoalState::SUCCEEDED)
//      {
//        rail_mesh_icp::MatchTemplateResult::ConstPtr result = match_acs_[obj_type]->getResult();
//        ROS_DEBUG_STREAM("[object detection] " << obj_type << " template match succeeded; error: "
//                                               << result->match_error * 1e6);
//  #pragma omp critical
//        error[obj_type] = result->match_error;
//        poses[obj_type] = result->template_pose;
//      }
//      else
//      {
//        ROS_ERROR_STREAM("[object detection] " << obj_type << " template match failed");
//  #pragma omp critical
//        error[obj_type] = 1.0;
//      }
    }

    // choose the template with the lowest error (PCL ICP call it score, very bad name imho)
    int best_match = std::min_element(match_errors.begin(), match_errors.end()) - match_errors.begin();;
//    // check if best match is good enough
//    if (best_match->second > max_matching_error_)
//    {
//      ROS_INFO_STREAM("[object detection] Best match (" << best_match->first << ") error above tolerance ("
//                                                        << best_match->second * 1e6 << " > " << max_matching_error_ * 1e6 << ")");
//      return false;
//    }

//    ROS_INFO_STREAM("[object detection] Best match is " << best_match->first << ", with error "
//                                                        << best_match->second * 1e6);


    // gets service results
                    //               pcl::fromROSMsg(match_clouds[best_match], *matched_template_cloud);
    tf::Transform icp_refinement;
    icp_refinement.setOrigin(tf::Vector3(match_tfs[best_match].translation.x,
                                         match_tfs[best_match].translation.y,
                                         match_tfs[best_match].translation.z));
    icp_refinement.setRotation(tf::Quaternion(match_tfs[best_match].rotation.x,
                                              match_tfs[best_match].rotation.y,
                                              match_tfs[best_match].rotation.z,
                                              match_tfs[best_match].rotation.w));

    // calculates the final estimated tf in the matching frame
    tf::Transform tf_final = icp_refinement.inverse() * initial_estimate * template_offset_;

    // prepares the service response
    tf::Vector3 final_trans = tf_final.getOrigin();
    tf::Quaternion final_rot = tf_final.getRotation();
    geometry_msgs::TransformStamped final_pose_stamped;
    final_pose_stamped.header.stamp = ros::Time::now();
    final_pose_stamped.header.frame_id = matching_frame_;
    final_pose_stamped.child_frame_id = template_frame_;
    final_pose_stamped.transform.translation.x = final_trans.x();
    final_pose_stamped.transform.translation.y = final_trans.y();
    final_pose_stamped.transform.translation.z = final_trans.z();
    final_pose_stamped.transform.rotation.x = final_rot.x();
    final_pose_stamped.transform.rotation.y = final_rot.y();
    final_pose_stamped.transform.rotation.z = final_rot.z();
    final_pose_stamped.transform.rotation.w = final_rot.w();

    // visualizes the matched point cloud and final estimated pose
    if (viz_)
    {
        static_broadcaster.sendTransform(final_pose_stamped);
    }
    if (debug_)
    {
        sensor_msgs::PointCloud2 matched_template_msg = match_clouds[best_match];
        matched_template_msg.header.frame_id = matching_frame_;
        pub_mtemp_.publish(matched_template_msg);
    }

  result.template_pose = final_pose_stamped;
  result.template_name = template_clouds_[best_match].first;
  result.match_error = match_errors[best_match];
  goal_handle.setSucceeded(result, "Template successfully matched");

  ROS_WARN_STREAM("   <<<   MATCH " << ros::this_node::getName() << " DONE  "<<goal.target_cloud.height << "   " <<goal.target_cloud.width);
}
