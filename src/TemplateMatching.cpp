#include "rail_mesh_icp/TemplateMatching.h"

TemplateMatcher::TemplateMatcher(ros::NodeHandle& pnh, std::string& matching_frame, std::string& pcl_topic,
                                 std::string& template_file_path, tf::Transform& initial_estimate,
                                 tf::Transform& template_offset, std::string& template_frame, bool visualize,
                                 bool debug, bool latch, bool pre_processed_cloud, const ICPMatcher& icp_matcher)
               : icp_matcher_(icp_matcher),
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

    // loads template cloud
    template_cloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(template_file_path, *template_cloud_) < 0) {
        ROS_ERROR("Could not load template PCD.");
        exit(-1);
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
}

void TemplateMatcher::matchTemplateGoalCB(MatchTemplateActionServer::GoalHandle goal_handle)
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
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr matched_template_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    tf::Transform initial_estimate;
    if (latched_initial_estimate_) {
        initial_estimate = initial_estimate_;
    } else {
        tf::transformMsgToTF(goal.initial_estimate,initial_estimate);
    }

    // prepares point cloud for matching by transforming by initial_estimate
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr _transformed_template_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl_ros::transformPointCloud(*template_cloud_,*_transformed_template_cloud,initial_estimate);

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

    // prepares sensor_msgs to make ICP request
    sensor_msgs::PointCloud2 template_msg;
    sensor_msgs::PointCloud2 target_msg;
    pcl::toROSMsg(*_transformed_template_cloud,template_msg);
    pcl::toROSMsg(*target_cloud,target_msg);

    // visualizes the transformed point cloud and estimated template pose
    if (debug_) {
        template_msg.header.frame_id = matching_frame_;
        target_msg.header.frame_id = matching_frame_;
        pub_temp_.publish(template_msg);
        pub_targ_.publish(target_msg);
    }

    // makes ICP request
    sensor_msgs::PointCloud2 matched_msg;
    geometry_msgs::Transform match_tf;
    double template_matching_error;
    if (!icp_matcher_.matchClouds(template_msg, target_msg, matched_msg, match_tf, template_matching_error)) {
        ROS_ERROR("Failed to call ICP service.");
        goal_handle.setAborted(result, "Failed to call ICP service");
        return;
    }

    // gets service results
    pcl::fromROSMsg(matched_msg,*matched_template_cloud);
    tf::Transform icp_refinement;
    icp_refinement.setOrigin(tf::Vector3(match_tf.translation.x,
                                         match_tf.translation.y,
                                         match_tf.translation.z));
    icp_refinement.setRotation(tf::Quaternion(match_tf.rotation.x,
                                              match_tf.rotation.y,
                                              match_tf.rotation.z,
                                              match_tf.rotation.w));

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
        sensor_msgs::PointCloud2 matched_template_msg = matched_msg;
        matched_template_msg.header.frame_id = matching_frame_;
        pub_mtemp_.publish(matched_template_msg);
    }

  result.template_pose = final_pose_stamped;
  result.match_error = template_matching_error;
  goal_handle.setSucceeded(result, "Template successfully matched");

  ROS_WARN_STREAM("   <<<   MATCH " << ros::this_node::getName() << " DONE");
}
