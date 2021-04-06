#include <ros/time.h>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <geometry_msgs/Transform.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <actionlib/server/action_server.h>

///#include <thread_pool.hpp>
#include "rail_mesh_icp/ThreadPool.h"
#include "rail_mesh_icp/ICPMatching.h"
#include "rail_mesh_icp/MatchTemplateAction.h"


class TemplateMatcher {
    public:
        TemplateMatcher(ros::NodeHandle& pnh, std::string& matching_frame, std::string& pcl_topic,
                        std::string& template_file_path, tf::Transform& initial_estimate,
                        tf::Transform& template_offset, std::string& template_frame, bool visualize,
                        bool debug, bool latch, bool pre_processed_cloud, const ICPMatcher& icp_matcher);

        typedef actionlib::ActionServer<rail_mesh_icp::MatchTemplateAction> MatchTemplateActionServer;
        // handles requests to match a template CAD model (in PCD form) to a point cloud from a point cloud topic
        void matchTemplateGoalCB(MatchTemplateActionServer::GoalHandle goal_handle);

    protected:
        std::string matching_frame_;
        std::string template_frame_;
        std::string pcl_topic_;
        tf::TransformListener tf_;
        tf::Transform initial_estimate_;
        tf::Transform template_offset_;
        std::vector<std::pair<std::string, pcl::PointCloud<pcl::PointXYZRGB>::Ptr>> template_clouds_;
        bool viz_;
        bool debug_;
        bool latched_initial_estimate_;
        bool pre_processed_cloud_;
        ros::Publisher pub_temp_;
        ros::Publisher pub_targ_;
        ros::Publisher pub_mtemp_;
        const ICPMatcher& icp_matcher_;

        ThreadPool thread_pool_;
        //thread_pool thread_pool_;
        actionlib::ActionServer<rail_mesh_icp::MatchTemplateAction> as_;

        tf2_ros::StaticTransformBroadcaster static_broadcaster;

        void matchTemplate(MatchTemplateActionServer::GoalHandle goal_handle);
};
