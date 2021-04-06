#include <ros/ros.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/icp.h>
#include <pcl_ros/transforms.h>

class ICPMatcher {
    public:
        ICPMatcher(ros::NodeHandle& nh, int iters, float dist, float trans, float fit);
        bool matchClouds(const sensor_msgs::PointCloud2& template_cloud_msg,
                         const sensor_msgs::PointCloud2& target_cloud_msg,
                         sensor_msgs::PointCloud2& matched_cloud_msg,
                         geometry_msgs::Transform& match_tf, double& match_error) const;

    protected:
        ros::NodeHandle matcher_nh_;
        int iters_;
        float dist_;
        float trans_;
        float fit_;
};