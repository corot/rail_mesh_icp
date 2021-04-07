#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <geometry_msgs/Transform.h>

class ICPMatcher {
    public:
        ICPMatcher(int iters, float dist, float trans, float fit);
        bool matchClouds(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& template_cloud,
                         const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& target_cloud,
                         pcl::PointCloud<pcl::PointXYZRGB>::Ptr& matched_template_cloud,
                         geometry_msgs::Transform& match_tf, double& match_error) const;

    protected:
        int iters_;
        float dist_;
        float trans_;
        float fit_;
};