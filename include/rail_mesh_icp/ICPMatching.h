#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <geometry_msgs/Transform.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class ICPMatcher {
    public:
        ICPMatcher(int iters, float dist, float trans, float fit);
        bool matchClouds(const PointCloudT::ConstPtr& template_cloud,
                         const PointCloudT::ConstPtr& target_cloud,
                         PointCloudT::Ptr& matched_template_cloud,
                         geometry_msgs::Transform& match_tf, double& match_error) const;

    protected:
        int iters_;
        float dist_;
        float trans_;
        float fit_;
};