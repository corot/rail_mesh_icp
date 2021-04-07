#include <pcl_ros/transforms.h>
#include <pcl/registration/transformation_estimation_2D.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/warp_point_rigid_3d.h>

#include "rail_mesh_icp/ICPMatching.h"

ICPMatcher::ICPMatcher(int iters, float dist, float trans, float fit) {
    iters_ = iters;
    dist_ = dist;
    trans_ = trans;
    fit_ = fit;
}

bool ICPMatcher::matchClouds(const PointCloudT::ConstPtr& template_cloud,
                             const PointCloudT::ConstPtr& target_cloud,
                             PointCloudT::Ptr& matched_template_cloud,
                             geometry_msgs::Transform& match_tf, double& match_error) const
{
    // Pass a 2D TransformationEstimation to the ICP algorithm
    pcl::registration::TransformationEstimation2D<PointT, PointT>::Ptr
      te2D(new pcl::registration::TransformationEstimation2D<PointT, PointT>);

    // prepare ICP
    pcl::IterativeClosestPointNonLinear<PointT, PointT> icp;
    icp.setInputSource(target_cloud);
    icp.setInputTarget(template_cloud);
    icp.setMaximumIterations(iters_);
    icp.setMaxCorrespondenceDistance(dist_);
    icp.setTransformationEpsilon(trans_);
    icp.setUseReciprocalCorrespondences(false);
    icp.setRANSACOutlierRejectionThreshold(dist_/2.0);
    icp.setEuclideanFitnessEpsilon(fit_);
    icp.setTransformationEstimation(te2D);

    // perform ICP to refine template pose
    matched_template_cloud.reset(new PointCloudT);
    try {
        icp.align(*matched_template_cloud);
    } catch (...) {
        ROS_ERROR("Could not match point clouds for given params. Please check ICP params.");
        return false;
    }
    Eigen::Matrix4f icp_tf = icp.getFinalTransformation();
    double fitness_score = icp.getFitnessScore();
    ROS_DEBUG("Clouds matched with error %f.", fitness_score);

    tf::Transform tf_refinement = tf::Transform(tf::Matrix3x3(icp_tf(0,0),icp_tf(0,1),icp_tf(0,2),
                                                              icp_tf(1,0),icp_tf(1,1),icp_tf(1,2),
                                                              icp_tf(2,0),icp_tf(2,1),icp_tf(2,2)),
                                                  tf::Vector3(icp_tf(0,3),icp_tf(1,3),icp_tf(2,3)));
    tf::Vector3 chuck_trans = tf_refinement.getOrigin();
    tf::Quaternion chuck_rot = tf_refinement.getRotation();
    match_tf.translation.x = chuck_trans.x();
    match_tf.translation.y = chuck_trans.y();
    match_tf.translation.z = chuck_trans.z();
    match_tf.rotation.x = chuck_rot.x();
    match_tf.rotation.y = chuck_rot.y();
    match_tf.rotation.z = chuck_rot.z();
    match_tf.rotation.w = chuck_rot.w();
    match_error = fitness_score;
    return true;
}
