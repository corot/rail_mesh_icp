#include <pcl/registration/transformation_estimation_2D.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/warp_point_rigid_3d.h>

#include "rail_mesh_icp/ICPMatching.h"

ICPMatcher::ICPMatcher(ros::NodeHandle& nh, int iters, float dist, float trans, float fit) {
    matcher_nh_ = nh;
    iters_ = iters;
    dist_ = dist;
    trans_ = trans;
    fit_ = fit;
    pose_srv_ = matcher_nh_.advertiseService("icp_match_clouds", &ICPMatcher::handle_match_clouds_service, this);
}

bool ICPMatcher::handle_match_clouds_service(rail_mesh_icp::ICPMatch::Request& req, rail_mesh_icp::ICPMatch::Response& res) {
    // prepare datastructures
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr template_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr matched_template_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::registration::TransformationEstimation2D<pcl::PointXYZRGB, pcl::PointXYZRGB>::Ptr
      te2D(new pcl::registration::TransformationEstimation2D<pcl::PointXYZRGB, pcl::PointXYZRGB>);

    // loads points clouds
    pcl::fromROSMsg(req.template_cloud,*template_cloud);
    pcl::fromROSMsg(req.target_cloud,*target_cloud);

//
//  pcl::registration::WarpPointRigid3D<pcl::PointXYZRGB, pcl::PointXYZRGB>::Ptr warp_fcn
//    (new pcl::registration::WarpPointRigid3D<pcl::PointXYZRGB, pcl::PointXYZRGB>);
//
//  // Create a TransformationEstimationLM object, and set the warp to it
//  pcl::registration::TransformationEstimationLM<pcl::PointXYZRGB, pcl::PointXYZRGB>::Ptr te(
//    new pcl::registration::TransformationEstimationLM<pcl::PointXYZRGB, pcl::PointXYZRGB>);
//  te->setWarpFunction (warp_fcn);


  // prepare ICP
    pcl::IterativeClosestPointNonLinear<pcl::PointXYZRGB,pcl::PointXYZRGB> icp;
    icp.setInputSource(target_cloud);
    icp.setInputTarget(template_cloud);
    icp.setMaximumIterations(iters_);
    icp.setMaxCorrespondenceDistance(dist_);
    icp.setTransformationEpsilon(trans_);
//    icp.setUseReciprocalCorrespondences(true);
    icp.setRANSACOutlierRejectionThreshold(dist_/2.0);
    icp.setEuclideanFitnessEpsilon(fit_);
    icp.setTransformationEstimation(te2D);

  // Pass the TransformationEstimation objec to the ICP algorithm
//  icp.setTransformationEstimation (te);

    // perform ICP to refine template pose
    try {
        icp.align(*matched_template_cloud);
    } catch (...) {
        ROS_ERROR("Could not match point clouds for given params. Please check ICP params.");
        return false;
    }
    Eigen::Matrix4f icp_tf = icp.getFinalTransformation();
    double fitness_score = icp.getFitnessScore();
    ROS_DEBUG("Clouds matched with error %f.", fitness_score);

    // prepares the response to the service request
    pcl::toROSMsg(*matched_template_cloud,res.matched_template_cloud);
    tf::Transform tf_refinement = tf::Transform(tf::Matrix3x3(icp_tf(0,0),icp_tf(0,1),icp_tf(0,2),
                                                              icp_tf(1,0),icp_tf(1,1),icp_tf(1,2),
                                                              icp_tf(2,0),icp_tf(2,1),icp_tf(2,2)),
                                                  tf::Vector3(icp_tf(0,3),icp_tf(1,3),icp_tf(2,3)));
    tf::Vector3 chuck_trans = tf_refinement.getOrigin();
    tf::Quaternion chuck_rot = tf_refinement.getRotation();
    res.match_tf.translation.x = chuck_trans.x();
    res.match_tf.translation.y = chuck_trans.y();
    res.match_tf.translation.z = chuck_trans.z();
    res.match_tf.rotation.x = chuck_rot.x();
    res.match_tf.rotation.y = chuck_rot.y();
    res.match_tf.rotation.z = chuck_rot.z();
    res.match_tf.rotation.w = chuck_rot.w();
    res.match_error = fitness_score;
    return true;
}
