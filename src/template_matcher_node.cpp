#include <tf/tf.h>
#include <ros/ros.h>
#include <ros/package.h>

#include "rail_mesh_icp/TemplateMatching.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "template_matcher_node");
    ros::NodeHandle nh, pnh("~");

    // sets the default params
    std::string matching_frame = "map";
    std::string pcl_topic = "/head_camera/depth_registered/points";
    std::string initial_estimate_string = "0 0 0 0 0 0";
    std::string template_offset_string = "0 0 0 0 0 0";
    std::string template_frame = "template_pose";
    bool visualize = true;
    bool debug = true;
    bool latched = true;
    bool pre_processed_cloud = false;

    std::string template_files_path = ros::package::getPath("rail_mesh_icp") + "/cad_models/";

    // gets roslaunch params
    pnh.getParam("matching_frame", matching_frame);
    pnh.getParam("pcl_topic", pcl_topic);
    pnh.getParam("template_files_path", template_files_path);
    pnh.getParam("initial_estimate_string", initial_estimate_string);
    pnh.getParam("template_offset_string", template_offset_string);
    pnh.getParam("template_frame", template_frame);
    pnh.getParam("visualize", visualize);
    pnh.getParam("debug", debug);
    pnh.getParam("latch_initial", latched);
    pnh.getParam("pre_processed_cloud", pre_processed_cloud);

    // gets the initial_estimate for schunk corner from the launch
    tf::Transform initial_estimate;
    tf::Transform template_offset;

    // initializes a tf for the initial_estimate
    std::vector<float> pose;
    std::istringstream initial_estimate_string_stream(initial_estimate_string);
    for(std::string value_string; initial_estimate_string_stream >> value_string;)
        pose.push_back(std::stof(value_string));
    initial_estimate.setOrigin(tf::Vector3(pose[0],pose[1],pose[2]));
    initial_estimate.setRotation(tf::Quaternion(pose[4],pose[5],pose[3]));

    // initializes a tf for the template_offset
    std::vector<float> offset;
    std::istringstream offset_string_stream(template_offset_string);
    for(std::string value_string; offset_string_stream >> value_string;)
        offset.push_back(std::stof(value_string));
    template_offset.setOrigin(tf::Vector3(offset[0],offset[1],offset[2]));
    template_offset.setRotation(tf::Quaternion(offset[4],offset[5],offset[3]));

    // loads ICP params
    int iters = 50;
    float dist = 1.0;
    float trans = 1e-8;
    float fit = 1e-8;
    pnh.getParam("iterations",iters);
    pnh.getParam("max_distance",dist);
    pnh.getParam("trans_epsilon",trans);
    pnh.getParam("fit_epsilon",fit);

    // start the ICP matcher
    ICPMatcher icp_matcher(iters, dist, trans, fit);

    // starts a template matcher
    TemplateMatcher matcher(pnh,matching_frame,pcl_topic,template_files_path,initial_estimate,template_offset,template_frame,
                            visualize,debug,latched,pre_processed_cloud, icp_matcher);

    ros::spin();

    return 0;
}
