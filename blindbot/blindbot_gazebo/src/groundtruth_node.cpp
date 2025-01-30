#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

class GroundTruthNode
{
public:
    GroundTruthNode()
    {
        ros::NodeHandle node;

        node.param<std::string>("Topic_name", topic_name_, "/ground_truth/state");
        node.param<std::string>("childFrame_id", child_frame_, "base_footprint");
        node.param<std::string>("parentFrame_id", parent_frame_, "world");

        gt_sub_ = node.subscribe(topic_name_, 1000, &GroundTruthNode::gtCallback, this);
    }

    void gtCallback(const nav_msgs::Odometry::ConstPtr& odometry)
    {
        static tf::TransformBroadcaster br;  // Static local broadcaster
        geometry_msgs::Pose pose_msg = odometry->pose.pose;
        tf::Transform tf_gt;
        tf::poseMsgToTF(pose_msg, tf_gt);

        // Use the odometry message's timestamp
        ros::Time tf_time = odometry->header.stamp;

        br.sendTransform(tf::StampedTransform(tf_gt, tf_time, parent_frame_, child_frame_));
    }

    void spin()
    {
        ros::spin();
    }

private:
    ros::Subscriber gt_sub_;
    std::string topic_name_;
    std::string child_frame_;
    std::string parent_frame_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "groundtruth_node");  // Initialize ROS

    GroundTruthNode node;
    node.spin();

    return 0;
}
