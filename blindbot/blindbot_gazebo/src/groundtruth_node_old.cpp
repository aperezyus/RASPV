#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#include <time.h>

#include <message_filters/subscriber.h>

struct mainLoop
{


	void gtCallback(const nav_msgs::Odometry::ConstPtr& odometry)
	{
			geometry_msgs::Pose pose_msg = odometry->pose.pose;
			tf::poseMsgToTF(pose_msg, tf_gt_);
//			std::cout << "entrandoCall" << std::endl;

	}

	void startMainLoop(int argc, char* argv[])
	{
		ros::init(argc, argv, "groundtruth_node");
		ros::NodeHandle node;
		std::string topic_name ;
		std::string childFrame;
		std::string parentFrame;
		node.param<std::string>("Topic_name", topic_name,"/ground_truth/state");
		node.param<std::string>("childFrame_id", childFrame,"base_footprint");
		node.param<std::string>("parentFrame_id", parentFrame,"world");
		ros::Subscriber gt_sub_ = node.subscribe(topic_name,1000,&mainLoop::gtCallback,this);

		//message_filters::Subscriber<nav_msgs::Odometry> pose_sub(nh, "/ground_truth/state", 1);

		/*int tries = 0;
		while (gt_sub_.getNumPublishers() == 0)
		{
			ROS_INFO("Waiting for subscribers");
			sleep(1);
			tries++;
			if (tries > 5)
				return;
		}*/

		boost::shared_ptr<nav_msgs::Odometry const> sharedPtr;
        	sharedPtr  = ros::topic::waitForMessage<nav_msgs::Odometry>(topic_name, ros::Duration(5));
        	if (sharedPtr == nullptr) {
            		ROS_INFO("Could not find RGB subscribers");
            		return;
        	}

		tf::TransformBroadcaster br;
		//tf::Transform tf_gt_;
		tf_gt_.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
		tf_gt_.setRotation( tf::Quaternion(0, 0, 0, 1) );

    ros::Rate rate(10000.0);
//		std::cout << "init_node" << std::endl;

		while (node.ok()){
			br.sendTransform(tf::StampedTransform(tf_gt_, ros::Time::now(), parentFrame, childFrame));
			// std::cout << "nodeOK" << std::endl;

      ros::spinOnce();
      rate.sleep();
			//ros::Duration(0.1).sleep();

		}
//		std::cout << "exitWhile" << std::endl;
	}
	//}
	//ros::NodeHandle node;
	tf::Transform tf_gt_;
//	boost::shared_ptr<ros::AsyncSpinner> capture_;

};



int main(int argc, char** argv){

	mainLoop app;
  try
	{
//		std::cout << "appStart" << std::endl;
    app.startMainLoop(argc, argv);
		//std::cout << "appStart" << std::endl;
  }
  catch (const std::bad_alloc& /*e*/)
  {
    std::cout << "Bad alloc" << std::endl;
  }
  catch (const std::exception& /*e*/)
  {
    std::cout << "Exception" << std::endl;
  }

  return 0;
};
