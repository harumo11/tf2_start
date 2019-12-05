#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "my_tf2_broadcaster");
	ros::NodeHandle node;

	tf2_ros::TransformBroadcaster tfb;
	geometry_msgs::TransformStamped transform_stampled;

	transform_stampled.header.frame_id = "turtle1";
	transform_stampled.child_frame_id = "carrot1";
	transform_stampled.transform.translation.x = 0.0;
	transform_stampled.transform.translation.y = 2.0;
	transform_stampled.transform.translation.z = 0.0;
	tf2::Quaternion quat;
	quat.setRPY(0, 0, 0);
	transform_stampled.transform.rotation.x = quat.x();
	transform_stampled.transform.rotation.y = quat.y();
	transform_stampled.transform.rotation.z = quat.z();
	transform_stampled.transform.rotation.w = quat.w();

	ros::Rate rate(10.0);
	while (node.ok()) {
		transform_stampled.header.stamp = ros::Time::now();
		//transform_stampled.transform.translation.x = 2.0 * std::sin(ros::Time::now().toSec());
		//transform_stampled.transform.translation.y = 2.0 * std::sin(ros::Time::now().toSec());
		tfb.sendTransform(transform_stampled);
		rate.sleep();
		ROS_INFO("sending");
	}

	
	return 0;
}
