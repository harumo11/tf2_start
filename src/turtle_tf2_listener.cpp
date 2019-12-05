#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "my_tf2_listener");
	ros::NodeHandle node;

	ros::service::waitForService("spawn");
	ros::ServiceClient spawner = node.serviceClient<turtlesim::Spawn>("spawn");
	turtlesim::Spawn turtle;
	turtle.request.x = 4;
	turtle.request.y = 2;
	turtle.request.theta = 0;
	turtle.request.name = "turtle2";
	spawner.call(turtle);

	ros::Publisher turtle_vel = node.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 10);

	tf2_ros::Buffer tf_buffer;
	tf2_ros::TransformListener tf_listener(tf_buffer);

	ros::Rate rate(10);
	while (node.ok()) {
		geometry_msgs::TransformStamped transform_stamped;
		try {
			transform_stamped = tf_buffer.lookupTransform("turtle2", "carrot1", ros::Time(0));
		} catch (tf2::TransformException &ex) {
			ROS_WARN_STREAM(ex.what());
			ros::Duration(1.0).sleep();
			continue;
		}

		geometry_msgs::Twist vel_msgs;
		ROS_INFO_STREAM(transform_stamped);
		vel_msgs.angular.z = 4.0 * std::atan2(transform_stamped.transform.translation.y, transform_stamped.transform.translation.x);
		vel_msgs.linear.x = 0.5 * std::sqrt(std::pow(transform_stamped.transform.translation.x, 2) + std::pow(transform_stamped.transform.translation.y, 2));
		turtle_vel.publish(vel_msgs);
		ROS_INFO_STREAM(vel_msgs);

		rate.sleep();
	}
	
	return 0;
}
