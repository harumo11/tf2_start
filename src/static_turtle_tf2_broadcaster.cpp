#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

std::string static_turtle_name;

int main(int argc, char* argv[])
{
	// rosノードの初期化
	ros::init(argc, argv, "my_static_tf2_broadcaster");

	if (argc != 8) {
		ROS_ERROR_STREAM("Invalid of arguments" << std::endl << "usage : static_turtle_tf2_broadcaster child_frame_name x y z roll pitch yaw");
		return -1;
	}

	//プログラム名を実行ファイル名から取る
	static_turtle_name = argv[1];
	static tf2_ros::StaticTransformBroadcaster static_broadcaster;
	geometry_msgs::TransformStamped static_transform_stamped;

	//header config
	static_transform_stamped.header.stamp = ros::Time::now();
	static_transform_stamped.header.frame_id = "world";
	static_transform_stamped.child_frame_id = static_turtle_name;
	//translation config
	static_transform_stamped.transform.translation.x = std::atof(argv[2]);
	static_transform_stamped.transform.translation.y = std::atof(argv[3]);
	static_transform_stamped.transform.translation.z = std::atof(argv[4]);
	//rotation config
	tf2::Quaternion quat;
	quat.setRPY(std::atof(argv[5]), std::atof(argv[6]), std::atof(argv[7]));
	static_transform_stamped.transform.rotation.x = quat.x();
	static_transform_stamped.transform.rotation.y = quat.y();
	static_transform_stamped.transform.rotation.z = quat.z();
	static_transform_stamped.transform.rotation.w = quat.w();
	static_broadcaster.sendTransform(static_transform_stamped);
	ROS_INFO_STREAM("Spinning until killed publishing " << static_turtle_name << "to world");
	ros::spin();

	return 0;
}
