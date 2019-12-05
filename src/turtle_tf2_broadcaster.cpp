//このプログラムはあなたがどのようにロボットの座標フレームをtf2に流すかを教えます．

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>		
#include <tf2_ros/transform_broadcaster.h>	//poseをtf2にpublishする関連. TransformBroadcasterを使えるようになる
#include <geometry_msgs/TransformStamped.h>
#include <turtlesim/Pose.h>

std::string turtle_name;

void pose_callback(const turtlesim::Pose msg){
	static tf2_ros::TransformBroadcaster br;
	geometry_msgs::TransformStamped transform_stamped;

	transform_stamped.header.stamp = ros::Time::now();
	transform_stamped.header.frame_id = "world";
	transform_stamped.child_frame_id = turtle_name;
	transform_stamped.transform.translation.x = msg.x;
	transform_stamped.transform.translation.y = msg.y;
	transform_stamped.transform.translation.z = 0.0;
	tf2::Quaternion q;
	q.setRPY(0, 0, msg.theta);
	transform_stamped.transform.rotation.x = q.x();
	transform_stamped.transform.rotation.y = q.y();
	transform_stamped.transform.rotation.z = q.z();
	transform_stamped.transform.rotation.w = q.w();

	br.sendTransform(transform_stamped);
}

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "my_tf_broadcaster");
	ros::NodeHandle private_node("~");
	if (!private_node.hasParam("turtle")) {
		if (argc != 2) {
			ROS_ERROR_STREAM("need turtle name as argument");
			return -1;
		}
		turtle_name = argv[1];
	}
	else {
		private_node.getParam("turtle", turtle_name);
	}

	ros::NodeHandle node;
	ros::Subscriber sub = node.subscribe(turtle_name+"/pose", 10, &pose_callback);

	ros::spin();
	return 0;
}
