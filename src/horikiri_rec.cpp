#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <eigen3/Eigen/Dense>
#include <fstream>

int main(int argc, char* argv[])
{
	// ros initialize
	ros::init(argc, argv, "horikiri_rec_node");
	ros::NodeHandle node;

	// tf2 settings
	tf2_ros::Buffer tf_buffer_waist;
	tf2_ros::Buffer tf_buffer_shoulder;
	tf2_ros::Buffer tf_buffer_between_waist_to_shoulder;
	tf2_ros::TransformListener tf_listener_waist(tf_buffer_waist);
	tf2_ros::TransformListener tf_listener_shoulder(tf_buffer_shoulder);
	tf2_ros::TransformListener tf_listener_between_waist_to_shoulder(tf_buffer_between_waist_to_shoulder);

	// HTC Vive config
	const std::string controller_id_waist = "controller_LHR_066549FF";
	const std::string controller_id_shoulder = "controller_LHR_3CCD7CA5";

	// file config
	std::ofstream file_waist("waist_log.csv");
	std::ofstream file_shoulder("waist_log.csv");
	std::ofstream file_btw("between_log.csv");

	ros::Rate rate(100);
	while (node.ok()) {
		geometry_msgs::TransformStamped transform_stamped_waist;
		geometry_msgs::TransformStamped transform_stamped_shoulder;
		geometry_msgs::TransformStamped transform_stamped_between_waist_to_shoulder;

		try {
			transform_stamped_waist = tf_buffer_waist.lookupTransform("world", controller_id_waist, ros::Time(0));
			transform_stamped_shoulder = tf_buffer_shoulder.lookupTransform("world", controller_id_shoulder, ros::Time(0));
			transform_stamped_between_waist_to_shoulder = tf_buffer_between_waist_to_shoulder.lookupTransform(controller_id_waist, controller_id_shoulder, ros::Time(0));
		} catch (tf2::TransformException& ex) {
			ROS_WARN_STREAM(ex.what());
			continue;
		}

		// convert from quat to RPY
		Eigen::Quaterniond q_waist(transform_stamped_waist.transform.rotation.w,
							 transform_stamped_waist.transform.rotation.x,
							 transform_stamped_waist.transform.rotation.y,
							 transform_stamped_waist.transform.rotation.z);
		auto waist_RPY = q_waist.toRotationMatrix().eulerAngles(0, 1, 2);
		Eigen::Quaterniond q_shoulder(transform_stamped_shoulder.transform.rotation.w,
									  transform_stamped_shoulder.transform.rotation.x,
							 		  transform_stamped_shoulder.transform.rotation.y,
							 		  transform_stamped_shoulder.transform.rotation.z);
		auto shoulder_RPY = q_shoulder.toRotationMatrix().eulerAngles(0, 1, 2);
		Eigen::Quaterniond q_btw(transform_stamped_between_waist_to_shoulder.transform.rotation.w,
								 transform_stamped_between_waist_to_shoulder.transform.rotation.x,
								 transform_stamped_between_waist_to_shoulder.transform.rotation.y,
								 transform_stamped_between_waist_to_shoulder.transform.rotation.z);
		auto btw_RPY = q_btw.toRotationMatrix().eulerAngles(0, 1, 2);

		std::cout << "[waist rpy] " << waist_RPY(0) << "\t" << waist_RPY(1) << "\t" << waist_RPY(2) << std::endl;
		std::cout << "[shold rpy] " << shoulder_RPY(0) << "\t" << shoulder_RPY(1) << "\t" << shoulder_RPY(2) << std::endl;
		std::cout << "[betwn rpy] " << btw_RPY(0) << "\t" << btw_RPY(1) << "\t" << btw_RPY(2) << "\t" << std::endl;
		//std::cout << btw_RPY(2) << "\t" << std::endl;
	}
	return 0;
}
