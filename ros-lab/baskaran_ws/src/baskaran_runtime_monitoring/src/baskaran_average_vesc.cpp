#include<ros/ros.h>
#include<std_msgs/Float64.h>
#include<vesc_msgs/VescStateStamped.h>
#include <vector>

class averaging_vesc
{
public: 
	averaging_vesc();

private:
	ros::NodeHandle nh;
	
	ros::Subscriber vesc_sub;
	ros::Publisher avg_vel_pub;
	void callback_vesc(const vesc_msgs::VescStateStamped::ConstPtr& vesc_msg);
	std::vector<double> vel_vec;
};

averaging_vesc::averaging_vesc()
{
	vesc_sub = nh.subscribe<vesc_msgs::VescStateStamped>("/sensors/core", 100, &averaging_vesc::callback_vesc, this);
	avg_vel_pub = nh.advertise<std_msgs::Float64>("average_velocity", 100);
}

void averaging_vesc::callback_vesc(const vesc_msgs::VescStateStamped::ConstPtr& vesc_msg)
{
	ros::Rate loop_rate(5);
	
	double new_vel = vesc_msg->state.speed;
	vel_vec.push_back(new_vel);

	if (vel_vec.size() > 10)
		vel_vec.erase(vel_vec.begin());
	
	double avg_vel_x = std::accumulate(vel_vec.begin(), vel_vec.end(), (double) 0) / vel_vec.size();

	ROS_INFO("Average velocity: %f", avg_vel_x);
	
	std_msgs::Float64 avg_vel;
	avg_vel.data = avg_vel_x;
	avg_vel_pub.publish(avg_vel);

	loop_rate.sleep();

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "averaging_vesc");
	averaging_vesc k;
	ros::spin();

	return 0;
}