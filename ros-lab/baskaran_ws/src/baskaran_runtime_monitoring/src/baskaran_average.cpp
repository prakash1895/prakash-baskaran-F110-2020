#include<ros/ros.h>
#include<std_msgs/Float64.h>
#include<geometry_msgs/Twist.h>
#include<string>
#include <vector>

class averaging
{
public: 
	averaging();

private:
	ros::NodeHandle nh;
	
	ros::Subscriber cmd_vel_sub;
	ros::Publisher avg_vel_pub;
	void callback_cmdvel(const geometry_msgs::Twist::ConstPtr& cmd_vel);
	std::vector<double> vel_vec;
};

averaging::averaging()
{
	cmd_vel_sub = nh.subscribe<geometry_msgs::Twist>("/turtle1/cmd_vel", 100, &averaging::callback_cmdvel, this);
	avg_vel_pub = nh.advertise<std_msgs::Float64>("average_velocity", 100);
}

void averaging::callback_cmdvel(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
	ros::Rate loop_rate(5);
	
	double new_vel = cmd_vel->linear.x;
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
	ros::init(argc, argv, "averaging");
	averaging k;
	ros::spin();

	return 0;
}