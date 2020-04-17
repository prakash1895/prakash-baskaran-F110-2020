#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>

#include "baskaran_gap_finding/Gap.h"
#include "baskaran_gap_finding/gaps.h"

#include <vector>
#include <cmath>
#include <numeric>

#define THRESHOLD 0.1  //10 cm is the threshold b/w 2 consecutive prev_measurement
#define MIN_WIDTH 0.5  //50 cm minimum permissible gap width for the car to penetrate.
#define MIN_DEPTH 1.0 //100 cm minimum permissible depth for the car to penetrate.
#define SCAN_THRESHOLD 1 // number of scans to average the gap center
#define METRIC 0 // 0 --> best gap based on max depth; 1 --> based gap based on max width

class find_gap
{
public:
	find_gap();

private:
	ros::NodeHandle nh;

	ros::Subscriber laser_scan_sub;
	ros::Publisher lidar_gaps_pub;
	ros::Publisher gap_center_pub;
	ros::Publisher gap_pose_pub;

	void callback_scan(const sensor_msgs::LaserScan& scan_msg);
	double calculate_width(double dist_l, double dist_r, double theta);
	geometry_msgs::Vector3 calculate_best_gap(baskaran_gap_finding::gaps gap_array);

	double angle_min;
	double angle_increment;
	double radius;
	double angle;

	baskaran_gap_finding::Gap best_gap;
	geometry_msgs::Vector3 best_gap_pos;

	std::vector<double> xpos_vec;
	std::vector<double> ypos_vec;
};

find_gap::find_gap()
{
	laser_scan_sub = nh.subscribe("/scan", 25, &find_gap::callback_scan, this);
	lidar_gaps_pub = nh.advertise<baskaran_gap_finding::gaps>("lidar_gaps", 25);
	gap_center_pub = nh.advertise<geometry_msgs::Vector3>("gap_center", 25);
}

double find_gap::calculate_width(double dist_l, double dist_r, double theta)
{
	double width;
	width = sqrt(dist_l*dist_l + dist_r*dist_r - 2*dist_l*dist_r*cos(theta));
	return width;
}

geometry_msgs::Vector3 find_gap::calculate_best_gap(baskaran_gap_finding::gaps gap_array)
{
	if (METRIC == 0)
		best_gap = gap_array.gapArray[gap_array.max_depth_idx];

	else if (METRIC == 1)
		best_gap = gap_array.gapArray[gap_array.max_width_idx];
	
	int mid_pt = best_gap.num_points/2;
	radius = best_gap.ranges[mid_pt];
	angle = angle_min + (best_gap.start_idx + mid_pt)*angle_increment;
	double xpos = radius*cos(angle);
	double ypos = radius*sin(angle);

	xpos_vec.push_back(xpos);
	ypos_vec.push_back(ypos);

	if (xpos_vec.size() > SCAN_THRESHOLD)
		xpos_vec.erase(xpos_vec.begin());

	if (ypos_vec.size() > SCAN_THRESHOLD)
		ypos_vec.erase(ypos_vec.begin());
	
	double avg_xpos = std::accumulate(xpos_vec.begin(), xpos_vec.end(), (double) 0) / xpos_vec.size();
	double avg_ypos = std::accumulate(ypos_vec.begin(), ypos_vec.end(), (double) 0) / ypos_vec.size();

	best_gap_pos.x = avg_xpos;
	best_gap_pos.y = avg_ypos;
	best_gap_pos.z = 0.00;

	return best_gap_pos;
}

void find_gap::callback_scan(const sensor_msgs::LaserScan& scan_msg)
{
	angle_min = scan_msg.angle_min;
	angle_increment = scan_msg.angle_increment;
	double theta = 0.00;

	baskaran_gap_finding::gaps gap_array;
	gap_array.max_depth = 0.0;
	gap_array.max_depth_idx = 0;
	gap_array.max_width = 0.0;
	gap_array.max_width_idx = 0;
	
	baskaran_gap_finding::Gap cur_gap;
	cur_gap.dist_l = scan_msg.ranges[0];
	cur_gap.min_depth = scan_msg.ranges[0];
	cur_gap.width = 0.0;
	cur_gap.start_idx = 0;
	cur_gap.num_points = 0;

	cur_gap.ranges.push_back(scan_msg.ranges[0]);
	cur_gap.num_points++;
	double prev_measurement = scan_msg.ranges[0];

	for(int i=1; i<scan_msg.ranges.size(); i++)
	{
		if (std::abs(scan_msg.ranges[i] - prev_measurement) <= THRESHOLD)
		{
			cur_gap.ranges.push_back(scan_msg.ranges[i]);
			cur_gap.num_points++;
			prev_measurement = scan_msg.ranges[i];

			if (scan_msg.ranges[i] < cur_gap.min_depth)
			{
				cur_gap.min_depth = scan_msg.ranges[i];
			}
		}

		else
		{	
			cur_gap.dist_r = prev_measurement;
			theta = angle_increment*cur_gap.num_points;
			cur_gap.width = calculate_width(cur_gap.dist_l, cur_gap.dist_r, theta);

			if (cur_gap.min_depth >= MIN_DEPTH && cur_gap.width >= MIN_WIDTH)
			{
				gap_array.gapArray.push_back(cur_gap);
				if (cur_gap.min_depth >= gap_array.max_depth)
				{
					gap_array.max_depth = cur_gap.min_depth;
					gap_array.max_depth_idx = gap_array.gapArray.size() - 1;
				}

				if (cur_gap.width >= gap_array.max_width)
				{
					gap_array.max_width = cur_gap.width;
					gap_array.max_width_idx = gap_array.gapArray.size() - 1;
				}
			}
			
			// re-initialize the values again
			cur_gap.dist_l = scan_msg.ranges[i];
			cur_gap.min_depth = scan_msg.ranges[i];
			cur_gap.width = 0.0;
			cur_gap.start_idx = i;
			cur_gap.num_points = 0;
			cur_gap.ranges.clear();

			cur_gap.ranges.push_back(scan_msg.ranges[i]);
			cur_gap.num_points++;
			prev_measurement = scan_msg.ranges[i];
		}
	}
	
	// Adding the last gap point
	cur_gap.dist_r = prev_measurement;
	theta = angle_increment*cur_gap.num_points;
	cur_gap.width = calculate_width(cur_gap.dist_l, cur_gap.dist_r, theta);
	if (cur_gap.min_depth >= MIN_DEPTH && cur_gap.width >= MIN_WIDTH)
	{
		gap_array.gapArray.push_back(cur_gap);
		if (cur_gap.min_depth > gap_array.max_depth)
		{
			gap_array.max_depth = cur_gap.min_depth;
			gap_array.max_depth_idx = gap_array.gapArray.size() - 1;
		}

		if (cur_gap.width >= gap_array.max_width)
		{
			gap_array.max_width = cur_gap.width;
			gap_array.max_width_idx = gap_array.gapArray.size() - 1;
		}
	}

	std::cerr<<"\n Gap Array size: "<<gap_array.gapArray.size()<<std::endl;
	std::cerr<<"Best Gap idx (width,depth): "<<gap_array.max_width_idx<<" "<< gap_array.max_depth_idx<<std::endl;
	for (int j=0; j< gap_array.gapArray.size(); j++)
	{
		cur_gap = gap_array.gapArray[j];
		std::cerr<<"Gap no. "<<j<<"  Start idx: "<<cur_gap.start_idx<<" Num points: "<<cur_gap.num_points<<" gap width: "<<cur_gap.width<<" gap depth: "<<cur_gap.min_depth<<std::endl;
		std::cerr<<std::endl;
	}
	
	if (gap_array.gapArray.size() != 0)
		best_gap_pos = calculate_best_gap(gap_array);
	
	gap_array.header.stamp = ros::Time::now();
	lidar_gaps_pub.publish(gap_array);
	gap_center_pub.publish(best_gap_pos);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "find_gap");
	find_gap k;
	ros::spin();

	return 0;
}