#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>

#include <algorithm>
#include <cstdlib>
#include <stdlib.h> 
#include <iostream>
#include <chrono>
#include <ctime>    
#include <cmath>  
#include <limits>
#include <vector>
#include <string>


using namespace std::chrono_literals;

class LidarToMmwave 
{

//Creates a function for when messages are to be sent. 
//Messages are sent based on a timed callback.
	public:
		LidarToMmwave(ros::NodeHandle *nh) {
			lidar_to_mmwave_pcl_publisher_ = nh->advertise<sensor_msgs::PointCloud2>("/lidar_to_mmwave_pcl", 10);
        	subscription_ = nh->subscribe("/laser/scan", 10, &LidarToMmwave::lidar_to_mmwave_pcl, this);
        }

		~LidarToMmwave() {
			ROS_INFO("Shutting down lidar_to_mmwave_converter..");
		}


	private:
		ros::Timer timer_;
		ros::Publisher lidar_to_mmwave_pcl_publisher_;
		ros::Subscriber subscription_;

		std::vector<geometry_msgs::Point32> objects_dist;
		std::vector<float> objects_angl;
		void lidar_to_mmwave_pcl(const sensor_msgs::PointCloud::ConstPtr& _msg);
		double range_value(const geometry_msgs::Point32 point);

		// Should make a custom message type since BlockLaser returns a PointCloud without 
		// this info, but i am running low on time
		float horiz_angle_min = -.261;
		float horiz_angle_max = .261;
		float ver_angle_min = -1.04;
		float ver_angle_max = 1.04;
		float range_max = 10;
		float range_min = 0.5;
		float total_vert_angle = ver_angle_max - ver_angle_min;
		float total_horiz_angle = horiz_angle_max - horiz_angle_min;
		int num_vert_rays = 30;
		int num_horiz_rays = 120; 
		float vert_angle_increment = total_vert_angle / num_vert_rays;
		float horiz_angle_increment = total_horiz_angle / num_horiz_rays;
};

double LidarToMmwave::range_value(const geometry_msgs::Point32 point) {
	return sqrt(pow(point.x, 2) + pow(point.y, 2) + pow(point.z, 2));;
}

// converts lidar data to pointcloud of detected objects to simulate sparse mmwave
void LidarToMmwave::lidar_to_mmwave_pcl(const sensor_msgs::PointCloud::ConstPtr& _msg){
	ros::Rate rate(5);
	// get shortest dist index
	int shortestDistIdx = 0;
	std::vector<geometry_msgs::Point32> points = _msg->points;

	for(int i = 0; i < num_vert_rays; i++){
		for(int j = 0; j < num_horiz_rays; j++){ 
			if(this->range_value(points[shortestDistIdx]) > this->range_value(points[i * num_vert_rays + j])){
				shortestDistIdx = i * num_vert_rays + j;
			}
		}
	}

	// group objects in lidar fov to simulate mmwave readings
	std::vector<float> object_center_angls;
	std::vector<geometry_msgs::Point32> object_center_dists;
	int grouped_previous = 0;
	float group_dist = 0;
	float group_angl = 0;
	float curr_value, next_horiz, vert_horiz;
	for(int i = 0; i < num_vert_rays-1; i++) {
		for(int j = 0; j < num_horiz_rays-1; j++) {
		// if point is within range and if it is above the ground
		if(this->range_value(points[i * num_horiz_rays + j]) > range_min && this->range_value(points[i * num_horiz_rays + j]) < range_max && points[i * num_horiz_rays + j].y > .2 && range_max){
			//ROS_INFO("%f", points[i * num_horiz_rays + j].y);
			// Only sample some points to simulate real life
			if ( ((rand() % 100) + 1) < 50){
				//std::cout << "End of object detected, pushing" << std::endl;
				object_center_dists.push_back(points[i * num_horiz_rays + j]);
			}
			}
		}
	}

	//this->objects_angl = object_center_angls;
	this->objects_dist = object_center_dists;
	ROS_INFO("%ld", object_center_dists.size());
	
	// angle compared to straight up from drone
	//float shortestDistIdxAngle = float(shortestDistIdx)*angle_increment - angle_max; 


	// The sensor already has a noise component, so we don't need to add more.

	// convert from "spherical" to cartesian
	/* std::vector<float> pcl_x;
	std::vector<float> pcl_y;
	std::vector<float> pcl_z;

	// set random seed once
	static bool seeded = false;
	if (seeded == false)
	{
		srand (1);
	}
	seeded = true;
	// generate noise
	/ float amplitude = 0.05;
	float noise;
	// convert to xyz (including noise)
	for(int i = 0; i<objects_dist.size(); i++){
		noise = -amplitude + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(amplitude-(-amplitude))));
		pcl_x.push_back( sin(object_center_angls.at(i)) * object_center_dists.at(i) + noise*object_center_dists.at(i));
		noise = -amplitude + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(amplitude-(-amplitude))));
		pcl_y.push_back( sin(			0			) * object_center_dists.at(i) 	+ noise*object_center_dists.at(i));
		noise = -amplitude + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(amplitude-(-amplitude))));
		pcl_z.push_back( cos(object_center_angls.at(i)) * object_center_dists.at(i) + noise*object_center_dists.at(i));
	} */

	// create PointCloud2 msg
	//https://github.com/ros-drivers/velodyne/blob/master/velodyne_laserscan/tests/system.cpp
	auto pcl2_msg = sensor_msgs::PointCloud2();
	pcl2_msg.header = std_msgs::Header();
	pcl2_msg.header.stamp = ros::Time::now();
	std::string frameID = "map";
	pcl2_msg.header.frame_id = frameID;
	pcl2_msg.fields.resize(3);
	pcl2_msg.fields[0].name = 'x';
	pcl2_msg.fields[0].offset = 0;
	pcl2_msg.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
	pcl2_msg.fields[0].count = 1;
	pcl2_msg.fields[1].name = 'y';
	pcl2_msg.fields[1].offset = 4;
	pcl2_msg.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
	pcl2_msg.fields[1].count = 1;
	pcl2_msg.fields[2].name = 'z';
	pcl2_msg.fields[2].offset = 8;
	pcl2_msg.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
	pcl2_msg.fields[2].count = 1;
	const uint32_t POINT_STEP = 12;
	if(objects_dist.size() > 0){
		pcl2_msg.data.resize(std::max((size_t)1, objects_dist.size()) * POINT_STEP, 0x00);
	}
	pcl2_msg.point_step = POINT_STEP; // size (bytes) of 1 point (float32 * dimensions (3 when xyz))
	pcl2_msg.row_step = pcl2_msg.data.size();//pcl2_msg.point_step * pcl2_msg.width; // only 1 row because unordered
	pcl2_msg.height = 1;  // because unordered cloud
	pcl2_msg.width = pcl2_msg.row_step / POINT_STEP; // number of points in cloud
	pcl2_msg.is_dense = false; // there may be invalid points

	// fill PointCloud2 msg data
	if(objects_dist.size() > 0){
		uint8_t *ptr = pcl2_msg.data.data();
		for (size_t i = 0; i < objects_dist.size(); i++)
		{
			*(reinterpret_cast<float*>(ptr + 0)) = objects_dist[i].x;
			*(reinterpret_cast<float*>(ptr + 4)) = objects_dist[i].y;
			*(reinterpret_cast<float*>(ptr + 8)) = objects_dist[i].z;
			ptr += POINT_STEP;
		}
	}
	// publish PointCloud2 msg
	this->lidar_to_mmwave_pcl_publisher_.publish(pcl2_msg);
	rate.sleep();
}

			
int main(int argc, char *argv[])
{
	std::cout << "Starting lidar_to_mmwave_converter node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	ros::init(argc, argv, "lidar_to_mmwave");
	ros::NodeHandle nh;
	LidarToMmwave ltm = LidarToMmwave(&nh);
	ros::spin();
	return 0;
}
