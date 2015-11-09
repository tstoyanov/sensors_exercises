#ifndef OCC_MAP_NODE_HH
#define OCC_MAP_NODE_HH

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <Eigen/Eigen>
#include <cv_bridge/cv_bridge.h>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, nav_msgs::Odometry> ScanOdomSync;

class OccMapNode {

    private:
	int xSize, ySize; //size of the map in number of cells
	double resolution;
	int **grid;

	ros::NodeHandle nh_;
	ros::Publisher map_publisher;
	message_filters::Subscriber<sensor_msgs::LaserScan> *laser_sub_;
       	message_filters::Subscriber<nav_msgs::Odometry> *odom_sub_;
	message_filters::Synchronizer<ScanOdomSync> *sync_po_;

    public:
	OccMapNode(ros::NodeHandle param_nh);
	~OccMapNode();

	void updateMapCallback(const sensor_msgs::LaserScanConstPtr &scan, const nav_msgs::OdometryConstPtr &odometry);
	void publishMap();

};

#endif
