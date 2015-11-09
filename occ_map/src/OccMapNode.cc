#include <OccMapNode.hh>


OccMapNode::OccMapNode(ros::NodeHandle param_nh){ 

    param_nh.param("xSize",xSize,100);
    param_nh.param("ySize",ySize,100);
    param_nh.param("resolution",resolution,0.1);

    //initialize subscibers and callbacks
    laser_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(nh_,"/laserscan",10);
    odom_sub_ = new message_filters::Subscriber<nav_msgs::Odometry>(nh_,"/vmc_navserver/state",10);
    sync_po_ = new message_filters::Synchronizer< ScanOdomSync >(ScanOdomSync(10), *laser_sub_, *odom_sub_);
    sync_po_->registerCallback(boost::bind(&OccMapNode::updateMapCallback, this, _1, _2));

    //advertise map publisher
    map_publisher= nh_.advertise<sensor_msgs::Image>("map",10);

    //TODO:
    //initialize the grid
}

OccMapNode::~OccMapNode(){ 
    delete sync_po_;
    delete laser_sub_;
    delete odom_sub_;
    //TODO: delete grid
}

void OccMapNode::updateMapCallback(const sensor_msgs::LaserScanConstPtr &scan, const nav_msgs::OdometryConstPtr &odo_in){ 
    ROS_INFO("Got a scan and an odometry");
    
    //convert odometry message to Eigen Transform
    Eigen::Quaterniond qd;

    qd.x() = odo_in->pose.pose.orientation.x;
    qd.y() = odo_in->pose.pose.orientation.y;
    qd.z() = odo_in->pose.pose.orientation.z;
    qd.w() = odo_in->pose.pose.orientation.w;

    Eigen::Affine3d this_odom = Eigen::Translation3d (odo_in->pose.pose.position.x,
	    odo_in->pose.pose.position.y,odo_in->pose.pose.position.z) * qd;

    //convert laser scan to a vector of Eigen Vectors
    std::vector<Eigen::Vector3d,  Eigen::aligned_allocator<Eigen::Vector3d> > points;
    for(int i=0; i<scan->ranges.size(); i++) {
	Eigen::Vector3d point;
	double angle = scan->angle_min + i*scan->angle_increment;
	//TODO: note: these formulas assume scanner is not upside down. verify that the transformed scans are correct.
	point(0) = cos(angle)*scan->ranges[i];
	point(1) = sin(angle)*scan->ranges[i];
	point(2) = 0;
	points.push_back(point);
    }

    //TODO: iterate through points and transform them using the odometry message
    //TODO: trace a line through the grid map between current pose and every transformed scan point
    //TODO: update occupancy probability

    this->publishMap();
}

void OccMapNode::publishMap(){ 

    //TODO: copy grid data into image. Note: untested code, debug if needed
    cv_bridge::CvImagePtr image_out (new cv_bridge::CvImage());
    image_out->header.stamp = ros::Time::now();
    image_out->encoding = "8UC1";

    image_out->image = cv::Mat(xSize,ySize,CV_8UC1);
    for(int i=0; i<xSize; i++) {
	for(int j=0; j<ySize; j++) {
	    image_out->image.at<int>(i,j) = grid[i][j];
	}
    }
    map_publisher.publish(image_out->toImageMsg());


}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "occ_map_node");

    ros::NodeHandle param("~");
    OccMapNode t(param);
    ros::spin();

    return 0;
}

