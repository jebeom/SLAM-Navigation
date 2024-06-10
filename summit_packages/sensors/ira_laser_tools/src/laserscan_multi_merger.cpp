#include <ros/ros.h>
#include <string.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include "sensor_msgs/LaserScan.h"
#include "pcl_ros/point_cloud.h"
#include <Eigen/Dense>
#include <dynamic_reconfigure/server.h>
#include <ira_laser_tools/laserscan_multi_mergerConfig.h>

using namespace std;
using namespace pcl;
using namespace laserscan_multi_merger;
// 전체 process -> 라이다 데이터를 subscribe 해서 scan과 cloud 데이터 발행
// 포인트 클라우드 형태로 변환하면 모든 데이터를 일관된 형식으로 통합할 수 있고,  
// 포인트 클라우드 형식은 3차원 공간에서 각 포인트의 정확한 위치를 유지하는 데 더 적합해서
// 레이저 스캔이 놓치기 쉬운 사각지대나 중첩 영역을 보다 효과적으로 처리할 수 있다.
class LaserscanMerger
{
	public:
		LaserscanMerger();
		void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan, std::string topic);
		void pointcloud_to_laserscan(Eigen::MatrixXf points, pcl::PCLPointCloud2 *merged_cloud);
		void reconfigureCallback(laserscan_multi_mergerConfig &config, uint32_t level);

	private:
		ros::NodeHandle node_;
		laser_geometry::LaserProjection projector_;
		tf::TransformListener tfListener_;

		ros::Publisher point_cloud_publisher_;
		ros::Publisher laser_scan_publisher_;
		vector<ros::Subscriber> scan_subscribers;
		vector<bool> clouds_modified;

		vector<pcl::PCLPointCloud2> clouds;
		vector<string> input_topics;

		void laserscan_topic_parser();

		double angle_min;
		double angle_max;
		double angle_increment;
		double time_increment;
		double scan_time;
		double range_min;
		double range_max;

		string destination_frame;
		string cloud_destination_topic;
		string scan_destination_topic;
		string laserscan_topics;
};

void LaserscanMerger::reconfigureCallback(laserscan_multi_mergerConfig &config, uint32_t level)
{
	this->angle_min = config.angle_min;
	this->angle_max = config.angle_max;
	this->angle_increment = config.angle_increment;
	this->time_increment = config.time_increment;
	this->scan_time = config.scan_time;
	this->range_min = config.range_min;
	this->range_max = config.range_max;
}

// front 라이다 데이터(front_laser/scan 토픽)와 rear 라이다 데이터(rear_laser/scan 토픽)을 동적으로 구독하는 함수
void LaserscanMerger::laserscan_topic_parser()
{
	// LaserScan topics to subscribe
	ros::master::V_TopicInfo topics;
	
	istringstream iss(laserscan_topics);
	set<string> tokens;
	copy(istream_iterator<string>(iss), istream_iterator<string>(), inserter<set<string>>(tokens, tokens.begin()));
	vector<string> tmp_input_topics;

	// lasercan_topics 제대로 들어오는지 확인
	ROS_INFO("Set laserscan_topics to: %s", laserscan_topics.c_str());

	while (!tokens.empty())
	{
		ROS_INFO("Waiting for topics ...");
		ros::master::getTopics(topics);
		sleep(1);

		for (int i = 0; i < topics.size(); i++)
		{
			
			if (topics[i].datatype == "sensor_msgs/LaserScan" && tokens.erase(topics[i].name) > 0)
			{
				tmp_input_topics.push_back(topics[i].name);
			}
		}
	}


	sort(tmp_input_topics.begin(), tmp_input_topics.end());
	std::vector<string>::iterator last = std::unique(tmp_input_topics.begin(), tmp_input_topics.end());
	tmp_input_topics.erase(last, tmp_input_topics.end());

	// Do not re-subscribe if the topics are the same
	if ((tmp_input_topics.size() != input_topics.size()) || !equal(tmp_input_topics.begin(), tmp_input_topics.end(), input_topics.begin()))
	{

		// Unsubscribe from previous topics
		for (int i = 0; i < scan_subscribers.size(); i++)
			scan_subscribers[i].shutdown();

		input_topics = tmp_input_topics;

		if (input_topics.size() > 0)
		{
			scan_subscribers.resize(input_topics.size());
			clouds_modified.resize(input_topics.size());
			clouds.resize(input_topics.size());
			ROS_INFO("Subscribing to topics\t%ld", scan_subscribers.size());
			for (int i = 0; i < input_topics.size(); ++i)
			{
				scan_subscribers[i] = node_.subscribe<sensor_msgs::LaserScan>(input_topics[i].c_str(), 1, boost::bind(&LaserscanMerger::scanCallback, this, _1, input_topics[i]));
				clouds_modified[i] = false;
				cout << input_topics[i] << " ";
			}
		}
		else
			ROS_INFO("Not subscribed to any topic.");
	}
}

// 필요한 매개변수를 로드하며, 외부로 데이터를 발행하는 발행자를 설정하는 함수
LaserscanMerger::LaserscanMerger()
{
	ros::NodeHandle nh("~");

	nh.param<std::string>("destination_frame", destination_frame, "cart_frame");
	nh.param<std::string>("cloud_destination_topic", cloud_destination_topic, "/merged_cloud");
	nh.param<std::string>("scan_destination_topic", scan_destination_topic, "/scan_multi");
	nh.param<std::string>("laserscan_topics", laserscan_topics, "");
	nh.param("angle_min", angle_min, -2.36);
	nh.param("angle_max", angle_max, 2.36);
	nh.param("angle_increment", angle_increment, 0.0058);
	nh.param("scan_time", scan_time, 0.0333333);
	nh.param("range_min", range_min, 0.45);
	nh.param("range_max", range_max, 25.0);

	this->laserscan_topic_parser();

	// cloud_destination_topic 제대로 들어오는지 확인
	ROS_INFO("Set cloud_destination_topic to: %s", cloud_destination_topic.c_str());
	// scan_destination_topic 제대로 들어오는지 확인
	ROS_INFO("Set scan_destination_topic to: %s", scan_destination_topic.c_str());

	// 포인트 클라우드(cloud 토픽으로) 발행자 초기화
	//point_cloud_publisher_ = node_.advertise<sensor_msgs::PointCloud2>(cloud_destination_topic.c_str(), 1, false);
	point_cloud_publisher_ = node_.advertise<sensor_msgs::PointCloud2>(cloud_destination_topic.c_str(), 1, false);
		if (!point_cloud_publisher_) {
    		ROS_ERROR("Failed to advertise topic %s", cloud_destination_topic.c_str());
		} 
		else {
    		ROS_INFO("Successfully advertised topic %s", cloud_destination_topic.c_str());
		}
	// 레이저 스캔(scan 토픽으로) 발행자 초기화
	//laser_scan_publisher_ = node_.advertise<sensor_msgs::LaserScan>(scan_destination_topic.c_str(), 1, false);
	laser_scan_publisher_ = node_.advertise<sensor_msgs::LaserScan>(scan_destination_topic.c_str(), 1, false);
		if (!laser_scan_publisher_) {
    		ROS_ERROR("Failed to advertise topic %s", scan_destination_topic.c_str());
		} 
		else {
    		ROS_INFO("Successfully advertised topic %s", scan_destination_topic.c_str());
		}
}

// 라이다로부터 laserScan 데이터를 받아서 포인트 클라우드 형식으로 변환하고, 필요한 변환을 적용한 뒤, 
// 모든 구독된 토픽의 데이터가 도착했을 때 최종적으로 하나의 합쳐진 포인트 클라우드를 생성하는 함수 
void LaserscanMerger::scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan, std::string topic)
{
	sensor_msgs::PointCloud tmpCloud1, tmpCloud2;
	sensor_msgs::PointCloud2 tmpCloud3;

	// refer to http://wiki.ros.org/tf/Tutorials/tf%20and%20Time%20%28C%2B%2B%29
	try
	{
		// Verify that TF knows how to transform from the received scan to the destination scan frame
		tfListener_.waitForTransform(scan->header.frame_id.c_str(), destination_frame.c_str(), scan->header.stamp, ros::Duration(1));
		// 레이저 스캔 데이터를 3D 포인트 클라우드(tmpCloud1)로 변환 
		projector_.transformLaserScanToPointCloud(scan->header.frame_id, *scan, tmpCloud1, tfListener_, laser_geometry::channel_option::Distance);
        // destination frame 즉, base_footprint frame으로 좌표 변환
		tfListener_.transformPointCloud(destination_frame.c_str(), tmpCloud1, tmpCloud2);
	}
	catch (tf::TransformException ex)
	{
		return;
	}

	for (int i = 0; i < input_topics.size(); i++)
	{
		if (topic.compare(input_topics[i]) == 0)
		{
			// 변환된 포인트 클라우드(tmpCloud2)를 PointCloud2 형식(tmpCloud3)으로 다시 변환
			sensor_msgs::convertPointCloudToPointCloud2(tmpCloud2, tmpCloud3);
			// 포인트 클라우드 데이터가 준비되면, 이 데이터들을 하나의 PCLPointCloud2 객체(merged_cloud)로 합칩
			pcl_conversions::toPCL(tmpCloud3, clouds[i]);
			clouds_modified[i] = true;
		}
	}

	// Count how many scans we have
	int totalClouds = 0;
	for (int i = 0; i < clouds_modified.size(); i++)
		if (clouds_modified[i])
			totalClouds++;

	// Go ahead only if all subscribed scans have arrived
	if (totalClouds == clouds_modified.size())
	{
		pcl::PCLPointCloud2 merged_cloud = clouds[0];
		clouds_modified[0] = false;

		for (int i = 1; i < clouds_modified.size(); i++)
		{
			#if PCL_VERSION_COMPARE(>=, 1, 10, 0)
				pcl::concatenate(merged_cloud, clouds[i], merged_cloud);
			#else
				pcl::concatenatePointCloud(merged_cloud, clouds[i], merged_cloud);
			#endif
				clouds_modified[i] = false;
		}

		point_cloud_publisher_.publish(merged_cloud); // cloud 라는 토픽으로 합쳐진 cloud 데이터 발행

		Eigen::MatrixXf points;
		getPointCloudAsEigen(merged_cloud, points);

		pointcloud_to_laserscan(points, &merged_cloud);
	}
}

// 포인트 클라우드 데이터를 레이저 스캔 데이터로 변환
void LaserscanMerger::pointcloud_to_laserscan(Eigen::MatrixXf points, pcl::PCLPointCloud2 *merged_cloud)
{
	sensor_msgs::LaserScanPtr output(new sensor_msgs::LaserScan());
	output->header = pcl_conversions::fromPCL(merged_cloud->header);
	output->angle_min = this->angle_min;
	output->angle_max = this->angle_max;
	output->angle_increment = this->angle_increment;
	output->time_increment = this->time_increment;
	output->scan_time = this->scan_time;
	output->range_min = this->range_min;
	output->range_max = this->range_max;

	uint32_t ranges_size = std::ceil((output->angle_max - output->angle_min) / output->angle_increment);
	output->ranges.assign(ranges_size, output->range_max + 1.0);

	for (int i = 0; i < points.cols(); i++)
	{
		const float &x = points(0, i);
		const float &y = points(1, i);
		const float &z = points(2, i);

		if (std::isnan(x) || std::isnan(y) || std::isnan(z))
		{
			ROS_DEBUG("rejected for nan in point(%f, %f, %f)\n", x, y, z);
			continue;
		}

		double range_sq = pow(y, 2) + pow(x, 2);
		double range_min_sq_ = output->range_min * output->range_min;
		if (range_sq < range_min_sq_)
		{
			ROS_DEBUG("rejected for range %f below minimum value %f. Point: (%f, %f, %f)", range_sq, range_min_sq_, x, y, z);
			continue;
		}

		double angle = atan2(y, x);
		if (angle < output->angle_min || angle > output->angle_max)
		{
			ROS_DEBUG("rejected for angle %f not in range (%f, %f)\n", angle, output->angle_min, output->angle_max);
			continue;
		}
		int index = (angle - output->angle_min) / output->angle_increment;

		if (output->ranges[index] * output->ranges[index] > range_sq)
			output->ranges[index] = sqrt(range_sq);
	}

	laser_scan_publisher_.publish(output); // merge 된 cloud 데이터를 scan으로 변환 후 publish
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "laser_multi_merger");

	LaserscanMerger _laser_merger;

	dynamic_reconfigure::Server<laserscan_multi_mergerConfig> server;
	dynamic_reconfigure::Server<laserscan_multi_mergerConfig>::CallbackType f;

	f = boost::bind(&LaserscanMerger::reconfigureCallback, &_laser_merger, _1, _2);
	server.setCallback(f);

	ros::spin();

	return 0;
}
