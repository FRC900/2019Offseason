#include <iostream>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//#include <std_msgs/Header.h>

//#include <geometry_msgs/Point32.h>
#include <cv_bridge/cv_bridge.h>

#include "teraranger_array/RangeArray.h"

#include "goal_detection/GoalDetection.h"

#include <sstream>

#include "GoalDetector.hpp"

namespace goal_detection
{
	class GoalDetect : public nodelet::Nodelet
	{
		public:

			GoalDetect(void)
			{
			}

			~GoalDetect()
			{
				if (gd_)
					delete gd_;
			}

		protected:
			void onInit() override
			{
				nh_ = getMTPrivateNodeHandle();
				image_transport::ImageTransport it(nh_);
				int sub_rate = 1;
				int pub_rate = 1;
				nh_.getParam("sub_rate", sub_rate);
				nh_.getParam("pub_rate", pub_rate);
				nh_.getParam("batch", batch_);
				nh_.getParam("hFov", hFov_);
				nh_.getParam("camera_angle", camera_angle_);

				bool no_depth = false;
				nh_.getParam("no_depth", no_depth);

				if (!no_depth)
				{
					ROS_INFO("starting goal detection using ZED");
					frame_sub_ = std::make_unique<image_transport::SubscriberFilter>(it, "/zed_goal/left/image_rect_color", sub_rate);
					depth_sub_ = std::make_unique<image_transport::SubscriberFilter>(it, "/zed_goal/depth/depth_registered", sub_rate);
					// ApproximateTime takes a queue size as its constructor argument, hence SyncPolicy(xxx)
					rgbd_sync_ = std::make_unique<message_filters::Synchronizer<RGBDSyncPolicy>>(RGBDSyncPolicy(10), *frame_sub_, *depth_sub_);
					rgbd_sync_->registerCallback(boost::bind(&GoalDetect::callback, this, _1, _2));
				}
				else
				{
					ROS_INFO("starting goal detection using webcam");
					frame_sub_ = std::make_unique<image_transport::SubscriberFilter>(it, "/c920_camera/image_raw", sub_rate);
					terabee_sub_ = std::make_unique<message_filters::Subscriber<teraranger_array::RangeArray>>(nh_, "multiflex_1/ranges_raw", sub_rate);
					rgb_terabee_sync_ = std::make_unique<message_filters::Synchronizer<RGBTerabeeSyncPolicy>>(RGBTerabeeSyncPolicy(10), *frame_sub_, *terabee_sub_);
					rgb_terabee_sync_->registerCallback(boost::bind(&GoalDetect::callback_no_depth, this, _1, _2));
					//rgb_sub_ = std::make_unique<image_transport::Subscriber>(it.subscribe("/c920_camera/image_raw", sub_rate, &GoalDetect::callback_no_depth, this));
					//terabee_sub_ = nh_.subscribe("/multiflex_1/ranges_raw", 1, &GoalDetect::multiflexCB, this);
				}

				// Set up publisher
				pub_ = nh_.advertise<goal_detection::GoalDetection>("goal_detect_msg", pub_rate);
			}
			void callback(const sensor_msgs::ImageConstPtr &frameMsg, const sensor_msgs::ImageConstPtr &depthMsg)
			{
				cv_bridge::CvImageConstPtr cvFrame = cv_bridge::toCvShare(frameMsg, sensor_msgs::image_encodings::BGR8);
				cv_bridge::CvImageConstPtr cvDepth = cv_bridge::toCvShare(depthMsg, sensor_msgs::image_encodings::TYPE_32FC1);

				// Avoid copies by using pointers to RGB and depth info
				// These pointers are either to the original data or to
				// the downsampled data, depending on the down_sample flag
				const cv::Mat *framePtr = &cvFrame->image;
				const cv::Mat *depthPtr = &cvDepth->image;

				// Initialize goal detector object the first time
				// through here. Use the size of the frame
				// grabbed from the ZED messages
				if (gd_ == NULL)
				{
					const cv::Point2f fov(hFov_ * (M_PI / 180.),
										  hFov_ * (M_PI / 180.) * ((double)framePtr->rows / framePtr->cols));
					gd_ = new GoalDetector(fov, framePtr->size(), !batch_);
				}
				//Send current color and depth image to the actual GoalDetector
				gd_->findBoilers(*framePtr, *depthPtr);

				std::vector< GoalFound > gfd = gd_->return_found();
				goal_detection::GoalDetection gd_msg;

				gd_msg.header.seq = frameMsg->header.seq;
				gd_msg.header.stamp = frameMsg->header.stamp;
				std::string frame_id = frameMsg->header.frame_id;
				const size_t idx = frame_id.rfind("_optical_frame");
				if (idx != std::string::npos)
				{
					frame_id.erase(idx);
					frame_id += "_frame";
				}
				gd_msg.header.frame_id = frame_id;
				for(size_t i = 0; i < gfd.size(); i++)
				{
					geometry_msgs::Point32 dummy;
					// Remove _optical_frame from the camera frame ID if present
					dummy.x = gfd[i].pos.y;
					dummy.y = gfd[i].pos.x;
					dummy.z = gfd[i].pos.z;
					gd_msg.location.push_back(dummy);
				}

				gd_msg.valid = gd_->Valid();

				pub_.publish(gd_msg);

				if (!batch_)
				{
					cv::Mat thisFrame(framePtr->clone());
					gd_->drawOnFrame(thisFrame, gd_->getContours(cvFrame->image));
					cv::imshow("Image", thisFrame);
					cv::waitKey(5);
				}

				if (gd_msg.valid == false)
				{
					return;
				}

				//Transform between goal frame and odometry/map.
				static tf2_ros::TransformBroadcaster br;
				for(size_t i = 0; i < gfd.size(); i++)
				{
					geometry_msgs::TransformStamped transformStamped;

					transformStamped.header.stamp = gd_msg.header.stamp;
					transformStamped.header.frame_id = frame_id;
					std::stringstream child_frame;
					child_frame << "goal_";
					child_frame << i;
					transformStamped.child_frame_id = child_frame.str();

					transformStamped.transform.translation.x = gd_msg.location[i].x;
					transformStamped.transform.translation.y = gd_msg.location[i].y;
					transformStamped.transform.translation.z = gd_msg.location[i].z;

					// Can't detect rotation yet, so publish 0 instead
					tf2::Quaternion q;
					q.setRPY(0, 0, 0);

					transformStamped.transform.rotation.x = q.x();
					transformStamped.transform.rotation.y = q.y();
					transformStamped.transform.rotation.z = q.z();
					transformStamped.transform.rotation.w = q.w();

					br.sendTransform(transformStamped);
				}

				/*
				//Transform between a fixed frame and the goal.
				tf2_ros::Buffer tfBuffer;
				tf2_ros::TransformListener tfListener(tfBuffer);

				geometry_msgs::TransformStamped transformStampedOdomCamera;
				try
				{
				transformStampedOdomCamera = tfBuffer.lookupTransform("odom", cvFrame->header.frame_id,
				ros::Time(0));
				}
				catch (tf2::TransformException &ex)
				{
				ROS_WARN("%s", ex.what());
				ros::Duration(1.0).sleep();
				return;
				}

				geometry_msgs::TransformStamped transformStampedOdomGoal;

				tf2::doTransform(transformStamped, transformStampedOdomGoal, transformStampedOdomCamera);

				br.sendTransform(transformStampedOdomGoal);
				*/
			}

			void callback_no_depth(const sensor_msgs::ImageConstPtr &frameMsg, const teraranger_array::RangeArrayConstPtr &msg)
			{
				double min_dist = std::numeric_limits<double>::max();
				for(int i = 0; i < 2; i++)
				{
					const double range = static_cast<double>(msg->ranges[i].range);
					if(!std::isnan(range))
						min_dist = std::min(min_dist, range);
				}
				if (min_dist == std::numeric_limits<double>::max())
					min_dist = -1;
				cv_bridge::CvImageConstPtr cvFrame = cv_bridge::toCvShare(frameMsg, sensor_msgs::image_encodings::BGR8);
				cv::Mat depthMat(cvFrame->image.size(), CV_32FC1, cv::Scalar(min_dist));
				callback(frameMsg, cv_bridge::CvImage(std_msgs::Header(), "32FC1", depthMat).toImageMsg());
			}
			typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> RGBDSyncPolicy;
			typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, teraranger_array::RangeArray> RGBTerabeeSyncPolicy;

			ros::NodeHandle                                                            nh_;
			std::unique_ptr<image_transport::SubscriberFilter>                         frame_sub_;
			std::unique_ptr<image_transport::SubscriberFilter>                         depth_sub_;
			std::unique_ptr<message_filters::Synchronizer<RGBDSyncPolicy>>             rgbd_sync_;

			std::unique_ptr<message_filters::Subscriber<teraranger_array::RangeArray>> terabee_sub_;
			std::unique_ptr<message_filters::Synchronizer<RGBTerabeeSyncPolicy>>       rgb_terabee_sync_;

			ros::Publisher  pub_;
			GoalDetector   *gd_                    = NULL;
			bool            batch_                 = true;
			bool            down_sample_           = false;
			double          hFov_                  = 105.;
			double          camera_angle_          = -25.0;
	};
} // namspace

PLUGINLIB_EXPORT_CLASS(goal_detection::GoalDetect, nodelet::Nodelet)
