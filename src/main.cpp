
#include <thread>
#include <iostream>
#include <fstream>
#include <chrono>
#include <string>
#include <iomanip>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/se3.hpp>
#include <opencv/cv.hpp>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/subscriber_filter.h>

#include "odom_buf.hpp"

// 结果保存的路径
std::string result_save_path = "/root/catkin_ws/src/ov2slam/catkin_ws/tools/get_camera_and_pose_from_bag/result/";
double trans_th = 1;
double angle_th = 1;

// 里程计数据存储的buf
OdomBuf odom_buf;

// 回调处理函数
void odom_callback(const nav_msgs::Odometry::ConstPtr& odom);
void image_callback(const sensor_msgs::Image::ConstPtr &image_data);
bool can_save(const Sophus::SE3d& pos1, const Sophus::SE3d& pos2);

int main(int argc, char** argv)
{
  // ros init
  ros::init(argc, argv, "get_odom_image");
  ros::NodeHandle nh, nh_private("~");

  // get param
  nh_private.param("result_save_path", result_save_path, result_save_path);
  nh_private.param("trans_th", trans_th, trans_th);
  nh_private.param("angle_th", angle_th, angle_th);

  // register callback1
  std::string odom_topic = "/odom";
  std::string image_topic = "/drivers/realsense/infra1/image_rect_raw0";
  nh_private.param("odom_topic", odom_topic, odom_topic);
  nh_private.param("image_topic", image_topic, image_topic);
  ros::Subscriber sub_odom = nh.subscribe(odom_topic, 1, odom_callback);

  // register callback2 in another thread
  ros::NodeHandle nh_image;
  ros::CallbackQueue callback_queue_image;
  nh_image.setCallbackQueue(&callback_queue_image);
  ros::Subscriber sub_image = nh_image.subscribe(image_topic, 1, image_callback);
  std::thread spinner_thread_image([&callback_queue_image]() {
    ros::SingleThreadedSpinner spinner_a;
    spinner_a.spin(&callback_queue_image);
  });

  // spin
  ros::spin();
  spinner_thread_image.join();
  return 0;
}


// 里程计的回调函数
void odom_callback(const nav_msgs::Odometry::ConstPtr& odom)
{
  double time = odom->header.stamp.toSec();
  Eigen::Quaterniond q(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x,
                       odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);
  Eigen::Vector3d t{odom->pose.pose.position.x, odom->pose.pose.position.y, odom->pose.pose.position.z};
  Sophus::SE3d qt(q, t);
  odom_buf.insert_data(time, qt);
}


// 图像的回调函数
void image_callback(const sensor_msgs::Image::ConstPtr &image_data)
{
  // 启动图像的保存的路径
  static std::ofstream fout(result_save_path + "odom_pose.txt");

  // 获取当前图像的时间
  double cur_image_time = image_data->header.stamp.toSec();

  // 查看是否可以获取对应的里程计的数据
  // 丢弃掉之前可能没有里程计的数据
  static bool is_ready = false;
  if (!is_ready)
  {
    if (!odom_buf.is_ready(cur_image_time)) {
      return;
    } else {
      is_ready = true;
    }
  }

  // 如果此时获取不到里程计的数据,则等一会里程计的数据
  Sophus::SE3d cur_odom_pose;
  if (!odom_buf.get_se3_interpolation(image_data->header.stamp.toSec(), cur_odom_pose))
  {
    // std::this_thread::sleep_for(20ms);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  // 存储上一帧里程计的pose
  static bool get_first_odom = false;
  static Sophus::SE3d last_odom_pose;
  static int cnt = 0;

  // 第一帧图像的数据则可以直接保存
  if (!get_first_odom) {
    // save result
    {
      // 保存对应图像对应的里程计的位姿
      Eigen::Quaterniond q = cur_odom_pose.unit_quaternion();
      Eigen::Vector3d t = cur_odom_pose.translation();
      fout << std::fixed << std::setprecision(5)
           << cur_image_time << " "
           << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << " "
           << t.x() << " " << t.y() << " " << t.z()
           << std::endl;

      // 保存对应的图像的位姿
      cv_bridge::CvImagePtr cv_ptr;
      try
      {
        cv_ptr = cv_bridge::toCvCopy(image_data);
      }
      catch (cv_bridge::Exception &e)
      {
          ROS_ERROR("cv_bridge exception: %s", e.what());
          return;
      }
      cv::Mat image_copy = cv_ptr->image.clone();
      std::string save_file_name = result_save_path + std::to_string(cnt++) + ".png";
      cv::imwrite(save_file_name, image_copy);
    }

    // 保存对应的标志位
    last_odom_pose = cur_odom_pose;
    get_first_odom = true;
  }


  if (can_save(last_odom_pose, cur_odom_pose)) {
    // save result
    {
      // 保存对应图像对应的里程计的位姿
      Eigen::Quaterniond q = cur_odom_pose.unit_quaternion();
      Eigen::Vector3d t = cur_odom_pose.translation();
      fout << std::fixed << std::setprecision(5)
           << cur_image_time << " "
           << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << " "
           << t.x() << " " << t.y() << " " << t.z()
           << std::endl;

      // 保存对应的图像的位姿
      cv_bridge::CvImagePtr cv_ptr;
      try
      {
        cv_ptr = cv_bridge::toCvCopy(image_data);
      }
      catch (cv_bridge::Exception &e)
      {
          ROS_ERROR("cv_bridge exception: %s", e.what());
          return;
      }
      cv::Mat image_copy = cv_ptr->image.clone();
      std::string save_file_name = result_save_path + std::to_string(cnt++) + ".png";
      cv::imwrite(save_file_name, image_copy);
    }

    // 保存对应的标志位
    last_odom_pose = cur_odom_pose;
  }
}

bool can_save(const Sophus::SE3d& pos1, const Sophus::SE3d& pos2)
{
  Sophus::SE3d delta_pose_se3 = pos1.inverse() * pos2;
  Eigen::Matrix<double, 6, 1> delta_log = delta_pose_se3.log();
  return delta_log.head(3).norm() > trans_th || delta_log.tail(3).norm() > angle_th;
}