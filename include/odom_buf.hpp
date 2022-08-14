#pragma once

#include <map>
#include <sophus/se3.hpp>
#include <mutex>

class OdomBuf
{
  public:
    OdomBuf(double time_length = 10.0);
    bool is_empty() const;
    bool get_se3_interpolation(double time, Sophus::SE3d& pos);
    static Sophus::SE3d se3_interpolation(double time1, double time2, double time3, const Sophus::SE3d& pos1, const Sophus::SE3d& pos2);
    bool insert_data(double time, const Sophus::SE3d& pos);
    bool is_ready(double time);
    void set_camera_to_robot_ex(const Sophus::SE3d& camera_to_robot_ex);
    Sophus::SE3d get_camera_to_robot_ex();
  private:
    double time_length_;
    std::map<double, Sophus::SE3d> bufs_;
    mutable std::mutex buf_mu_;
    Sophus::SE3d camera_to_robot_ex_;
};