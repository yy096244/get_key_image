#include "odom_buf.hpp"

OdomBuf::OdomBuf(double time_length) : time_length_(time_length)
{
}

bool OdomBuf::is_empty() const
{
  std::lock_guard<std::mutex> lock(buf_mu_);
  return bufs_.empty();
}

bool OdomBuf::get_se3_interpolation(double time, Sophus::SE3d& pos)
{
  std::lock_guard<std::mutex> lock(buf_mu_);

  // 如果此时的buf是空的，则返回
  if (bufs_.empty())
  {
    return false;
  }

  // 如果时间在bufs的区间外，则返回
  if (time < bufs_.begin()->first || time > bufs_.rbegin()->first)
  {
    return false;
  }

  // 如果此时查询的时间戳刚好在对应的时间戳上，则返回
  auto it_can_find = bufs_.find(time);
  if (it_can_find != bufs_.end()) {
    // std::cout << it_can_find->second.matrix3x4() << std::endl;

    Eigen::Matrix<double,3,4> mat = it_can_find->second.matrix3x4();

    // TODO : 用mat来进行启动,后面还是要进行更换的
    pos =Sophus::SE3d(mat.block<3,3>(0,0), mat.block<3,1>(0,3));

    // std::cout << "can find2" << std::endl;
    // auto data = it_can_find->second;
    // std::cout << "can find2.1" << std::endl;
    // // pos = Sophus::SE3d(it_can_find->second);
    // pos = data;
    // std::cout << "can find3" << std::endl;
    // pos = it_can_find->second;
    return true;
  }

  // if (bufs_.count(time))
  // {
  // std::cout << "3.1" << std::endl;

  //   try {
  //     // bufs_.at(time);
  //     pos = bufs_.at(time);
  //   } catch(const std::out_of_range &e) {
  //     std::cerr << "Exception at " << e.what() << std::endl;
  //     return false;
  //   }

  //   // pos = bufs_[time];
  //   // pos = bufs_.at(time);
  //   return true;
  // }

  // 找到相邻两帧之间的se3的pos
  // TODO :  优化成二分查找的方式
  auto it1 = bufs_.begin();
  for(; std::next(it1) != bufs_.end(); it1++)
  {
    auto it2 = std::next(it1);
    if (it1->first <= time && it2->first >= time)
    {
      break;
    }
  }

  // 找到了对应的数据，在it1和it2之间来进行插值
  if (std::next(it1) != bufs_.end())
  {
    auto it2 = std::next(it1);
    auto pos1 = it1->second;
    auto pos2 = it2->second;
    pos = se3_interpolation(it1->first, it2->first, time, pos1, pos2);

    // pos = se3_interpolation(it1->first, it2->first, time, it1->second, it2->second);
    return true;
  }

  return false;
}

// 根据时间来进行SE3上的数据的插值，确保time1和time2是不相等的
Sophus::SE3d OdomBuf::se3_interpolation(double time1, double time2, double time3, const Sophus::SE3d& pos1, const Sophus::SE3d& pos2)
{
  // 求pose2相对于pos1的位置，并根据时间差计算出来一个等效的速度
  Eigen::Matrix<double, 6, 1> delta_log = (pos1.inverse() * pos2).log();
  Eigen::Matrix<double, 6, 1> delta_log_v = delta_log / (time2 - time1);
  Eigen::Matrix<double, 6, 1> relative_log = delta_log_v * (time3 - time1);
  Sophus::SE3d relative_pos = Sophus::SE3d::exp(relative_log);
  Sophus::SE3d rtv_pos = pos1 * relative_pos;
  return rtv_pos;
}

bool OdomBuf::insert_data(double time, const Sophus::SE3d& pos)
{
  // std::cout << "--------------------" << std::endl;
  // std::cout << "1" << std::endl;
  std::lock_guard<std::mutex> lock(buf_mu_);
  // bufs_[time] = pos;
  bufs_.insert(std::make_pair(time, pos));

  while (!bufs_.empty() && bufs_.rbegin()->first - bufs_.begin()->first > time_length_)
  {
    bufs_.erase(bufs_.begin());
  }
  // std::cout << "2" << std::endl;
  return true;
}

bool OdomBuf::is_ready(double time)
{
  std::lock_guard<std::mutex> lock(buf_mu_);
  if (bufs_.empty())
  {
    return false;
  }
  return (time >= bufs_.begin()->first);
}

void OdomBuf::set_camera_to_robot_ex(const Sophus::SE3d& camera_to_robot_ex)
{
  camera_to_robot_ex_ = camera_to_robot_ex;
}

Sophus::SE3d OdomBuf::get_camera_to_robot_ex()
{
  return camera_to_robot_ex_;
}