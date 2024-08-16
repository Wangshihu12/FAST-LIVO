// This file is part of SVO - Semi-direct Visual Odometry.
//
// Copyright (C) 2014 Christian Forster <forster at ifi dot uzh dot ch>
// (Robotics and Perception Group, University of Zurich, Switzerland).
//
// SVO is free software: you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the Free Software
// Foundation, either version 3 of the License, or any later version.
//
// SVO is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include <stdexcept>
#include <frame.h>
#include <feature.h>
#include <point.h>
#include <boost/bind.hpp>
#include <vikit/math_utils.h>
#include <vikit/vision.h>
#include <vikit/performance_monitor.h>
// #include <fast/fast.h>

namespace lidar_selection {

int Frame::frame_counter_ = 0; 

Frame::Frame(vk::AbstractCamera* cam, const cv::Mat& img) :
    id_(frame_counter_++),  // frame ID
    cam_(cam), 
    key_pts_(5),            // 关键点
    is_keyframe_(false)     // 是否是关键帧
{
  initFrame(img);
}

Frame::~Frame()
{
  std::for_each(fts_.begin(), fts_.end(), [&](FeaturePtr i){i.reset();});
}

void Frame::initFrame(const cv::Mat& img)
{
  // check image 检查图像
  if(img.empty() || img.type() != CV_8UC1 || img.cols != cam_->width() || img.rows != cam_->height())
    throw std::runtime_error("Frame: provided image has not the same size as the camera model or image is not grayscale");

  // 设置关键点为空
  std::for_each(key_pts_.begin(), key_pts_.end(), [&](FeaturePtr ftr){ ftr=nullptr; });
  
  ImgPyr ().swap(img_pyr_);   // 清空图像金字塔 img_pyr_
  img_pyr_.push_back(img);    // 将当前图像放在图像金字塔最顶层
  // Build Image Pyramid
  // frame_utils::createImgPyramid(img, max(Config::nPyrLevels(), Config::kltMaxLevel()+1), img_pyr_);
  // frame_utils::createImgPyramid(img, 5, img_pyr_); 
}

void Frame::setKeyframe()
{
  // 设置关键帧
  is_keyframe_ = true;
  setKeyPoints();
}

void Frame::addFeature(FeaturePtr ftr)
{
  // 将特征添加到当前帧 fts_
  fts_.push_back(ftr);
}

// 重新设置 特征点
void Frame::setKeyPoints()
{
  // 遍历 key_pts_ 的前5个，如果点为空，则设置为空
  for(size_t i = 0; i < 5; ++i)
    if(key_pts_[i] != nullptr)
      if(key_pts_[i]->point == nullptr)
        key_pts_[i] = nullptr;

  // 遍历所有特征，如果特征里的点不为空，则调用 checkKeyPoints 来处理这些特征
  std::for_each(fts_.begin(), fts_.end(), [&](FeaturePtr ftr){ if(ftr->point != nullptr) checkKeyPoints(ftr); });
}

// 传入一个特征，更新 key_pts_ 中的关键点
void Frame::checkKeyPoints(FeaturePtr ftr)
{
  // 计算图像中心
  const int cu = cam_->width()/2;
  const int cv = cam_->height()/2;

  // 如果 key_pts_[0] 为 nullptr，直接将 ftr 赋值给 key_pts_[0]。
  // 否则，比较当前特征点 ftr 到图像中心的距离与 key_pts_[0] 的距离，如果 ftr 更接近中心，则替换 key_pts_[0]
  if(key_pts_[0] == nullptr)
    key_pts_[0] = ftr;

  else if(std::max(std::fabs(ftr->px[0]-cu), std::fabs(ftr->px[1]-cv))
        < std::max(std::fabs(key_pts_[0]->px[0]-cu), std::fabs(key_pts_[0]->px[1]-cv)))
    key_pts_[0] = ftr;

  // 根据特征点的坐标 (ftr->px[0], ftr->px[1]) 在图像中心 (cu, cv) 的位置，将其分配到五个关键点之一。
  // 对于每个象限，检查是否需要更新对应的关键点
  if(ftr->px[0] >= cu && ftr->px[1] >= cv)
  {
    if(key_pts_[1] == nullptr)
      key_pts_[1] = ftr;
    else if((ftr->px[0]-cu) * (ftr->px[1]-cv)
          > (key_pts_[1]->px[0]-cu) * (key_pts_[1]->px[1]-cv))
      key_pts_[1] = ftr;
  }

  if(ftr->px[0] >= cu && ftr->px[1] < cv)
  {
    if(key_pts_[2] == nullptr)
      key_pts_[2] = ftr;
    // else if((ftr->px[0]-cu) * (ftr->px[1]-cv)
    else if((ftr->px[0]-cu) * (cv-ftr->px[1])
          // > (key_pts_[2]->px[0]-cu) * (key_pts_[2]->px[1]-cv))
          > (key_pts_[2]->px[0]-cu) * (cv-key_pts_[2]->px[1]))
      key_pts_[2] = ftr;
  }

  if(ftr->px[0] < cu && ftr->px[1] < cv)
  {
    if(key_pts_[3] == nullptr)
      key_pts_[3] = ftr;
    else if((ftr->px[0]-cu) * (ftr->px[1]-cv)
          > (key_pts_[3]->px[0]-cu) * (key_pts_[3]->px[1]-cv))
      key_pts_[3] = ftr;
  }

  if(ftr->px[0] < cu && ftr->px[1] >= cv)  
  // if(ftr->px[0] < cv && ftr->px[1] >= cv)
  {
    if(key_pts_[4] == nullptr)
      key_pts_[4] = ftr;

    else if(cu-(ftr->px[0]) * (ftr->px[1]-cv) 
          > (cu-key_pts_[4]->px[0]) * (key_pts_[4]->px[1]-cv))      
      key_pts_[4] = ftr;
  }
}

void Frame::removeKeyPoint(FeaturePtr ftr)
{
  bool found = false;
  // 遍历关键点里的所有特征，如果与传入的 特征相同，则从 kety_pts_ 里删除该特征
  std::for_each(key_pts_.begin(), key_pts_.end(), [&](FeaturePtr& i){
    if(i == ftr) {
      i = nullptr;
      found = true;
    }
  });
  // 重新设置关键点
  if(found)
    setKeyPoints();
}

// 检查是否在图像内
bool Frame::isVisible(const Vector3d& xyz_w) const
{
  // 转换到相机坐标系
  Vector3d xyz_f = T_f_w_*xyz_w;

  if(xyz_f.z() < 0.0)
    return false; // point is behind the camera

  // 转换成像素坐标
  Vector2d px = f2c(xyz_f);

  // 检查是否在图像内
  if(px[0] >= 0.0 && px[1] >= 0.0 && px[0] < cam_->width() && px[1] < cam_->height())
    return true;
  return false;
}

/// Utility functions for the Frame class
namespace frame_utils {

// 创建图像金字塔
void createImgPyramid(const cv::Mat& img_level_0, int n_levels, ImgPyr& pyr)
{
  pyr.resize(n_levels);   // 根据层数初始化图像金字塔
  pyr[0] = img_level_0;

  for(int i=1; i<n_levels; ++i)
  {
    // 创建一半大小的图像，然后降采样
    pyr[i] = cv::Mat(pyr[i-1].rows/2, pyr[i-1].cols/2, CV_8U);
    vk::halfSample(pyr[i-1], pyr[i]);
  }
}


bool getSceneDepth(const Frame& frame, double& depth_mean, double& depth_min)
{
  // 分配内存，存储所有特征的深度值
  vector<double> depth_vec;
  depth_vec.reserve(frame.fts_.size());
  // 初始化 depth_min 为一个很大的值
  depth_min = std::numeric_limits<double>::max(); 

  // 遍历所有特征值，转换到相机坐标系，并存储深度值
  for(auto it=frame.fts_.begin(), ite=frame.fts_.end(); it!=ite; ++it)
  {
    if((*it)->point != nullptr) 
    {
      const double z = frame.w2f((*it)->point->pos_).z();
      depth_vec.push_back(z);
      depth_min = fmin(z, depth_min);
    }
  }

  if(depth_vec.empty())
  {
    cout<<"Cannot set scene depth. Frame has no point-observations!"<<endl;
    return false;
  }

  // 计算深度值的中位数
  depth_mean = vk::getMedian(depth_vec);
  return true;
}

} // namespace frame_utils
} // namespace lidar_selection
