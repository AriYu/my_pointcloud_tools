#include "ransac_remove_ground.h"
#include <pcl/filters/extract_indices.h>

class UnitVector
{
public:
  UnitVector()
  {
    init();
  }
  UnitVector(double x, double y, double z)
  {
    init();
    val_[0] = x;
    val_[1] = y;
    val_[2] = z;
    double sum = 0;
    for (size_t i = 0; i < val_.size(); ++i) {
      sum += pow(val_[i], 2.0);
    }
    sum = sqrt(sum);
    for (size_t i = 0; i < val_.size(); ++i) {
      val_[i] = val_[i] / sum;
    }
  }
  void init()
  {
    val_.resize(3);
  }
  std::vector<double> val_;

};


RansacRemoveGround::RansacRemoveGround(pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_cloud_ptr)
{
  source_cloud_ptr_ = source_cloud_ptr;

  this->setGridSize(1.5, 1.5);
  this->setRemoveGroundArea();
  this->setRansacDistThres(0.15);
  this->setRansacMaxIter(100000);
}

RansacRemoveGround::~RansacRemoveGround()
{
}

void RansacRemoveGround::removeGround()
{
  // Preparetion
  this->divideGrid();

  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations(ransac_max_iterations_);
  seg.setDistanceThreshold (ransac_distance_threshold_);
  std::cout << "grid_cloud_vec_ : " << grid_cloud_vec_.size() << std::endl;
  boost::progress_display show_progress(grid_cloud_vec_.size());
  for (size_t i = 0; i < grid_cloud_vec_.size(); ++i) {
    if (grid_cloud_vec_[i].points.size() < 5) {
      ++show_progress;
      continue;
    }
    int r = rand()/255; int g = rand()/255; int b = rand()/255;
    for (size_t j = 0; j < grid_cloud_vec_[i].points.size(); ++j) {
      grid_cloud_vec_[i].points[j].r = r;//237;
      grid_cloud_vec_[i].points[j].g = g;//17;
      grid_cloud_vec_[i].points[j].b = b;//193;
    }
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    seg.setInputCloud(grid_cloud_vec_[i].makeShared());
    seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0)
    {
      PCL_ERROR ("Could not estimate a planar model for the given dataset.");
      return;
    }
    UnitVector plane_vector(coefficients->values[0],
                            coefficients->values[1],
                            coefficients->values[2]);
    UnitVector origin_vector(0, 0, 1);
    double cos_dist = 0;
    for (size_t j = 0; j < 3; ++j) {
      cos_dist += (plane_vector.val_[j] * origin_vector.val_[j]);
    }
    //std::cout << "cos dist : " << cos_dist << std::endl;
    if (fabs(cos_dist) > 0.8) {
      PlaneModelParam plane_model_param;
      plane_model_param.coefficients_ = *coefficients;
      pcl::PointXYZ average_pos(0, 0, 0);
      for (size_t j = 0; j < inliers->indices.size (); ++j){
        pcl::PointXYZRGB point;
        point = grid_cloud_vec_[i].points[inliers->indices[j]];
        average_pos.x += point.x; average_pos.y += point.y;
      }
      plane_model_param.position_.x = average_pos.x / (double)inliers->indices.size();
      plane_model_param.position_.y = average_pos.y / (double)inliers->indices.size();
      plane_model_param.position_.z = 0;
      plane_coefficient_vec_.push_back(plane_model_param);
      pcl::ExtractIndices<pcl::PointXYZRGB> extract;
      extract.setInputCloud(grid_cloud_vec_[i].makeShared());
      extract.setIndices(inliers);
      extract.setNegative(true);
      pcl::PointCloud<pcl::PointXYZRGB> ground_removed_cloud;
      extract.filter(ground_removed_cloud);
      ground_removed_cloud_ += ground_removed_cloud;
    }else
    {
      ground_removed_cloud_ += grid_cloud_vec_[i];
    }
    //ground_removed_cloud_ += grid_cloud_vec_[i];
    ++show_progress;
  }
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr RansacRemoveGround::getGroundRemovedCloudPtr()
{
   return ground_removed_cloud_.makeShared();
}

std::vector<PlaneModelParam> RansacRemoveGround::getPlaneCoEfficientsVector()
{
  return plane_coefficient_vec_;
}

void RansacRemoveGround::setRemoveGroundArea()
{
  pcl::PointXYZRGB min_point, max_point;
  pcl::getMinMax3D(*source_cloud_ptr_, min_point, max_point);
  max_cloud_x_ = max_point.x;
  min_cloud_x_ = min_point.x;
  max_cloud_y_ = max_point.y;
  min_cloud_y_ = min_point.y;
}

void RansacRemoveGround::setGridSize(double x_length, double y_length)
{
  grid_x_len_ = x_length;
  grid_y_len_ = y_length;
}

void RansacRemoveGround::setRansacMaxIter(int max_num)
{
  ransac_max_iterations_ = max_num;
}

void RansacRemoveGround::setRansacDistThres(float dist_threshold)
{
  ransac_distance_threshold_ = dist_threshold;
}

void RansacRemoveGround::divideGrid()
{
  // int num_of_grid = int((max_cloud_x_ - min_cloud_x_)/grid_x_len_) * int((max_cloud_y_ - min_cloud_y_) / grid_y_len_);
  int resolution_x = int((max_cloud_x_ - min_cloud_x_) / grid_x_len_);
  int resolution_y = int((max_cloud_y_ - min_cloud_y_) / grid_y_len_);
  int num_of_grid = (resolution_x+1) * (resolution_y+1);
  grid_cloud_vec_.resize(num_of_grid);
  std::cout << "res x : " << resolution_x << ", res y : " << resolution_y << std::endl;
  std::cout << "num of points : " << source_cloud_ptr_->points.size() << std::endl;
  std::cout << "num of grid   : " << num_of_grid << std::endl;
  //exit(1);
  boost::progress_display show_progress(source_cloud_ptr_->points.size());
  for (size_t i = 0; i < source_cloud_ptr_->points.size(); ++i) {
    int x_id = int((source_cloud_ptr_->points[i].x - min_cloud_x_)/grid_x_len_);
    int y_id = int((source_cloud_ptr_->points[i].y - min_cloud_y_)/grid_y_len_);
    int store_id = x_id + (resolution_x * y_id);
    if(store_id > num_of_grid)
    {
      std::cout << "(" << x_id << "," << y_id << ") : " << store_id << std::endl;
    }
    //std::cout << "(" << x_id << "," << y_id << ") : " << store_id << std::endl;
    //std::cout << "store_id : " << store_id << std::endl;
    grid_cloud_vec_[store_id].points.push_back(source_cloud_ptr_->points[i]);
    ++show_progress;
  }
  std::cout << "divideGrid is done." << std::endl;
}
