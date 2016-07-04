#include "ransac_remove_ground.h"

RansacRemoveGround::RansacRemoveGround(pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_cloud_ptr)
{
  source_cloud_ptr_ = source_cloud_ptr;

  this->setGridSize(2.0, 2.0);
  this->setRemoveGroundArea();
  this->setRansacDistThres(0.01);
  this->setRansacMaxIter(100);
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
  for (size_t i = 0; i < grid_cloud_vec_.size(); ++i) {
    if (grid_cloud_vec_[i].points.size() < 10) {
      continue;
    }
    int r = rand() % 255;
    int g = rand() % 255;
    int b = rand() % 255;
    for (size_t j = 0; j < grid_cloud_vec_[i].points.size(); ++j) {
      grid_cloud_vec_[i].points[j].r = r;
      grid_cloud_vec_[i].points[j].g = g;
      grid_cloud_vec_[i].points[j].b = b;
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

    // std::cerr << "Model coefficients: " << coefficients->values[0] << " "
    //           << coefficients->values[1] << " "
    //           << coefficients->values[2] << " "
    //           << coefficients->values[3] << std::endl;

    // std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
    pcl::PointCloud<pcl::PointXYZRGB> inlier_cloud;
    for (size_t j = 0; j < inliers->indices.size (); ++j){
      pcl::PointXYZRGB point;
      point = grid_cloud_vec_[i].points[inliers->indices[j]];
      inlier_cloud.push_back(point);
    }
    ground_removed_cloud_ += inlier_cloud;
    //ground_removed_cloud_ += grid_cloud_vec_[i];
  }
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr RansacRemoveGround::getGroundRemovedCloudPtr()
{
   return ground_removed_cloud_.makeShared();
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

void RansacRemoveGround::setGridSize(int x_length, int y_length)
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
  int num_of_grid = int((max_cloud_x_ - min_cloud_x_)/grid_x_len_) * int((max_cloud_y_ - min_cloud_y_) / grid_y_len_);

  grid_cloud_vec_.resize(num_of_grid);

  boost::progress_display show_progress(source_cloud_ptr_->size());
  for (size_t i = 0; i < source_cloud_ptr_->points.size(); ++i) {
    int x_id = int((source_cloud_ptr_->points[i].x - min_cloud_x_)/grid_x_len_);
    int y_id = int((source_cloud_ptr_->points[i].y - min_cloud_y_)/grid_y_len_);
    int store_id = x_id * y_id;
    grid_cloud_vec_[store_id].points.push_back(source_cloud_ptr_->points[i]);
    ++show_progress;
  }
}
