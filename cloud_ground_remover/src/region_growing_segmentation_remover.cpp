#include "region_growing_segmentation_remover.h"

RegionGrowingSegmentationRemoveGround::RegionGrowingSegmentationRemoveGround(pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_cloud_ptr)
{
  source_cloud_ptr_ = source_cloud_ptr;

  this->setGridSize(2.0, 2.0);
  this->setRemoveGroundArea();
  this->setRegionGrowingSegmentationDistThres(0.01);
  this->setRegionGrowingSegmentationMaxIter(100);
}

RegionGrowingSegmentationRemoveGround::~RegionGrowingSegmentationRemoveGround()
{
}

void RegionGrowingSegmentationRemoveGround::removeGround()
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

  boost::progress_display show_progress(grid_cloud_vec_.size());
  std::cout << "grid_cloud_vec_ : " << grid_cloud_vec_.size() << std::endl;
  for (size_t i = 0; i < grid_cloud_vec_.size(); ++i) {
    //std::cout << "i : " << i << std::endl;
    if (grid_cloud_vec_[i].points.size() < 10) {
      ++show_progress;
      continue;
    }
    // colorize
    int r = rand() % 255; int g = rand() % 255; int b = rand() % 255;
    for (size_t j = 0; j < grid_cloud_vec_[i].points.size(); ++j) {
      grid_cloud_vec_[i].points[j].r = r;
      grid_cloud_vec_[i].points[j].g = g;
      grid_cloud_vec_[i].points[j].b = b;
    }

    pcl::search::Search<pcl::PointXYZRGB>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB> > (new pcl::search::KdTree<pcl::PointXYZRGB>);
    pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod (tree);
    normal_estimator.setInputCloud (grid_cloud_vec_[i].makeShared());
    normal_estimator.setKSearch (500);
    normal_estimator.compute (*normals);

    pcl::RegionGrowing<pcl::PointXYZRGB, pcl::Normal> reg;
    reg.setMinClusterSize (1);
    reg.setMaxClusterSize (1000000);
    reg.setSearchMethod (tree);
    reg.setNumberOfNeighbours (300);
    reg.setInputCloud (grid_cloud_vec_[i].makeShared());
    reg.setInputNormals (normals);
    reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);
    reg.setCurvatureThreshold (1.0);
    std::vector <pcl::PointIndices> clusters;
    reg.extract (clusters);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();
    // for (size_t j = 0; j < colored_cloud->points.size(); ++j) {
    //   ground_removed_cloud_.points.push_back(colored_cloud->points[j]);
    // }
    // std::cout << "colored_cloud : " << colored_cloud->points.size() << std::endl;
    ground_removed_cloud_ += *colored_cloud;
    //ground_removed_cloud_ += grid_cloud_vec_[i];
    ++show_progress;
  }
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr RegionGrowingSegmentationRemoveGround::getGroundRemovedCloudPtr()
{
   return ground_removed_cloud_.makeShared();
}

std::vector<PlaneModelParam> RegionGrowingSegmentationRemoveGround::getPlaneCoEfficientsVector()
{
  return plane_model_param_vec_;
}

void RegionGrowingSegmentationRemoveGround::setRemoveGroundArea()
{
  pcl::PointXYZRGB min_point, max_point;
  pcl::getMinMax3D(*source_cloud_ptr_, min_point, max_point);
  max_cloud_x_ = max_point.x;
  min_cloud_x_ = min_point.x;
  max_cloud_y_ = max_point.y;
  min_cloud_y_ = min_point.y;
}

void RegionGrowingSegmentationRemoveGround::setGridSize(int x_length, int y_length)
{
  grid_x_len_ = x_length;
  grid_y_len_ = y_length;
}

void RegionGrowingSegmentationRemoveGround::setRegionGrowingSegmentationMaxIter(int max_num)
{
  ransac_max_iterations_ = max_num;
}

void RegionGrowingSegmentationRemoveGround::setRegionGrowingSegmentationDistThres(float dist_threshold)
{
  ransac_distance_threshold_ = dist_threshold;
}

void RegionGrowingSegmentationRemoveGround::divideGrid()
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
