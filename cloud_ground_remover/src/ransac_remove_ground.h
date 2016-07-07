#ifndef RANSAC_REMOVE_GROUND_H
#define RANSAC_REMOVE_GROUND_H

#include <iostream>
#include "remove_ground_base.h"
#include <pcl/common/common.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <boost/program_options.hpp>
#include <boost/progress.hpp>
#include <boost/timer.hpp>


class RansacRemoveGround: public RemoveGroundBase
{
public:
  RansacRemoveGround(pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_cloud_ptr);
  virtual ~RansacRemoveGround();
  void setGridSize(int x_length, int y_length);
  void setRansacMaxIter(int max_num);
  void setRansacDistThres(float dist_threshold);
  void removeGround();
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr getGroundRemovedCloudPtr();
  std::vector<PlaneModelParam> getPlaneCoEfficientsVector();
private:
  void setRemoveGroundArea();
  void divideGrid();
  int grid_x_len_;
  int grid_y_len_;
  std::vector<pcl::PointCloud<pcl::PointXYZRGB> > grid_cloud_vec_;
  std::vector<PlaneModelParam> plane_coefficient_vec_;
  pcl::PointCloud<pcl::PointXYZRGB> ground_removed_cloud_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_cloud_ptr_;
  float max_cloud_x_;
  float min_cloud_x_;
  float max_cloud_y_;
  float min_cloud_y_;
  int ransac_max_iterations_;
  float ransac_distance_threshold_;
};


#endif /* RANSAC_REMOVE_GROUND_H */
