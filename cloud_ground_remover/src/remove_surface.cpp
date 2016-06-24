#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
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

class GridDividedPointCloud
{
public:
  GridDividedPointCloud(int grid_x_len, int grid_y_len, pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud_ptr)
  {
    source_cloud_ptr_ = source_cloud_ptr;
    grid_x_len_ = grid_x_len;
    grid_y_len_ = grid_y_len;

    pcl::PointXYZ min_point, max_point;
    pcl::getMinMax3D(*source_cloud_ptr_, min_point, max_point);
    max_cloud_x_ = max_point.x;
    min_cloud_x_ = min_point.x;
    max_cloud_y_ = max_point.y;
    min_cloud_y_ = min_point.y;
    std::cout << "max_cloud_x : " << max_cloud_x_ << std::endl;
    std::cout << "min_cloud_x : " << min_cloud_x_ << std::endl;
    std::cout << "max_cloud_y : " << max_cloud_y_ << std::endl;
    std::cout << "min_cloud_y : " << min_cloud_y_ << std::endl;

    int num_of_grid_ = int((max_cloud_x_ - min_cloud_x_)/grid_x_len_) * int((max_cloud_y_ - min_cloud_y_) / grid_y_len_);

    grid_cloud_vec_.resize(num_of_grid_);

  }
  void divideGrid()
  {
    boost::progress_display show_progress(source_cloud_ptr_->size());
    for (size_t i = 0; i < source_cloud_ptr_->points.size(); ++i) {
      int x_id = int((source_cloud_ptr_->points[i].x - min_cloud_x_)/grid_x_len_);
      int y_id = int((source_cloud_ptr_->points[i].y - min_cloud_y_)/grid_y_len_);
      int store_id = x_id * y_id;
      grid_cloud_vec_[store_id].points.push_back(source_cloud_ptr_->points[i]);
      ++show_progress;
    }
  }
  void estimateSurface()
  {
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold (0.01);
    for (size_t i = 0; i < grid_cloud_vec_.size(); ++i) {
      if (grid_cloud_vec_[i].points.size() < 10) {
        continue;
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

      std::cerr << "Model coefficients: " << coefficients->values[0] << " "
                << coefficients->values[1] << " "
                << coefficients->values[2] << " "
                << coefficients->values[3] << std::endl;

      std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
      pcl::PointCloud<pcl::PointXYZ> inlier_cloud;
      for (size_t j = 0; j < inliers->indices.size (); ++j){
        pcl::PointXYZ point;
        point.x = grid_cloud_vec_[i].points[inliers->indices[j]].x;
        point.y = grid_cloud_vec_[i].points[inliers->indices[j]].y;
        point.z = grid_cloud_vec_[i].points[inliers->indices[j]].z;
        inlier_cloud.push_back(point);
      }
      ground_removed_cloud_ += inlier_cloud;
    }
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr getGroundRemovedCloudPtr()
  {
    return ground_removed_cloud_.makeShared();
  }

  ~GridDividedPointCloud()
  {
  }

private:
  std::vector<pcl::PointCloud<pcl::PointXYZ> > grid_cloud_vec_;
  pcl::PointCloud<pcl::PointXYZ> ground_removed_cloud_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud_ptr_;
  float max_cloud_x_;
  float min_cloud_x_;
  float max_cloud_y_;
  float min_cloud_y_;
  float grid_x_len_;
  float grid_y_len_;
};



int main (int argc, char* argv[])
{
  boost::program_options::options_description desc("Options");
  desc.add_options()
    ("help", "Print help message")
    ("pcd_filename", boost::program_options::value<std::string>()->required(), "pcl pcd filename");

  boost::program_options::variables_map vm;
  try {
    boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
    if( vm.count("help") ){
      std::cout << "Basic Command Line Parameter App" << std::endl;
      std::cerr << desc << std::endl;
      return 0;
    }
    boost::program_options::notify(vm);
  } catch (boost::program_options::error& e) {
    std::cerr << "ERROR: " << e.what() << std::endl << std::endl;
    std::cerr << desc << std::endl;
    return -1;
  }

  std::string pcd_fname = vm["pcd_filename"].as<std::string>();

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile (pcd_fname, *cloud);

  // pcl::VoxelGrid<pcl::PointXYZ> sor;
  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  // sor.setInputCloud(cloud);
  // sor.setLeafSize(0.05, 0.05, 0.05);
  // sor.filter(*cloud_filtered);
  boost::timer segmentation_timer;

    pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
  normal_estimator.setSearchMethod (tree);
  normal_estimator.setInputCloud (cloud);
  normal_estimator.setKSearch (500);
  normal_estimator.compute (*normals);

  pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
  reg.setMinClusterSize (50);
  reg.setMaxClusterSize (1000000);
  reg.setSearchMethod (tree);
  reg.setNumberOfNeighbours (300);
  reg.setInputCloud (cloud);
  //reg.setIndices (indices);
  reg.setInputNormals (normals);
  reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);
  reg.setCurvatureThreshold (0.05);

  std::vector <pcl::PointIndices> clusters;
  reg.extract (clusters);
  std::cout << "Segmentation elapased : " << segmentation_timer.elapsed() << " sec" << std::endl;
  std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
  std::cout << "First cluster has " << clusters[0].indices.size () << " points." << endl;
  std::cout << "These are the indices of the points of the initial" <<
    std::endl << "cloud that belong to the first cluster:" << std::endl;
  // GridDividedPointCloud grid_clouds(1.0, 1.0, cloud_filtered);
  // grid_clouds.divideGrid();
  // grid_clouds.estimateSurface();

  //pcl::visualization::PCLVisualizer viewer;

  //blocks until the cloud is actually rendered
  //pcl::PointCloud<pcl::PointXYZ>::Ptr viewcloudptr = grid_clouds.getGroundRemovedCloudPtr();
  pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
  
  // int v1(0);
  // viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
  // viewer.setBackgroundColor(0, 0, 0, v1); // background color dark
  // viewer.addText("original", 10, 10, "right", v1);
  // viewer.addPointCloud(cloud_filtered,  "original", v1);
  
  // int v2(0);
  // viewer.createViewPort (0.5, 0.0, 1.0, 1.0, v2);
  // viewer.setBackgroundColor(0.1, 0.1, 0.1, v2); // background color light
  // viewer.addText("removed", 10, 10, "left", v2);
  // //viewer.addPointCloud(viewcloudptr, "removed", v2);
  // viewer.addPointCloud(colored_cloud);
  
  // while (!viewer.wasStopped ())
  // {
  //   viewer.spinOnce(100);
  // }
  pcl::visualization::CloudViewer viewer ("Cluster viewer");
  viewer.showCloud(colored_cloud);
  while (!viewer.wasStopped ())
  {
  }
  return 0;
}
