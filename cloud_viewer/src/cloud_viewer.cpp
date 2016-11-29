#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <cmath>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <boost/program_options.hpp>
#include <boost/math/special_functions/round.hpp>

#include "utils.h"

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;



int main (int argc, char* argv[])
{
  boost::program_options::options_description desc("Options");
  desc.add_options()
    ("help", "Print help message")
    ("pcd_filename", boost::program_options::value<std::string>()->required(), "pcl pcd filename")
    ("wc", boost::program_options::value<bool>()->default_value(true), "Display point clouds with colors")
    ("ds", boost::program_options::value<bool>()->default_value(false), "Display downsampled point cloud");

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

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::io::loadPCDFile (pcd_fname, *cloud);

  pcl::visualization::CloudViewer viewer("Cloud Viewer");

  if(vm["ds"].as<bool>()){
    downsamplePointClouds(cloud, cloud, 0.05);
  }
  if(vm["wc"].as<bool>()){
    colorizePointClouds(cloud);
  }
  std::cout << "Num of points : " << cloud->points.size() << std::endl;
  viewer.showCloud(cloud);


  while (!viewer.wasStopped ())
  {
  }
  return 0;
}
