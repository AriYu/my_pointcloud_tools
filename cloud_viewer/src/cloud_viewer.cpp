#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <boost/program_options.hpp>
#include <boost/math/special_functions/round.hpp>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;


void colorizePointClouds(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud)
{
  double min, max;
  min = cloud->points[0].z;
  max = cloud->points[0].z;
  for (PointCloudT::iterator cloud_it = cloud->begin(); cloud_it != cloud->end(); ++cloud_it) {
    if(min > cloud_it->z){
      min = cloud_it->z;
    }
    if(max < cloud_it->z){
      max = cloud_it->z;
    }
  }
  double lut_scale = 255.0 / (max - min); // max is 255, min is 0
  if(min == max){
    lut_scale = 1.0;
  }
  for (PointCloudT::iterator cloud_it = cloud->begin(); cloud_it != cloud->end(); ++cloud_it) {
    int value;
    value = boost::math::iround( (cloud_it->z -min) * lut_scale);
    // Blue(=min) -> Red(=max)
    cloud_it->r = value;
    cloud_it->g = 0;
    cloud_it->b = 255 - value;
  }
}

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

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::io::loadPCDFile (pcd_fname, *cloud);

  pcl::visualization::CloudViewer viewer("Cloud Viewer");


  colorizePointClouds(cloud);
  viewer.showCloud(cloud);


  while (!viewer.wasStopped ())
  {
  }
  return 0;
}
