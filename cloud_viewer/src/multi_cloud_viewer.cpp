#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>

#include <iostream>
#include <string>

#include <vector>

#include "utils.h"

typedef pcl::PointXYZRGBA PointType;
typedef pcl::PointCloud<PointType> Cloud;
typedef Cloud::ConstPtr CloudConstPtr;
typedef Cloud::Ptr CloudPtr;
typedef std::pair<std::string, CloudPtr> CloudPair;
typedef std::vector<CloudPair> CloudVector;

int main (int argc, char **argv)
{
  std::vector<int> pcd_indices;
  pcd_indices = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr sum_clouds (new pcl::PointCloud<pcl::PointXYZRGBA>);
  
  CloudVector clouds;
  for (size_t i = 0; i < pcd_indices.size (); i++)
  {
    CloudPtr pc (new Cloud);
    pcl::io::loadPCDFile (argv[pcd_indices[i]], *pc);
    *sum_clouds += *pc;
    std::cout << "loading file: " << argv[pcd_indices[i]] << " size: " << pc->size () << std::endl;
  }

  pcl::visualization::CloudViewer viewer("Cloud Viewer");

  bool is_colorize(true);
  //pcl::console::parse_argument(argc, argv, "--ws", is_colorize);
  if(is_colorize){
    std::cout << "colorize option is enable." << std::endl;
    colorizePointClouds(sum_clouds);
  }
  bool is_downsample(false);
  pcl::console::parse_argument(argc, argv, "--ds", is_downsample);
  if(is_downsample){
    downsamplePointClouds(sum_clouds, sum_clouds, 0.5);
  }

  
  viewer.showCloud(sum_clouds);

  while (!viewer.wasStopped ())
  {
  }
  return 0;
}
