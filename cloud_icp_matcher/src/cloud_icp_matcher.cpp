#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <boost/program_options.hpp>

int main (int argc, char* argv[])
{
  boost::program_options::options_description desc("Options");
  desc.add_options()
    ("help", "Print help message");

  boost::program_options::variables_map vm;
  try {
    boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
    if( vm.count("help") ){
      std::cout << "Resize Point cloud using voxel grid filter" << std::endl;
      std::cerr << desc << std::endl;
      return 0;
    }
    boost::program_options::notify(vm);
  } catch (boost::program_options::error& e) {
    std::cerr << "ERROR: " << e.what() << std::endl << std::endl;
    std::cerr << desc << std::endl;
    return -1;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr my_points (new pcl::PointCloud<pcl::PointXYZ>);
  my_points->width = 9;
  my_points->height = 1;
  my_points->is_dense = false;
  my_points->points.resize(my_points->width * my_points->height);
  my_points->points[0].x = 0; my_points->points[0].y = 0; my_points->points[0].z = 0;
  my_points->points[1].x = 10.468375; my_points->points[1].y = -1.4e-5; my_points->points[1].z = -0.0443;
  my_points->points[2].x = 15.670134; my_points->points[2].y = 1.998623; my_points->points[2].z = -0.46705;
  my_points->points[3].x = 26.22456; my_points->points[3].y = 2.187819; my_points->points[3].z = -1.0161;
  my_points->points[4].x = 29.734006; my_points->points[4].y = 9.728766; my_points->points[4].z = -0.9221;
  my_points->points[5].x = 16.105749; my_points->points[5].y = 8.378591; my_points->points[5].z = -0.41734;
  my_points->points[6].x = 10.79801; my_points->points[6].y = 7.583685; my_points->points[6].z = -0.197;
  my_points->points[7].x = 3.1773751; my_points->points[7].y = 14.31936; my_points->points[7].z = -0.163;
  my_points->points[8].x = 1.490801; my_points->points[8].y = 8.878731; my_points->points[8].z = -0.0823;

  pcl::PointCloud<pcl::PointXYZ>::Ptr target_points (new pcl::PointCloud<pcl::PointXYZ>);
  target_points->width = 9;
  target_points->height = 1;
  target_points->is_dense = false;
  target_points->points.resize(target_points->width * target_points->height);
  target_points->points[0].x = 0; target_points->points[0].y = 0; target_points->points[0].z = 0;
  target_points->points[1].x = 10.613084; target_points->points[1].y = 0; target_points->points[1].z = -0.59540268;
  target_points->points[2].x = 15.782479; target_points->points[2].y = 2.029233477; target_points->points[2].z = -1.2316133;
  target_points->points[3].x = 26.397331; target_points->points[3].y = 2.422540545; target_points->points[3].z = -2.21392744;
  target_points->points[4].x = 29.823159; target_points->points[4].y = 9.952434095; target_points->points[4].z = -2.33933013;
  target_points->points[5].x = 16.125478; target_points->points[5].y = 8.378477795; target_points->points[5].z = -1.31785314;
  target_points->points[6].x = 10.893295; target_points->points[6].y = 7.641654581; target_points->points[6].z = -0.75705363;
  target_points->points[7].x = 3.1771734; target_points->points[7].y = 14.33753218; target_points->points[7].z = -0.2734411;
  target_points->points[8].x = 1.4750299; target_points->points[8].y = 8.883308285; target_points->points[8].z = -0.16165782;

  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputCloud(my_points);
  icp.setInputTarget(target_points);
  pcl::PointCloud<pcl::PointXYZ> Final;
  icp.align(Final);
  std::cout << "has converged: " << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
  std::cout << icp.getFinalTransformation() << std::endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_points(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(*my_points, *transformed_points, icp.getFinalTransformation());

  std::cout << "---------------------" << std::endl;
  for (size_t i = 0; i < transformed_points->width*transformed_points->height; ++i) {
    std::cout << transformed_points->points[i].x << "\t" << transformed_points->points[i].y << "\t" << transformed_points->points[i].z << std::endl;
  }
  return 0;
}
