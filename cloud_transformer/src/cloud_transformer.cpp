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
    ("help", "Print help message")
    ("pcd_filename", boost::program_options::value<std::string>()->required(), "pcl pcd filename");

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

  std::string pcd_fname = vm["pcd_filename"].as<std::string>();
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::io::loadPCDFile (pcd_fname, *source_cloud);

  // Rotate pointcloud 0.6[rad] around z axis
  Eigen::Matrix4f rotate_around_zaxis = Eigen::Matrix4f::Identity();
  float theta = 0.6;
  rotate_around_zaxis(0,0) = cos(theta);
  rotate_around_zaxis(0,1) = -sin(theta);
  rotate_around_zaxis(1,0) = sin(theta);
  rotate_around_zaxis(1,1) = cos(theta);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr rotated_cloud(new pcl::PointCloud<pcl::PointXYZRGB> ());
  pcl::transformPointCloud(*source_cloud, *rotated_cloud, rotate_around_zaxis);

  // Transform point cloud using homogeneous transformation matrix
  // which is got by icp mathcing between 3D map reference point and manual map reference point
  Eigen::Matrix4f icp_transform_matrix = Eigen::Matrix4f::Identity();
  icp_transform_matrix(0, 0) = 0.950235;
  icp_transform_matrix(0, 1) = -0.307961;
  icp_transform_matrix(0, 2) = 0.0470266;
  icp_transform_matrix(0, 3) = 0.147511;
  icp_transform_matrix(1, 0) = 0.30832;
  icp_transform_matrix(1, 1) = 0.951281;
  icp_transform_matrix(1, 2) = -0.000411891;
  icp_transform_matrix(1, 3) = -0.0305933;
  icp_transform_matrix(2, 0) = -0.0446088;
  icp_transform_matrix(2, 1) = 0.0148907;
  icp_transform_matrix(2, 2) = 0.998893;
  icp_transform_matrix(2, 3) = -0.0312964;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB> ());
  pcl::transformPointCloud(*rotated_cloud, *transformed_cloud, icp_transform_matrix);

  pcl::io::savePCDFileASCII("transformed_cloud.pcd", *transformed_cloud);
  std::cout << "Write to transformed_cloud.pcd" << std::endl;
  return 0;
}
