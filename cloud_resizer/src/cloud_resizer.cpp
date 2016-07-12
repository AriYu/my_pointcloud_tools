#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <boost/program_options.hpp>

bool save_flag = false;
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                            void* viewer_void)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
  if (event.getKeySym () == "s" && event.keyDown ())
  {
    save_flag = true;
    viewer->close();
  }
}


int main (int argc, char* argv[])
{
  boost::program_options::options_description desc("Options");
  desc.add_options()
    ("help", "Print help message")
    ("pcd_filename", boost::program_options::value<std::string>()->required(), "pcl pcd filename")
    ("vs", boost::program_options::value<double>()->required(), "Voxel size")
    ("saveas", boost::program_options::value<std::string>()->default_value("binary"), "Format of save pcd type. binary or ascii");

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
  std::cout << "If you want to save, please push 's' key." << std::endl;
  std::string pcd_fname = vm["pcd_filename"].as<std::string>();

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::io::loadPCDFile (pcd_fname, *cloud);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::VoxelGrid<pcl::PointXYZRGB> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (vm["vs"].as<double>(), vm["vs"].as<double>(), vm["vs"].as<double>());
  sor.filter (*cloud_filtered);

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("CloudViewer"));
  viewer->setBackgroundColor(250, 12, 242);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_filtered);
  viewer->addPointCloud<pcl::PointXYZRGB>(cloud_filtered, rgb, "resized cloud");

  viewer->addCoordinateSystem(1.0);
  viewer->initCameraParameters();
  viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)&viewer);

  while(!viewer->wasStopped())
  {
    viewer->spinOnce(100);
    boost::this_thread::sleep(boost::posix_time::microseconds(100000));
  }

  if(save_flag == true){
    if(vm["saveas"].as<std::string>() == "binary")
    {
      pcl::io::savePCDFileBinary("resized.pcd", *cloud_filtered);
      std::cout << "Write to resized.pcd as " << vm["saveas"].as<std::string>() << "."<< std::endl;
    }else if(vm["saveas"].as<std::string>() == "ascii")
    {
      pcl::io::savePCDFileASCII("resized.pcd", *cloud_filtered);
      std::cout << "Write to resized.pcd as " << vm["saveas"].as<std::string>() << "."<< std::endl;
    }else
    {
      std::cout << "fail to match." << std::endl;
    }
  }
  return 0;
}
