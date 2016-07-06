#include "remove_ground.h"
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>

int main(int argc, char *argv[])
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
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::io::loadPCDFile (pcd_fname, *cloud);

  Factory *factory = new RemoveGroundFactory();
  RemoveGroundBase *remover = factory->Create("RansacRemover", cloud);
  remover->removeGround();

  // pcl::visualization::CloudViewer viewer("Cloud Viewer");
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("CloudViewer"));
  viewer->setBackgroundColor(0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(remover->getGroundRemovedCloudPtr());
  viewer->addPointCloud<pcl::PointXYZRGB>(remover->getGroundRemovedCloudPtr(), rgb, "removed cloud");
  // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "removed cloud");
  viewer->addCoordinateSystem(1.0);
  viewer->initCameraParameters();
  std::vector<pcl::ModelCoefficients> coefficients = remover->getPlaneCoEfficientsVector();
  for (size_t i = 0; i < coefficients.size(); ++i) {
    std::stringstream plane_name;
    plane_name << "plane_" << i;
    viewer->addPlane(coefficients[i], plane_name.str());
  }
  // viewer.showCloud(remover->getGroundRemovedCloudPtr());
  while(!viewer->wasStopped())
  {
    viewer->spinOnce(100);
    boost::this_thread::sleep(boost::posix_time::microseconds(100000));
  }

  return 0;
}
