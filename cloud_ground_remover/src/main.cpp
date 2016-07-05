#include "remove_ground.h"
#include <pcl/visualization/cloud_viewer.h>

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
  RemoveGroundBase *remover = factory->Create("RegionGrowingSegmentationRemover", cloud);
  remover->removeGround();

  pcl::visualization::CloudViewer viewer("Cloud Viewer");
  viewer.showCloud(remover->getGroundRemovedCloudPtr());
  while(!viewer.wasStopped())
  {
  }
  
  return 0;
}
