#include "remove_ground.h"
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>

std::string getTimeAsString()
{
  time_t rawtime;
  struct tm * timeinfo;
  char buffer[80];

  time (&rawtime);
  timeinfo = localtime(&rawtime);

  strftime(buffer,80,"%Y%m%d%I%M%S",timeinfo);
  std::string str(buffer);
  return str;
}

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

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("CloudViewer"));
  viewer->setBackgroundColor(0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(remover->getGroundRemovedCloudPtr());
  viewer->addPointCloud<pcl::PointXYZRGB>(remover->getGroundRemovedCloudPtr(), rgb, "removed cloud");

  viewer->addCoordinateSystem(1.0);
  viewer->initCameraParameters();
  viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)&viewer);
  std::vector<PlaneModelParam> plane_model_param = remover->getPlaneCoEfficientsVector();
  for (size_t i = 0; i < plane_model_param.size(); ++i) {
    std::stringstream plane_name;
    plane_name << "plane_" << i;
    viewer->addPlane(plane_model_param[i].coefficients_,
                     plane_model_param[i].position_.x,
                     plane_model_param[i].position_.y,
                     plane_model_param[i].position_.z, plane_name.str());
  }

  while(!viewer->wasStopped())
  {
    viewer->spinOnce(100);
    boost::this_thread::sleep(boost::posix_time::microseconds(100000));
  }
  if(save_flag == true){
    std::string dst_filename;
    dst_filename += "./";
    dst_filename += getTimeAsString();
    dst_filename += ".pcd";
    pcl::io::savePCDFileBinary(dst_filename, *remover->getGroundRemovedCloudPtr());
    std::cout << "Saved to : " << dst_filename << std::endl;
  }
  return 0;
}
