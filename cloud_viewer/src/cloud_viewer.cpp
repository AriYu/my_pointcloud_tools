#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <boost/program_options.hpp>

int user_data;

void viewerOneOff (pcl::visualization::PCLVisualizer& viewer)
{
    viewer.setBackgroundColor (1.0, 0.5, 1.0);
    pcl::PointXYZ o;
    o.x = 1.0;
    o.y = 0;
    o.z = 0;
    viewer.addSphere (o, 0.25, "sphere", 0);
    std::cout << "i only run once" << std::endl;
}

void viewerPsycho (pcl::visualization::PCLVisualizer& viewer)
{
    // static unsigned count = 0;
    // std::stringstream ss;
    // ss << "Once per viewer loop: " << count++;
    // viewer.removeShape ("text", 0);
    // viewer.addText (ss.str(), 200, 300, "text", 0);
    // //FIXME: possible race condition here:
    // user_data++;
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

  //blocks until the cloud is actually rendered
  viewer.showCloud(cloud);

  //use the following functions to get access to the underlying more advanced/powerful
  //PCLVisualizer


  //This will only get called once
  viewer.runOnVisualizationThreadOnce (viewerOneOff);

  //This will get called once per visualization iteration
  viewer.runOnVisualizationThread (viewerPsycho);
  while (!viewer.wasStopped ())
  {
    //you can also do cool processing here
    //FIXME: Note that this is running in a separate thread from viewerPsycho
    //and you should guard against race conditions yourself...
    user_data++;
  }
  return 0;
}
