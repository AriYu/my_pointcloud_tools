#include "remove_ground.h"
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> Cloud;
typedef Cloud::ConstPtr CloudConstPtr;
typedef Cloud::Ptr CloudPtr;
typedef std::pair<std::string, CloudPtr> CloudPair;
typedef std::vector<CloudPair> CloudVector;


int main(int argc, char *argv[])
{
   std::vector<int> pcd_indices;
  pcd_indices = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr sum_clouds (new pcl::PointCloud<pcl::PointXYZRGBA>);
  
  CloudVector clouds;
  for (size_t i = 0; i < pcd_indices.size (); i++)
  {
    CloudPtr pc (new Cloud);
    pcl::io::loadPCDFile (argv[pcd_indices[i]], *pc);
    std::cout << "processing file: " << argv[pcd_indices[i]] << " size: " << pc->size () << std::endl;
    Factory *factory = new RemoveGroundFactory();
    RemoveGroundBase *remover = factory->Create("RansacRemover", pc);
    remover->removeGround();
    std::string result_filename (argv[pcd_indices[i]]);
    result_filename = result_filename.substr (result_filename.rfind ("/") + 1);
    result_filename = "rm_d_" + result_filename;
    pcl::io::savePCDFileBinary(result_filename, *remover->getGroundRemovedCloudPtr());
  }

  return 0;
}
