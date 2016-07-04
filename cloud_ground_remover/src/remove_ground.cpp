#include "remove_ground.h"

RemoveGroundBase* Factory::Create(std::string type,
                                  pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_cloud_ptr)
{
  RemoveGroundBase* p = CreateGroundRemover(type, source_cloud_ptr);
  return p;
}

RemoveGroundBase* RemoveGroundFactory::CreateGroundRemover(std::string type,
                                                           pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_cloud_ptr)
{
  if(type=="RansacRemover")
  {
    return new RansacRemoveGround(source_cloud_ptr);
  }
  else
  {
    std::cerr << "Unknown Remover : " << type << std::endl;
    exit(-1);
  }
}
