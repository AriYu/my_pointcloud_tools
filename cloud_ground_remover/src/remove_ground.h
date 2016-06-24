#ifndef REMOVE_GROUND_H
#define REMOVE_GROUND_H

#include "ransac_remove_ground.h"

class Factory
{
public:
  RemoveGroundBase* Create(std::string type,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud_ptr);
private:
  virtual RemoveGroundBase* CreateGroundRemover(std::string type,
                                                pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud_ptr)=0;
};

class RemoveGroundFactory: public Factory
{
public:
  RemoveGroundBase* CreateGroundRemover(std::string type,
                                        pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud_ptr);
};

#endif /* REMOVE_GROUND_H */
