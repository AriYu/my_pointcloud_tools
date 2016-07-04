#ifndef REMOVE_GROUND_BASE_H
#define REMOVE_GROUND_BASE_H

#include <iostream>
#include <pcl/common/common.h>

class RemoveGroundBase
{
public:
  virtual void removeGround()=0;
  virtual pcl::PointCloud<pcl::PointXYZRGB>::Ptr getGroundRemovedCloudPtr()=0;
};


#endif /* REMOVE_GROUND_BASE_H */
