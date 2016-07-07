#ifndef REMOVE_GROUND_BASE_H
#define REMOVE_GROUND_BASE_H

#include <iostream>
#include <pcl/common/common.h>
#include <pcl/ModelCoefficients.h>

class PlaneModelParam
{
public:
  PlaneModelParam()
    :position_(0, 0, 0)
  {}
  pcl::PointXYZRGB position_;
  pcl::ModelCoefficients coefficients_;
};

class RemoveGroundBase
{
public:
  virtual void removeGround()=0;
  virtual pcl::PointCloud<pcl::PointXYZRGB>::Ptr getGroundRemovedCloudPtr()=0;
  virtual std::vector<PlaneModelParam> getPlaneCoEfficientsVector()=0;
};


#endif /* REMOVE_GROUND_BASE_H */
