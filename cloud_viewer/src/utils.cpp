#include "utils.h"
#include <pcl/filters/voxel_grid.h>
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

RGB HSVToRGB(HSV hsv)
{
  double r = 0, g = 0, b = 0;
  hsv.H *= 360.0;
  if (hsv.S == 0)
  {
    r = hsv.V;
    g = hsv.V;
    b = hsv.V;
  }
  else
  {
    int i;
    double f, p, q, t;

    if (hsv.H == 360)
      hsv.H = 0;
    else
      hsv.H = hsv.H / 60;

    i = (int)trunc(hsv.H);
    f = hsv.H - i;

    p = hsv.V * (1.0 - hsv.S);
    q = hsv.V * (1.0 - (hsv.S * f));
    t = hsv.V * (1.0 - (hsv.S * (1.0 - f)));

    switch (i)
    {
      case 0:
        r = hsv.V; g = t; b = p;
        break;
      case 1:
        r = q; g = hsv.V; b = p;
        break;
      case 2:
        r = p; g = hsv.V; b = t;
        break;
      case 3:
        r = p; g = q; b = hsv.V;
        break;
      case 4:
        r = t; g = p; b = hsv.V;
        break;
      default:
        r = hsv.V; g = p; b = q;
        break;
    }
  }
  return RGB((unsigned char)(255 * r), (unsigned char)(255 * g), (unsigned char)(255 *b));
}

void colorizePointClouds(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud)
{
  double min, max;
  min = cloud->points[0].z;
  max = cloud->points[0].z;
  for (PointCloudT::iterator cloud_it = cloud->begin(); cloud_it != cloud->end(); ++cloud_it) {
    if(min > cloud_it->z){
      min = cloud_it->z;
    }
    if(max < cloud_it->z){
      max = cloud_it->z;
    }
  }
  double lut_scale = 1.0 / (max - min); // max is 255, min is 0
  if(min == max){
    lut_scale = 1.0;
  }
  for (PointCloudT::iterator cloud_it = cloud->begin(); cloud_it != cloud->end(); ++cloud_it) {
    HSV hsv_color = HSV((cloud_it->z - min) * lut_scale, 0.9, 0.9);
    RGB rgb_color = HSVToRGB(hsv_color);
    cloud_it->r = (int)rgb_color.R;
    cloud_it->g = (int)rgb_color.G;
    cloud_it->b = (int)rgb_color.B;
  }
}

void downsamplePointClouds(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,
                           pcl::PointCloud<pcl::PointXYZRGBA>::Ptr filtered_cloud,
                           float leafsize)
{
  pcl::VoxelGrid<pcl::PointXYZRGBA> sor;
  sor.setInputCloud(cloud);
  sor.setLeafSize(leafsize, leafsize, leafsize);
  sor.filter(*filtered_cloud);
}
