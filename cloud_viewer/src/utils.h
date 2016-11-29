#ifndef UTILS_H
#define UTILS_H

#include <pcl/io/io.h>

class RGB
{
public:
  unsigned char R;
  unsigned char G;
  unsigned char B;

  RGB(unsigned char r, unsigned char g, unsigned char b)
  {
    R = r;
    G = g;
    B = b;
  }

  bool Equals(RGB rgb)
  {
    return (R == rgb.R) && (G == rgb.G) && (B == rgb.B);
  }
};

class HSV
{
public:
  double H;
  double S;
  double V;

  HSV(double h, double s, double v)
  {
    H = h;
    S = s;
    V = v;
  }

  bool Equals(HSV hsv)
  {
    return (H == hsv.H) && (S == hsv.S) && (V == hsv.V);
  }
};

RGB HSVToRGB(HSV hsv);
void colorizePointClouds(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);
void downsamplePointClouds(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,
                           pcl::PointCloud<pcl::PointXYZRGBA>::Ptr filtered_cloud,
                           float leafsize);
#endif /* UTILS_H */
