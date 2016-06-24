#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <boost/program_options.hpp>



pcl::PointCloud<pcl::Normal>::Ptr surface_normals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (cloud);//法線の計算を行いたい点群を指定する

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());//KDTREEを作る
    ne.setSearchMethod (tree);//検索方法にKDTREEを指定する

    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);//法線情報を入れる変数

    ne.setRadiusSearch (0.1);//検索する半径を指定する

    ne.compute (*cloud_normals);//法線情報の出力先を指定する

    return cloud_normals;
}

pcl::PolygonMesh Generate_mesh(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal> ());
    cloud_normals = surface_normals(cloud);

    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields (*cloud, *cloud_normals, *cloud_with_normals);

    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud (cloud_with_normals);

    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    pcl::PolygonMesh triangles;

    gp3.setSearchRadius (0.1);

    gp3.setMu (2.5);
    gp3.setMaximumNearestNeighbors (100);
    gp3.setMaximumSurfaceAngle(M_PI/4);
    gp3.setMinimumAngle(M_PI/18);
    gp3.setMaximumAngle(2*M_PI/3);
    gp3.setNormalConsistency(false);

    gp3.setInputCloud (cloud_with_normals);
    gp3.setSearchMethod (tree2);
    gp3.reconstruct (triangles);

    std::vector<int> parts = gp3.getPartIDs();
    std::vector<int> states = gp3.getPointStates();

    return triangles;
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

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile (pcd_fname, *cloud);

  pcl::io::saveOBJFile("objfile.obj", Generate_mesh(cloud));
  
  return 0;
}
