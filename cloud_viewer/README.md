# cloud_viewer tools

## `cloud_viewer`
### Usage example
```bash
$ cd path/to/my_cloud_tools/build/cloud_viewer
$ ./cloud_viewer --pcd_filename path/to/cloud.pcd --ds 1
```
### Options
- --help                Print help message
- --pcd_filename arg    pcl pcd filename
- --wc arg (=1)         Display point clouds with colors
- --ds arg (=0)         Display downsampled point cloud

## `multi_cloud_viewer`
This show some point clouds file as one pointcloud.  

### Usage examples
```bash
$ cd path/to/my_cloud_tools/build/cloud_viewer
$ ./multi_cloud_viewer path/to/cloud1.pcd path/to/cloud2.pcd
```
If you want to show many pcd files at once, you can use following command.  
```bash
$ cd path/to/my_cloud_tools/build/cloud_viewer
$ find path/to/pcd_files -name "*.pcd" | xargs ./cloud_viewer/multi_cloud_viewer
```
