# MY POINTCLOUD TOOLS
This is my convinience point cloud processing tools.  

## Table of Contents
- Requirements
- Build
- Usage
- License

## Requirements
This tools require the following to build:  
- [PCL](http://pointclouds.org/) 1.7+
- [boost](http://www.boost.org/ )
- [cmake](https://cmake.org/)

## Build
You should use `out of source` method.  

```bash
$ cd my_pointcloud_tools  
$ mkdir build && cd build  
$ cmake ..  
$ make -j4
```

## Usage

You can use `--help` option.  
For example, 
```bash
$ cd build/cloud_viewer
$ ./cloud_viewer --help
```

## License

BSD
