# Cloud LUM SLAM

## Usage

### example

```bash
$ cd path/to/my_cloud_tools/build/cloud_lum_slam
$ find path/to/cloudfiles -name "*.pcd" | sort | xargs ./cloud_lum_slam -i 200
```
Note that you need to sort argument parameters correctry.  
In above example, `sort` is used.  
So, if your number of file over 10, you should rename like this.  
- wrong
```
file1.pcd ... file10.pcd
```
- correct
```
file01.pcd file02.pcd ... file10.pcd
```


