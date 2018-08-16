# KITTI_util
KITTI_util provides tools for the famous [KITTI Benchmark Suite](http://www.cvlibs.net/datasets/kitti/). Currently, it only supports the [Visual Odometry](http://www.cvlibs.net/datasets/kitti/eval_odometry.php) section, but you are encouraged to add extensions.

# Cloning as git submodule
You can clone KITTI_util as git submodule in your application with

    git submodule add https://github.com/sebdi/KITTI_util KITTI_util

# How to use

## Velodyne Data

To receive a pcl::PointCloud call getPointCloudByIndex(point_cloud,index) method, e.g.:

        pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        kitti.getPointCloudByIndex(point_cloud,index);


