#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/filters/voxel_grid_occlusion_estimation.h>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

using PointT = PointXYZ;
using CloudT = PointCloud<PointT>;

static const float default_leaf_size = 0.1f;

int main()
{
    float leaf_x = default_leaf_size,
          leaf_y = default_leaf_size,
          leaf_z = default_leaf_size;

    CloudT::Ptr input_cloud(new CloudT);

    loadPCDFile("test_pcd.pcd", *input_cloud);

    VoxelGridOcclusionEstimation<PointXYZ> vg;
    vg.setInputCloud(input_cloud);
    vg.setLeafSize(leaf_x, leaf_y, leaf_z);
    vg.initializeVoxelGrid();

    Eigen::Vector3f b_min, b_max;
    b_min = vg.getMinBoundCoordinates();
    b_max = vg.getMaxBoundCoordinates();

    std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i> > occluded_voxels;
    vg.occlusionEstimationAll(occluded_voxels);

    std::cout << occluded_voxels.size() << '\n';

    for (int i = 0; i < occluded_voxels.size(); ++i)
    {
        auto &voxel = occluded_voxels[i];
        std::cout << "    " << voxel.x() << " " << voxel.y() << " " << voxel.z() << '\n';
    }

    CloudT::Ptr occ_centroids(new CloudT);
    occ_centroids->width = occluded_voxels.size();
    occ_centroids->height = 1;
    occ_centroids->is_dense = false;
    occ_centroids->points.resize(occluded_voxels.size());
    for (std::size_t i = 0; i < occluded_voxels.size(); ++i)
    {
        Eigen::Vector4f xyz = vg.getCentroidCoordinate(occluded_voxels[i]);
        PointT point;
        point.x = xyz[0];
        point.y = xyz[1];
        point.z = xyz[2];
        (*occ_centroids)[i] = point;
    }
    savePCDFile("occ_centroids.pcd", *occ_centroids);

    CloudT::Ptr cloud_centroids(new CloudT);
    cloud_centroids->width = input_cloud->size();
    cloud_centroids->height = 1;
    cloud_centroids->is_dense = false;
    cloud_centroids->points.resize(input_cloud->size());

    for (std::size_t i = 0; i < input_cloud->size(); ++i)
    {
        float x = (*input_cloud)[i].x;
        float y = (*input_cloud)[i].y;
        float z = (*input_cloud)[i].z;
        Eigen::Vector3i c = vg.getGridCoordinates(x, y, z);
        Eigen::Vector4f xyz = vg.getCentroidCoordinate(c);
        PointT point;
        point.x = xyz[0];
        point.y = xyz[1];
        point.z = xyz[2];
        (*cloud_centroids)[i] = point;
    }
    savePCDFile("cloud_centroids.pcd", *cloud_centroids);

    return 0;
}