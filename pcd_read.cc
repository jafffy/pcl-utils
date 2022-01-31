#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main()
{
    pcl::PointCloud<pcl::PointXYZ> cloud;

    pcl::io::loadPCDFile("test_pcd.pcd", cloud);

    std::cerr << "Loaded " << cloud.size() << " data points to test_pcd.pcd.\n";

    for (const auto &point : cloud)
    {
        std::cerr << "    " << point.x << " " << point.y << " " << point.z << '\n';
    }

    return 0;
}