#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace location {
class CloudData {
public:
    using POINT = pcl::PointXYZ;
    using CLOUD = pcl::PointCloud<POINT>;
    //using CLOUD_PTR = CLOUD::Ptr;

    //TODO:CLOUD_PTR() 
    CloudData():cloud(CLOUD()) {}

public:
    double time = 0.0;
    CLOUD cloud;
};
}