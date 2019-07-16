/* @Author: eikoloki data: 07/16/2019
 */

#pragma once

#include <iostream>

#include <boost/make_shared.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <dataloader.h>
#include <PCbuilder.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

class PointCurvature: public pcl::PointRepresentation<PointNormalT>{
    using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
public:
    PointCurvature(){
        // number of dimensions
        nr_dimensions_ = 4;
    }

    // Override copyToFloatArray method to define our feature vector
    virtual void copyToFloatArray(const PointNormalT &p, float *out) const{
        // < x, y, z, curvature>
        out[0] = p.x;
        out[1] = p.y;
        out[2] = p.z;
        out[4] = p.curvature;
    }
};

using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;

class Registrator{
private:
    int vp1, vp2;
    bool visual = false;
    PointCloud::Ptr cloud_tgt;
    PointCloud::Ptr cloud_src;

    std::string source_file;
    std::string target_file;

    pcl::visualization::PCLVisualizer *p;

    Eigen::Matrix4f transformation;

public:
    void createViewPort();
    void showCloudOriginal(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt);
    void showCloudResult(const PointCloudWithNormals::Ptr cloud_src, const PointCloudWithNormals::Ptr cloud_tgt);

    void setRegPointCloudPair(PointCloud::Ptr &target, PointCloud::Ptr &source);

    PointCloud::Ptr loadTiffPC(const std::string tiffFile);

    void pairAlign(PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample = false);



};
