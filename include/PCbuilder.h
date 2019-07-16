/* @Author: eikoloki data:07/02/2019
 */

#ifndef PCBUILDER_H
#define PCBUILDER_H

#include <string>
#include <iostream>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

class PCbuilder{
private:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    std::string savePath;

public:
    /*
     * build point cloud from 3 channel depth
     * input: RGB image, depth map
     */

    PCbuilder(){
        ;
    }


    PCbuilder(cv::Mat image, cv::Mat depth){
        cloud = buildPointCloud(image, depth);
    }


    pcl::PointCloud<pcl::PointXYZRGB>::Ptr buildPointCloud(const cv::Mat &image, const cv::Mat &depth);
    pcl::PointCloud<pcl::PointXYZ>::Ptr buildPointCloud(const cv::Mat &depth);


    void savePCDfile(const std::string& savePath);
    std::string getSavePath() const;

};


#endif
