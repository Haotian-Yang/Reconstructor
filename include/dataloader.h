/*@Author: eikoloki date: 06/30/2019
 */
#ifndef DATALOADER_H
#define DATALOADER_H

#include <string>
#include <iostream>
#include <string_view>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <tiffio.h>

class dataloader{
public:
    cv::Mat depth;
    cv::Mat img;
private:
    std::string depthPath;
    std::string imgPath;
    std::string keyword;

public:
    dataloader(){
        ;
    }


    dataloader(std::string depthFile, std::string imgFile, std::string depthKeyword = "cloud"):depthPath(depthFile), imgPath(imgFile), keyword(depthKeyword)
    {

        if (depthFile.substr(depthFile.find_last_of('.') + 1) == "xml" || depthFile.substr(depthFile.find_last_of('.') + 1) == "ext"){
            depth = xml_pointsLoader(depthFile, depthKeyword);
        }
        else if(depthFile.substr(depthFile.find_last_of('.') + 1) == "tiff"){
            depth = tiff_pointsLoader(depthFile);
        }
        img = imgLoader (imgFile);
    }


    /*
     * coorLoader: load a tiff file which contents 3 channels data representing [x, y, z]
     * coordinates of an point cloud
     * input: std::string of tiff file path.
     * output: CV_64FC3 Mat.
     */
    cv::Mat tiff_pointsLoader(const std::string &filepath);
    cv::Mat xml_pointsLoader(const std::string& filepath, const std::string& key);
    cv::Mat imgLoader(const std::string& filepath);

};

#endif
