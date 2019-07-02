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

public:
    dataloader(std::string depthFile, std::string imgFile):depthPath(depthFile), imgPath(imgFile)
    {

        if (depthFile.substr(depthFile.find_last_of('.') + 1) == "xml"){
            depth = xml_depthLoader(depthFile);
        }
        else if(depthFile.substr(depthFile.find_last_of('.') + 1) == "tiff"){
            depth = tiff_depthLoader(depthFile);
        }
        img = imgLoader (imgFile);
    }

private:
    cv::Mat xml_depthLoader(const std::string& filepath);
    cv::Mat imgLoader(const std::string& filepath);

    /*
     * coorLoader: load a tiff file which contents 3 channels data representing [x, y, z]
     * coordinates of an point cloud
     * input: std::string of tiff file path.
     * output: CV_64FC3 Mat.
     */
    cv::Mat tiff_depthLoader(const std::string &filepath);
};

#endif
