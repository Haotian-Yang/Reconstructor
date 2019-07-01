#ifndef DATALOADER_H
#define DATALOADER_H

#include <string>
#include <iostream>
#include <string_view>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

class dataloader{
public:
    cv::Mat depth;
    cv::Mat img;
    cv::Mat coordinates;

    std::string depthPath;
    std::string imgPath;
    std::string coorPath;

public:
    dataloader(std::string depthFile, std::string imgFile):depthPath(depthFile), imgPath(imgFile)
    {
        depth = depthLoader(depthFile);
        img = imgLoader (imgFile);
    }

    dataloader(std::string coorFile, std::string imgFile = nullptr):coorPath(coorFile), imgPath(imgFile)
    {
        coordinates = coorLoader(coorFile);
        if( imgFile != nullptr)
            img = imgLoader(imgFile);
    }

private:
    cv::Mat depthLoader(std::string_view filepath);
    cv::Mat imgLoader(std::string_view filepath);

    /* @author: eikoloki date: 06/30/2019
     * coorLoader: load a tiff file which contents 3 channels data representing [x, y, z]
     * coordinates of an point cloud
     * input: std::string of tiff file path.
     * output: CV_64FC3 Mat.
     */
    cv::Mat coorLoader(std::string_view filepath);
};

#endif
