#ifndef DATALOADER_H
#define DATALOADER_H

#include <string>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

class dataloader{
public:
    cv::Mat depth;
    cv::Mat img;
    std::string depthPath;
    std::string imgPath;

public:
    dataloader(std::string depthFile, std::string imgFile):depthPath(depthFile), imgPath(imgFile)
    {
        depth = depthLoader(depthFile);
        img = imgLoader (imgFile);
    }

private:
    cv::Mat depthLoader(const std::string& filepath);
    cv::Mat imgLoader(const std::string& filepath);
};

#endif
