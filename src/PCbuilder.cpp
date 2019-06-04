#include <PCbuilder.h>

using namespace std;
using namespace cv;
using namespace pcl;

PointCloud<PointXYZRGB>::Ptr PCbuilder::buildPointCloud(const Mat &image, const Mat &depth){

    PointCloud<PointXYZRGB>::Ptr PC(new PointCloud<PointXYZRGB>);
    for (int r = 0; r < image.rows; r++){
        for (int c = 0; c < image.cols; c++){
            PointXYZRGB p;
            p.x = depth.ptr<float>(c + r * image.cols)[0];
            p.y = depth.ptr<float>(c + r * image.cols)[1];
            p.z = depth.ptr<float>(c + r * image.cols)[2];

            p.b = image.ptr<uchar>(r)[3*c];
            p.g = image.ptr<uchar>(r)[3*c + 1];
            p.r = image.ptr<uchar>(r)[3*c + 2];

            PC->points.push_back(p);
        }
    }

    PC->height = image.rows;
    PC->width = image.cols;
    PC->is_dense = false;
    return PC;
}


void PCbuilder::savePCDfile(const std::string &savePath){
    this->savePath = savePath;
    cout << "-- Saving Point Cloud ... --" << endl;
    cout << "Point Cloud size: " << cloud->points.size() << endl;
    pcl::io::savePCDFile(savePath, *cloud);
    cout << "Point Cloud has been saved." << endl;
}


string PCbuilder::getSavePath(){
    return savePath;
}
