#include <PCbuilder.h>

using namespace std;
using namespace cv;
using namespace pcl;

PointCloud<PointXYZRGB> PCbuilder::buildPointCloud(const Mat &image, const Mat &depth){

    PointCloud<PointXYZRGB>::Ptr PC(new PointCloud);
    for (int r = 0; r < depth.rows; r++){
        for (int c = 0; c < depth.cols; c++){
            PointXYZRGB p;
            p.x = depth.ptr<float>(r)[3*c];
            p.y = depth.ptr<float>(r)[3*c + 1];
            p.z = depth.ptr<float>(r)[3*c + 2];

            p.b = image.ptr<uchar>(r)[3*c];
            p.g = image.ptr<uchar>(r)[3*c + 1];
            p.r = image.ptr<uchar>(r)[3*c + 2];

            PC->points.push_back(p);
        }
    }
}


void PCbuilder::savePCDfile(const std::string &savePath){
    this->savePath = savePath;
    cout << "Point Cloud size: " << cloud->points.size() << endl;
    pcl::io::savePCDFile(savePath, *cloud);
    cout << "Point Cloud has been saved." << endl;
}


string PCbuilder::getSavePath(){
    return savePath;
}
