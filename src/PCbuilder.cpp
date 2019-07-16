/* @Author: eikoloki data:07/02/2019
 */

#include <PCbuilder.h>

using namespace std;
using namespace cv;
using namespace pcl;


PointCloud<PointXYZRGB>::Ptr PCbuilder::buildPointCloud(const Mat &image, const Mat &PC_mat){

    PointCloud<PointXYZRGB>::Ptr PC(new PointCloud<PointXYZRGB>);
    cout << "img rows:" << image.rows << " PC rows:" << PC_mat.rows << endl;
    for (int r = 0; r < image.rows; r++){
        for (int c = 0; c < image.cols; c++){
            PointXYZRGB p;
            if (PC_mat.type() == CV_32FC3){
                p.x = PC_mat.ptr<float>(r)[3*c];
                p.y = PC_mat.ptr<float>(r)[3*c + 1];
                p.z = PC_mat.ptr<float>(r)[3*c + 2];
                if (p.x > 200 || p.x < -200){
                    cout << "x: " <<p.x << endl;
                    p.x = 0.0;
                }
                if (p.y > 200  || p.y < -200){
                    cout << "y: " << p.y << endl;
                    p.y = 0.0;
                }
                if (p.z > 200 || p.z < -200){
                    cout << "z: " << p.z << endl;
                    p.z = 0.0;
                }

            }
            else {
                p.x = PC_mat.ptr<float>(c + r * image.cols)[0];
                p.y = PC_mat.ptr<float>(c + r * image.cols)[1];
                p.z = PC_mat.ptr<float>(c + r * image.cols)[2];
            }


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



PointCloud<PointXYZ>::Ptr PCbuilder::buildPointCloud(const Mat &PC_mat){
    PointCloud<PointXYZ>::Ptr PC(new PointCloud<PointXYZ>);
    for (int r = 0; r < PC_mat.rows; r++){
        for (int c = 0; c < PC_mat.cols; c++){
            PointXYZ p;
            if (PC_mat.type() == CV_32FC3){
                p.x = PC_mat.ptr<float>(r)[3*c];
                p.y = PC_mat.ptr<float>(r)[3*c + 1];
                p.z = PC_mat.ptr<float>(r)[3*c + 2];
                if (p.x > 200 || p.x < -200){
                    //cout << "x: " <<p.x << endl;
                    p.x = 0.0;
                }
                if (p.y > 200  || p.y < -200){
                    //cout << "y: " << p.y << endl;
                    p.y = 0.0;
                }
                if (p.z > 200 || p.z < -200){
                    //cout << "z: " << p.z << endl;
                    p.z = 0.0;
                }

            }
            else {
                p.x = PC_mat.ptr<float>(c + r * PC_mat.cols)[0];
                p.y = PC_mat.ptr<float>(c + r * PC_mat.cols)[1];
                p.z = PC_mat.ptr<float>(c + r * PC_mat.cols)[2];
            }

            PC->points.push_back(p);
        }
    }

    PC->height = PC_mat.rows;
    PC->width = PC_mat.cols;
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


string PCbuilder::getSavePath() const{
    return savePath;
}
