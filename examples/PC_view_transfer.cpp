#include <dataloader.h>
#include <PCbuilder.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/core/persistence.hpp>



#include <iostream>
#include <string_view>
#include <boost/filesystem.hpp>


typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;


bool fileExist(std::string filename){
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    std::cout << filename << "," << fs.isOpened() << std::endl;
    return fs.isOpened();
}

cv::Mat cutLeftCloud(cv::Mat raw){
    cv::Mat left_cloud;
    cv::Rect left_ROI(0,0, 1280, 1024);
    left_cloud = raw(left_ROI);
    return left_cloud;
}


PointCloud::Ptr loadTiffPC(const std::string tiffFile){
    dataloader dl;
    PCbuilder PCB;
    cv::Mat PC_mat;
    cv::Mat left_PC;
    PC_mat = dl.tiff_pointsLoader(tiffFile);
    left_PC = cutLeftCloud(PC_mat);
    PointCloud::Ptr PC(new PointCloud);
    PC = PCB.buildPointCloud(left_PC, PCbuilder::DepthFileType::DEPTHTYPE_TIFF);

    // debug left point cloud
    std::cout << "left PC size:" << left_PC.size << std::endl;
    std::cout << "left PC type:" << left_PC.type() << std::endl;

    std::cout << "--Tiff point cloud loaded successfully--" << std::endl;
    return PC;
}


PointCloud::Ptr loadExtPC(const std::string extFile, const std::string keyword){
    dataloader dl;
    PCbuilder PCB;
    cv::Mat PC_mat;
    PC_mat = dl.xml_pointsLoader(extFile, keyword);
    PointCloud::Ptr PC(new PointCloud);
    PC = PCB.buildPointCloud(PC_mat, PCbuilder::DepthFileType::DEPTHTYPE_EXT);

    std::cout << "Ext point cloud loaded successfully--" << std::endl;
    return PC;
}

cv::Mat loadExtTransMat(std::string_view extFile, std::string_view keyword){
    cv::FileStorage fs(std::string(extFile).c_str(), cv::FileStorage::READ);
    cv::Mat trans;
    fs[std::string(keyword).data()] >> trans ;
    return trans;

}


int main(int argc, char** argv){

    // test registrated transformation matrix
    std::string rootpath = "../demo/transformation_test";
    std::string original_pc_path = rootpath + "/original_pc.tiff";
    std::string sgm_pc_path = rootpath + "/sgm_pc.ext";
    std::string trans_mat_path = rootpath + "trans_mat.ext";

    // step 1: load original point cloud (The Ground Truth), TIFF file
    PointCloud::Ptr original_pc(new PointCloud);
    original_pc = loadTiffPC(original_pc_path);

    // step 2: load sgm generated point cloud (The geometric truth), EXT file

    PointCloud::Ptr sgm_pc(new PointCloud);
    sgm_pc = loadExtPC(sgm_pc_path, "cloud");

    // step 3: load transformation matrix between these two clouds, EXT file
    cv::Mat trans_mat;
    trans_mat = loadExtTransMat(trans_mat_path, "transformation");
    // step 4: visulize the original ground truth with color along side with the transformed
    //         cloud with color

    return 1;
}
