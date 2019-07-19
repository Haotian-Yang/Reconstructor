#include <iostream>
#include <PCbuilder.h>
#include <dataloader.h>
#include <registrator.h>
#include <pcl/visualization/pcl_visualizer.h>


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

//    for (int r = 0; r < left_PC.rows; r++){
//        for (int c = 0; c < left_PC.cols; c++){
//            std::cout << "r: " << r  << ",c: " << c << ",x:" << left_PC.ptr<float>(r)[3*c] << std::endl;
//        }
//    }

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


int main(int argc, char** argv){
    if(argc != 3){
        std::cerr << "input: ./<excutable> <target_pc_file> <source_pc_file>" << std::endl;
        return -1;
    }

    // this should be a ext(xml) file
    std::string target_pc_file = argv[1];
    // this should be a tiff file
    std::string source_pc_file = argv[2];

    Registrator reg;

    reg.createViewPort();
    PointCloud::Ptr cloud_src(new PointCloud);
    PointCloud::Ptr cloud_tgt(new PointCloud);



    cloud_src = loadTiffPC(source_pc_file);
    cloud_tgt = loadExtPC(target_pc_file, "cloud");

    reg.setRegPointCloudPair(cloud_tgt, cloud_src);

    reg.showCloudOriginal(cloud_src, cloud_tgt);

    PointCloud::Ptr output(new PointCloud);
    Eigen::Matrix4f trans;

    auto start = cv::getTickCount();
    reg.pairAlign(50.0, 0.6, 30, output, trans, true);
    auto end = cv::getTickCount();
    std::cout << "elapsed time:" <<  1000 * (end - start) /cv::getTickFrequency() << std::endl;
    std::cout<< trans << std::endl;
    return 0;

}
