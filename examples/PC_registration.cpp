#include <iostream>
#include <PCbuilder.h>
#include <dataloader.h>
#include <registrator.h>
#include <pcl/visualization/pcl_visualizer.h>


PointCloud::Ptr loadTiffPC(const std::string tiffFile){
    dataloader dl;
    PCbuilder PCB;
    cv::Mat PC_mat;
    PC_mat = dl.tiff_pointsLoader(tiffFile);
    PointCloud::Ptr PC(new PointCloud);
    PC = PCB.buildPointCloud(PC_mat);

    std::cout << "--point cloud successfully loaded--" << std::endl;
    return PC;
}


int main(int argc, char** argv){
    if(argc != 3){
        std::cerr << "input: ./<excutable> <target_pc_file> <source_pc_file>" << std::endl;
        return -1;
    }

    std::string target_pc_file = argv[1];
    std::string source_pc_file = argv[2];

    Registrator reg;

    reg.createViewPort();
    PointCloud::Ptr cloud_src(new PointCloud);
    PointCloud::Ptr cloud_tgt(new PointCloud);



    cloud_src = loadTiffPC(source_pc_file);
    cloud_tgt = loadTiffPC(target_pc_file);

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
