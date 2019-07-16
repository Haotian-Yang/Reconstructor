#include <iostream>
#include <registrator.h>
#include <pcl/visualization/pcl_visualizer.h>

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



    cloud_src = reg.loadTiffPC(source_pc_file);
    cloud_tgt = reg.loadTiffPC(target_pc_file);

    reg.setRegPointCloudPair(cloud_tgt, cloud_src);

    reg.showCloudOriginal(cloud_src, cloud_tgt);

    PointCloud::Ptr output(new PointCloud);
    Eigen::Matrix4f trans;
    reg.pairAlign(output, trans, true);

    std::cout<< trans << std::endl;
    return 0;

}
