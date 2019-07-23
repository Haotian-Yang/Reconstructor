#include <iostream>
#include <PCbuilder.h>
#include <dataloader.h>
#include <registrator.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <opencv2/core/eigen.hpp>
#include <boost/filesystem.hpp>


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


void saveTransMat(cv::Mat trans, std::string savePath){
    cv::FileStorage fs(savePath, cv::FileStorage::WRITE);
    fs << "transformation" << trans;
    fs.release();

}

int main(int argc, char** argv){
//    if(argc != 3){
//        std::cerr << "input: ./<excutable> <target_pc_file> <source_pc_file>" << std::endl;
//        return -1;
//    }

    std::string rootpath = "/media/xiran_zhang/TOSHIBA EXT/MICCAI_SCARED/dataset1";

    std::array<std::string, 3> keyframe= {"/keyframe_1", "/keyframe_2", "/keyframe_3"};

    for(auto k : keyframe){

        std::string data = "/data";

        // this should be a ext(xml) file
        std::string target_pc_path = rootpath + k + data + "/scene_points_sgm";
        // this should be a tiff file
        std::string source_pc_path = rootpath + k + data + "/scene_points";
        // registration data
        std::string registration_path = rootpath + k + data + "/registration_data";
        std::string log_file = rootpath + k + data + "/log.ext";

        Registrator reg;
        cv::FileStorage fs_log(log_file, cv::FileStorage::WRITE);
        int fileIndex = 0;
        while(true){

            char target_name[30];
            char source_name[30];
            char registration_name[40];
            sprintf(target_name, "/scene_points%06d.ext", fileIndex);
            sprintf(source_name, "/scene_points%06d.tiff", fileIndex);
            sprintf(registration_name, "/registration_matrix%06d.ext", fileIndex);

            std::string source_pc_file = source_pc_path + source_name;
            std::string target_pc_file = target_pc_path + target_name;
            std::string registration_file = registration_path + registration_name;

            //reg.createViewPort();
            PointCloud::Ptr cloud_src(new PointCloud);
            PointCloud::Ptr cloud_tgt(new PointCloud);

            if(!fileExist(target_pc_file)){
                std::cout << "keyframe loaded over" << std::endl;
                break;
            }

            std::cout << "load target:"  << target_pc_file << std::endl;
            cloud_src = loadTiffPC(source_pc_file);
            //cloud_tgt = loadTiffPC(target_pc_file);
            cloud_tgt = loadExtPC(target_pc_file, "cloud");


            reg.setRegPointCloudPair(cloud_src, cloud_tgt);

            //reg.showCloudOriginal(cloud_src, cloud_tgt);

            PointCloud::Ptr output(new PointCloud);
            Eigen::Matrix4f trans;
            float CorrDis;
            CorrDis = reg.pairAlign(50.0, 0.6, 30, output, trans, true, 3);


            // add max correspondence distance to log
            char log_index[40];
            sprintf(log_index, "scene_points%06d", fileIndex);
            fs_log << log_index << CorrDis;


            cv::Mat transformation;
            cv::eigen2cv(trans, transformation);
            saveTransMat(transformation, registration_file);

            std::cout << "transformation saved to:" << registration_file << std::endl;
            fileIndex ++;

        }

        fs_log.release();


    }


    return 0;

}
