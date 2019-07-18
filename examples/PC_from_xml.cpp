#include <iostream>
#include <dataloader.h>
#include <PCbuilder.h>
#include <fstream>

using namespace std;

int main(int argc, char* argv[]){
    if (argc != 3){
        cerr << "please input ./<executable> <rgb_file> <depth_file> <pcd_save_path>" << endl;
    }

    string dataset_path = argv[1];
    string save_path = argv[2];
    /*
    cout << "dataset path:" << dataset_path << endl;
    cout << "save path:" << save_path<< endl;

    // step 1: data loaded
    array<string, 1> tissue_dir = {"spleen/"};
    array<string, 1> distance_dir = {"distance_min/"};
    array<string, 4> sub_dir = {"pose1/", "pose2/", "pose3/", "pose4/"};
    array<string, 6> image_dir = {"imgL_1.png", "imgL_2.png", "imgL_3.png",
                                  "imgL_4.png", "imgL_5.png", "imgL_6.png" };

    for(auto t : tissue_dir){
        for(auto d : distance_dir){
            for(auto s : sub_dir){
                for (auto i : image_dir){

                    string img_path = dataset_path + t + d + s + i;
                    string depth_path = dataset_path + t + d + s + "depthMap.xml";
                    cout << img_path << endl;
                    dataloader DL(depth_path, img_path);
                    cout << DL.depth.type() << endl;

                    //step 2: pcd file saved
                    PCbuilder PCDsaver(DL.img, DL.depth);
                    string pcd_path = save_path + t + d + s + i.substr(0, i.find(".")) + ".PCD";
                    cout << pcd_path << endl;
                    PCDsaver.savePCDfile(pcd_path);
                }
            }
        }
    }
    */

    return 0;
}
