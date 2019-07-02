#include <dataloader.h>
#include <PCbuilder.h>

using namespace std;

int main(int argc, char** argv){

    if (argc != 4){
        cerr << "please input ./<executable> <tiff_filepath> <image_filepath> <save_path>.pcd" << endl;
    }
    string tiff_path = argv[1];
    string img_path = argv[2];
    string pc_path = argv[3];

    dataloader DL(tiff_path, img_path);

    cv::Mat depth(DL.img.rows, DL.img.cols,CV_32FC3);
    cv::Rect R(0,0, DL.img.cols, DL.img.rows);
    depth = DL.depth(R);
    cout << "new depth size:" << depth.type() << endl;
    PCbuilder PCB(DL.img, depth);
    PCB.savePCDfile(pc_path);



}
