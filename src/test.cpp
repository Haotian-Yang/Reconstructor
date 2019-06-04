#include <iostream>
#include <dataloader.h>
#include <PCbuilder.h>

using namespace std;

int main(int argc, char* argv[]){
    if (argc != 3){
        cout << "please input ./<executable> <depth_path> <image_path>" << endl;
    }

    string depth_path = argv[1];
    string img_path = argv[2];

    cout << "depth file path:" << depth_path << endl;
    cout << "image file path:" << img_path << endl;
    // step 1: data loaded
    dataloader DL(depth_path, img_path);

    //step 2: pcd file saved
    PCbuilder PCDsaver(DL.img, DL.depth);

    PCDsaver.savePCDfile("../data/EndoAbS/PCDfiles/test.pcd");

    return 0;
}
