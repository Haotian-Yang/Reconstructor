#include <dataloader.h>

using namespace cv;
using namespace std;

Mat dataloader::depthLoader(const string& filepath){
    Mat depth;
    FileStorage depth_fs;
    depth_fs.open(filepath, FileStorage::READ);
    depth_fs["DepthMap"] >> depth;
    depth_fs.release();

    // Ouput properties
    cout << "-- Depth file has been loaded --\n" << endl;
    cout << "Depth Mat type:" << depth.type() << "\n" << endl;
    cout << "Depth Mat size:" << depth.size << "\n" << endl;

    return depth;
}

Mat dataloader::imgLoader(const string &filepath){
    Mat img;
    img = imread(filepath, IMREAD_COLOR);

    // Output properties
    cout << "-- Image has been loaded --\n" << endl;
    cout << "Image type:" << img.type() << "\n" << endl;
    cout << "Image size:" << img.size << "\n" << endl;

    return img;
}
