#include <dataloader.h>

using namespace cv;
using namespace std;

Mat dataloader::depthLoader(string_view filepath){
    Mat depth;
    FileStorage depth_fs;

    depth_fs.open(filepath, FileStorage::READ);
    depth_fs["DepthMap"] >> depth;
    depth_fs.release();

    // Ouput properties
    cout << "-- Depth file has been loaded --\n" << endl;
    cout << "Depth Mat type:" << depth.type() << endl;
    cout << "Depth Mat size:" << depth.size << endl;

    return depth;
}

Mat dataloader::imgLoader(string_view filepath){
    Mat img;
    img = imread(filepath, IMREAD_COLOR);

    // Output properties
    cout << "-- Image has been loaded --\n" << endl;
    cout << "Image type: " << img.type()  << endl;
    cout << "Image size: " << img.size << endl;

    return img;
}


Mat dataloader::coorLoader(string_view filepath){
    Mat coordinates;
    coordinates = imread(filepath, -1);
    // Output properties
    cout << "-- Coordinates file has been loaded --\n" << endl;
    cout << "Coordinates type: " << coordinates.type() << endl;
    cout << "Coordinates size: " << coordinates.size << endl;
}
