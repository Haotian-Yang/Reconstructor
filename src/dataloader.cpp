/*@Author: eikoloki date: 06/30/2019
 */

#include <dataloader.h>

using namespace cv;
using namespace std;

Mat dataloader::xml_pointsLoader(const string& filepath){
    Mat depth;
    FileStorage depth_fs;

    depth_fs.open(filepath, FileStorage::READ);
    depth_fs["DepthMap"] >> depth;
    depth_fs.release();

    // Ouput properties
    cout << "-- XML file has been loaded --" << endl;
    cout << "XML Mat type:" << depth.type() << endl;
    cout << "XML Mat size:" << depth.size << endl;

    return depth;
}

Mat dataloader::imgLoader(const string& filepath){
    Mat img;
    img = imread(filepath, IMREAD_COLOR);

    // Output properties
    cout << "-- Image has been loaded --" << endl;
    cout << "Image type: " << img.type()  << endl;
    cout << "Image size: " << img.size << endl;

    return img;
}


Mat dataloader::tiff_pointsLoader(const string& filepath){
    Mat depth;
    depth = imread(filepath, IMREAD_UNCHANGED);
    // Output properties
    cout << "-- TIFF file has been loaded --" << endl;
    cout << "TIFF type: " << depth.type() << endl;
    cout << "TIFF size: " << depth.size << endl;
    cout << "TIFF channels: " << depth.channels() << endl;
    return depth;
}
