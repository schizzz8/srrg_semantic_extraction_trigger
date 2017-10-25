#include "cloud_generator.h"

namespace srrg_semantic_extraction_trigger{

using namespace std;
using namespace srrg_core;

CloudGenerator::CloudGenerator(){

    _raw_depth_scale = 0.001;
    _iK = Eigen::Matrix3f::Identity();
    _sensor_offset = Eigen::Isometry3f::Identity();

}

void CloudGenerator::setImage(const cv::Mat &image_){
    _image = image_.clone();
    _rows = _image.rows;
    _cols = _image.cols;

    _cloud = new Cloud3D;
    _cloud->resize(_rows*_cols);

}

void CloudGenerator::compute(){

    for (int r=0; r<_rows; r++) {
        const unsigned short* id_ptr  = _image.ptr<unsigned short>(r);
        for (int c=0; c<_cols; c++) {
            unsigned short id = *id_ptr;
            float d = id * _raw_depth_scale;
            Eigen::Vector3f point = _sensor_offset * (_iK * Eigen::Vector3f(c*d,r*d,d));
            _cloud->at(c+r*_cols) = RichPoint3D(point);
            id_ptr++;
        }
    }
}

void CloudGenerator::saveCloud(){
    std::ofstream outfile;
    outfile.open("depth.cloud");
    _cloud->write(outfile);
    outfile.close();
}

}
