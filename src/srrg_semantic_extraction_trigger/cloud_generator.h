#pragma once

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "srrg_types/cloud_3d.h"


namespace srrg_semantic_extraction_trigger{
class CloudGenerator{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    CloudGenerator();

    inline void setRawDepthScale(float raw_depth_scale_){_raw_depth_scale=raw_depth_scale_;}

    void setImage(const cv::Mat& image_);
    inline void setInverseK(const Eigen::Matrix3f& K_){_iK = K_.inverse();}
    inline void setSensorOffset(const Eigen::Isometry3f& sensor_offset_){_sensor_offset = sensor_offset_;}

    void compute();
    srrg_core::Cloud3D* cloud(){return _cloud;}
    void saveCloud();

private:
    cv::Mat _image;
    int _rows;
    int _cols;
    float _raw_depth_scale;
    srrg_core::Cloud3D* _cloud;
    Eigen::Matrix3f _iK;
    Eigen::Isometry3f _sensor_offset;
};
}
