#pragma once

#include "srrg_types/cloud_3d.h"

namespace srrg_semantic_extraction_trigger {

class StructureAnalyzer{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    StructureAnalyzer();

    inline void setCloud(srrg_core::Cloud3D* cloud_){_cloud = *cloud_;}
    inline void setResolution(float resolution_){_resolution = resolution_;}
    inline void setRobotClimbStep(float robot_climb_step_){_robot_climb_step = robot_climb_step_;}
    inline void setRobotHeight(float robot_height_){_robot_height = robot_height_;}

    void initGrid();

    void compute();

    const Eigen::Vector3f& origin(){return _origin;}
    const srrg_core::UnsignedCharImage& classified(){return _classified;}

private:

    srrg_core::Cloud3D _cloud;

    float _resolution;
    Eigen::Vector3f _origin;
    Eigen::Vector3i _size;

    int _rows;
    int _cols;

    float _robot_climb_step;
    float _robot_height;

    srrg_core::IntImage _indices;
    srrg_core::FloatImage _elevations;
    srrg_core::UnsignedCharImage _classified;
};

}
