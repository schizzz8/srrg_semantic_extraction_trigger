#pragma once

#include "cloud_generator.h"
#include "structure_analyzer.h"
#include "clusters_extractor.h"

namespace srrg_semantic_extraction_trigger{

class SemanticExtractionTrigger{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    SemanticExtractionTrigger(CloudGenerator* cloud_generator_=0,
                              StructureAnalyzer* structure_analyzer_=0,
                              ClustersExtractor* clusters_extractor_=0);

    inline void setDepthImagePath(std::string depth_image_path_){_depth_image_path=depth_image_path_;}
    inline void setK(const Eigen::Matrix3f& K_){_K = K_;}
    inline void setSensorOffset(const Eigen::Isometry3f& sensor_offset_){_sensor_offset=sensor_offset_;}

    void generateCloud();

    void analyzeStructure();

    void extractClusters();

private:
    cv::Mat _map_image;
    Eigen::Isometry3f _robot_transform;
    Eigen::Isometry3f _sensor_offset;
    Eigen::Matrix3f _K;
    std::string _depth_image_path;
    srrg_core::Cloud3D* _cloud;
    srrg_core::UnsignedCharImage _classified_image;
    ClustersExtractor::ClusterVector _clusters;

    CloudGenerator* _cloud_generator;
    StructureAnalyzer* _structure_analyzer;
    ClustersExtractor* _clusters_extractor;
};
}
