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

    inline void setDepthImage(const cv::Mat& image_){_cloud_generator->setImage(image_);}
    inline void setK(const Eigen::Matrix3f& K_){_cloud_generator->setInverseK(K_);}
    inline void setSensorOffset(const Eigen::Isometry3f& sensor_offset_){_cloud_generator->setSensorOffset(sensor_offset_);}
    inline void setMapResolution(float resolution_){_map_resolution = resolution_;}
    inline void setMapOrigin(const Eigen::Vector3f& origin_){_map_origin = origin_;}

    inline CloudGenerator* cloudGenerator(){return _cloud_generator;}
    inline StructureAnalyzer* structureAnalyzer(){return _structure_analyzer;}
    inline ClustersExtractor* clustersExtractor(){return _clusters_extractor;}

    void generateCloud();

    void analyzeStructure();

    void extractClusters();

    const ClustersExtractor::ClusterVector& clusters(){return _clusters;}

    void processClusters(const cv::Mat& map_image,
                         const Eigen::Isometry3f& global_transform = Eigen::Isometry3f::Identity());

private:
    Eigen::Isometry3f _robot_transform;
    Eigen::Isometry3f _sensor_offset;
    Eigen::Matrix3f _K;
    std::string _depth_image_path;
    srrg_core::Cloud3D* _cloud;
    srrg_core::UnsignedCharImage _classified_image;
    ClustersExtractor::ClusterVector _clusters;
    float _map_resolution;
    Eigen::Vector3f _map_origin;

    CloudGenerator* _cloud_generator;
    StructureAnalyzer* _structure_analyzer;
    ClustersExtractor* _clusters_extractor;
};
}
