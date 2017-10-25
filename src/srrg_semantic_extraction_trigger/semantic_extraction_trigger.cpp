#include "semantic_extraction_trigger.h"

namespace srrg_semantic_extraction_trigger{

using namespace std;
using namespace srrg_core;

SemanticExtractionTrigger::SemanticExtractionTrigger(CloudGenerator *cloud_generator_,
                                                     StructureAnalyzer *structure_analyzer_,
                                                     ClustersExtractor *clusters_extractor_){

    if(!cloud_generator_)
        _cloud_generator = new CloudGenerator();
    else
        _cloud_generator = cloud_generator_;

    if(!structure_analyzer_)
        _structure_analyzer = new StructureAnalyzer();
    else
        _structure_analyzer = structure_analyzer_;

    if(!clusters_extractor_)
        _clusters_extractor = new ClustersExtractor();
    else
        _clusters_extractor = clusters_extractor_;

}

void SemanticExtractionTrigger::generateCloud(){

    _cloud_generator->compute();
    _cloud = _cloud_generator->cloud();

}

void SemanticExtractionTrigger::analyzeStructure(){
    _structure_analyzer->setCloud(_cloud);
    _structure_analyzer->initGrid();

    _structure_analyzer->compute();
    _classified_image = _structure_analyzer->classified().clone();
}

void SemanticExtractionTrigger::extractClusters(){
    int rows = _classified_image.rows;
    int cols = _classified_image.cols;

    IntImage regions;
    regions.create(rows,cols);
    for (int r=0;r<rows; ++r) {
        int* regions_ptr=regions.ptr<int>(r);
        const uchar* src_ptr=_classified_image.ptr<const uchar>(r);
        for (int c=0;c<cols; ++c, ++src_ptr, ++ regions_ptr){
            *regions_ptr = (*src_ptr==255) ? 0 : -1;
        }
    }

    PathMap  output_map;
    _clusters_extractor->setOutputPathMap(output_map);
    _clusters_extractor->setRegionsImage(regions);
    _clusters_extractor->init();

    _clusters_extractor->compute();
    _clusters = _clusters_extractor->clusters();

}

void SemanticExtractionTrigger::processClusters(const cv::Mat& map_image, const Eigen::Isometry3f &global_transform){
    cv::Mat image = map_image.clone();

    for(int i=0; i<_clusters.size(); i++){

        ClustersExtractor::Cluster cluster = _clusters[i];

        Eigen::Vector3f a_w = _structure_analyzer->grid2world(Eigen::Vector2i(cluster.lower.y(),cluster.upper.x()));
        Eigen::Vector3f d_w = _structure_analyzer->grid2world(Eigen::Vector2i(cluster.upper.y(),cluster.lower.x()));

        a_w = global_transform*a_w;
        d_w = global_transform*d_w;

        Eigen::Vector3f a_g = (a_w - _map_origin)/_map_resolution;
        Eigen::Vector3f d_g = (d_w - _map_origin)/_map_resolution;

        cv::Point2i a_gp (a_g.x(),image.rows-a_g.y()-1);
        cv::Point2i d_gp (d_g.x(),image.rows-d_g.y()-1);

        cv::rectangle(image,
                      a_gp,
                      d_gp,
                      cv::Scalar(255,0,0),
                      1);
    }

    cv::namedWindow("map with bounding boxes",CV_WINDOW_NORMAL);
    cv::imshow("map with bounding boxes",image);
    cv::waitKey(1);

}

}
