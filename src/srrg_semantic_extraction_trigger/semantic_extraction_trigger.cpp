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

    cerr << "saving cloud..." << endl;
    std::ofstream outfile;
    outfile.open("depth.cloud");
    _cloud->write(outfile);
    outfile.close();
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

void SemanticExtractionTrigger::processClusters(string path){

    cv::Mat map = cv::imread(path);

    float resolution = 0.05f;
    Eigen::Vector3f origin (-23.4f,-12.2f,0);

    for(int i=0; i<_clusters.size(); i++){

        ClustersExtractor::Cluster cluster = _clusters[i];

        Eigen::Vector3f a_w (cluster.lower.y()*resolution + _structure_analyzer->origin().x(),
                             (_classified_image.rows-cluster.upper.x()-1)*resolution + _structure_analyzer->origin().y(),
                             0);
        Eigen::Vector3f d_w (cluster.upper.y()*resolution + _structure_analyzer->origin().x(),
                             (_classified_image.rows-cluster.lower.x()-1)*resolution + _structure_analyzer->origin().y(),
                             0);

        Eigen::Vector3f a_g = (a_w - origin)/resolution;
        Eigen::Vector3f d_g = (d_w - origin)/resolution;

        cv::Point2i a_gp (a_g.x(),map.rows-a_g.y()-1);
        cv::Point2i d_gp (d_g.x(),map.rows-d_g.y()-1);

        cv::rectangle(map,
                      a_gp,
                      d_gp,
                      cv::Scalar(255,0,0),
                      1);
    }

    cv::imshow("map with bounding boxes",map);
    cv::waitKey();

}

}
