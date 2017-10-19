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

    _cloud_generator->setImage(_depth_image_path);
    _cloud_generator->setInverseK(_K);
    _cloud_generator->setSensorOffset(_sensor_offset);

    _cloud_generator->compute();
    _cloud = _cloud_generator->cloud();
}

void SemanticExtractionTrigger::analyzeStructure(){
    _structure_analyzer->setCloud(_cloud);

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

}
