#pragma once

#include <stdexcept>
#include "srrg_path_map/base_path_search.h"

namespace srrg_semantic_extraction_trigger {

class ClustersExtractor: public srrg_core::BasePathSearch {
public:
    struct Cluster{
        srrg_core::PathMapCell* cell;
        Eigen::Vector2i lower;
        Eigen::Vector2i upper;
        float mean_r;
        float mean_c;
        int point_count;
    };
    typedef std::vector<Cluster> ClusterVector;

    ClustersExtractor();

    //! @param regions image: this is the input.
    // cell to cluster should be set to 0
    // each cell to ignore should be set to -1;
    inline void setRegionsImage(const srrg_core::IntImage& regions_image) {_regions_image=&regions_image;}

    virtual void init() override; //< call this once after setting indices image

    virtual bool compute() override;

    // returns a vector of cluster centroids
    inline const ClusterVector& clusters() const {return _clusters;}

protected:
    void fillFromImage();
    void expandRegion(srrg_core::PathMapCell* cell);

    int _max_index;
    const srrg_core::IntImage* _regions_image;
    ClusterVector _clusters;
};

}
