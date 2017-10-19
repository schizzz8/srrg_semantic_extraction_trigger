#include "structure_analyzer.h"

namespace srrg_semantic_extraction_trigger{
using namespace std;
using namespace srrg_core;

StructureAnalyzer::StructureAnalyzer(){

    _resolution = 0.05f;
    _origin = Eigen::Vector3f::Zero();
    _size = Eigen::Vector3i::Zero();

    _robot_climb_step = 0.05f;
    _robot_height = 0.5f;

}

void StructureAnalyzer::initGrid(){
    Eigen::Vector3f upper;
    _cloud.computeBoundingBox(_origin,upper);

    Eigen::Vector3f range = upper - _origin;
    _size = (range/_resolution).cast<int>();

    _rows = _size.y();
    _cols = _size.x();

    _indices.create(_rows,_cols);
    _elevations.create(_rows,_cols);
    _classified.create(_rows,_cols);
    _indices=-1;
    _elevations=4;
    _classified=127;
}

void StructureAnalyzer::compute(){

    // compute the elevatio of the surface
    for (size_t i=0; i<_cloud.size(); i++){
        const RichPoint3D& p = _cloud[i];
        float z = p.point().z();
        Eigen::Vector3f projected_point = (p.point() - Eigen::Vector3f(_origin.x(),_origin.y(),0))/_resolution;

        //compute row and column of the projection
        int r=projected_point.y();
        int c=projected_point.x();

        if (r>=_rows || r<0)
            continue;
        if (c>=_cols || r<0)
            continue;

        float &h=_elevations.at<float>(_rows-r-1,c);
        int& idx=_indices.at<int>(_rows-r-1,c);

        if (z<h) {
            h=z;
            idx=i;
        }
    }

    // mark the cells that are obstacles
    for (size_t i=0; i<_cloud.size(); i++){
        const RichPoint3D& p = _cloud[i];
        Eigen::Vector3f projected_point = (p.point() - Eigen::Vector3f(_origin.x(),_origin.y(),0))/_resolution;
        float z = p.point().z();

        //compute row and column of the projection
        int r=projected_point.y();
        int c=projected_point.x();

        //cerr << r << "," << c << " ";
        if (r>=_rows || r<0)
            continue;
        if (c>=_cols || r<0)
            continue;

        float &g=_elevations.at<float>(_rows-r-1,c);
        int& idx=_indices.at<int>(_rows-r-1,c);

        float min_obstacle_height=g+_robot_climb_step;
        float max_obstacle_height=g+_robot_height;

        if (z< min_obstacle_height) {
            continue;
        }
        if (z>max_obstacle_height)
            continue;

        idx=-2;
        g=z;
    }

    // fill in the invalid points
    for (int r=0; r<_rows; r++)
        for (int c=0; c<_cols; c++) {
            int idx = _indices.at<int>(r,c);
            if (idx==-1)
                continue;
            if (idx<-1){
                _classified.at<unsigned char>(r,c)=255; //obstacles
                continue;
            }
            _classified.at<unsigned char>(r,c)=0; //free
        }


    // clean the spurious points
    for (int r=1; r<_rows-1; r++)
        for (int c=1; c<_cols-1; c++) {
            unsigned char & cell=_classified.at<unsigned char>(r,c);
            if (cell!=255)
                continue;

            // seek for the 8 neighbors and isolate spurious points
            bool one_big=false;
            for (int rr=-1; rr<=1; rr++)
                for (int cc=-1; cc<=1; cc++) {
                    if (rr==0 && cc==0)
                        continue;
                    one_big |= _classified.at<unsigned char>(r+rr,c+cc)==255;
                }
            if (! one_big) {
                cell=0;
            }
        }
}
}
