#include "path_map.h"
#include "dijkstra_path_search.h"
#include "distance_map_path_search.h"
#include "clusterer_path_search.h"
#include <math.h>

namespace srrg_core {

using namespace std;

void grayMap2indices(IntImage& dest,
                     const UnsignedCharImage& src,
                     unsigned char occ_threshold,
                     unsigned char free_threshold) {
    int rows=src.rows;
    int cols=src.cols;
    dest.create(rows, cols);
    int num_occupied=0;
    int num_free=0;
    int num_unknown=0;
    for (int r=0; r<rows; ++r){
        int* dest_ptr=dest.ptr<int>(r);
        const unsigned char* src_ptr=src.ptr<const unsigned char>(r);
        for (int c=0; c<cols; ++c, ++dest_ptr, ++ src_ptr){
            unsigned char cell_value=*src_ptr;
            if (*src_ptr>free_threshold){
                *dest_ptr=-1;
                ++num_free;
            } else if(*src_ptr<occ_threshold) {
                *dest_ptr = ++num_occupied;
            } else {
                *dest_ptr = -2;
                ++num_unknown;
            }
        }
    }
    /*
    cerr << "map made binary" << endl;
    cerr << "free cells:" << num_free << endl;
    cerr << "unknown cells:" << num_unknown << endl;
    cerr << "occupied cells:" << num_occupied <<  endl;
    */
}


void indices2distancePathMap(PathMap& distance_map,
                             const IntImage& indices,
                             float resolution,
                             float max_distance) {

    unsigned int int_max_distance  = max_distance/resolution;
    DistanceMapPathSearch dmap_calculator;
    dmap_calculator.setMaxDistance(int_max_distance);
    dmap_calculator.setIndicesImage(indices);
    dmap_calculator.setOutputPathMap(distance_map);
    dmap_calculator.init();
    dmap_calculator.compute();
}

void indices2distances(FloatImage& distances,
                       const IntImage& indices,
                       float resolution,
                       float max_distance) {
    unsigned int int_max_distance  = max_distance/resolution;
    DistanceMapPathSearch dmap_calculator;
    PathMap distance_map;
    dmap_calculator.setMaxDistance(int_max_distance);
    dmap_calculator.setIndicesImage(indices);
    dmap_calculator.setOutputPathMap(distance_map);
    dmap_calculator.init();
    dmap_calculator.compute();
    distances=dmap_calculator.distanceImage() * resolution;
}

void indices2customDistances(FloatImage& distances,
                             const IntImage& indices,
                             float resolution,
                             float max_distance) {
    unsigned int int_max_distance  = max_distance/resolution;
    PathMap distance_map;
    DistanceMapPathSearch dmap_calculator;
    dmap_calculator.setOutputPathMap(distance_map);
    dmap_calculator.setIndicesImage(indices);
    dmap_calculator.setMaxDistance(int_max_distance);
    dmap_calculator.init();
    dmap_calculator.compute();

    distances = dmap_calculator.distanceImage() * resolution;
    float mdist = 0;
    for (int r=0; r<indices.rows; r++){
        const float* dist_ptr = distances.ptr<const float>(r);
        for (int c=0; c<indices.cols; c++, dist_ptr++){
            const float& dist = *dist_ptr;
            if (dist == std::numeric_limits<float>::max())
                continue;
            mdist = (mdist < dist) ?  dist : mdist;
        }
    }
    mdist = std::sqrt(mdist);

    for (int r=0; r<indices.rows; ++r) {
        const int* src_ptr=indices.ptr<const int>(r);
        float* dist_ptr=distances.ptr<float>(r);
        for (int c=0; c<indices.cols; ++c, ++src_ptr, ++dist_ptr){
            if (*src_ptr<-1)
                *dist_ptr = .5f;
            else if (*src_ptr == -1)
                *dist_ptr = std::sqrt(*dist_ptr)/mdist;
            else
                *dist_ptr=0.f;
        }
    }
}

void distances2cost(FloatImage& dest,
                    const FloatImage& src,
                    float robot_radius,
                    float safety_region,
                    float min_cost,
                    float max_cost) {
    int rows=src.rows;
    int cols=src.cols;
    dest.create(rows, cols);
    dest=0;
    float delta=(safety_region+robot_radius)/2;
    float slope=(max_cost-min_cost)/(robot_radius-safety_region);
    float offset=max_cost-slope*robot_radius;

    /*
    cerr << endl;
    cerr << "robot_radius: " << robot_radius << endl;
    cerr << "safety_region: " << safety_region << endl;
    cerr << "min_cost: " << min_cost << endl;
    cerr << "max_cost: " << max_cost << endl;

    cerr << "slope: " << slope << endl;
    cerr << "offset: " << max_cost << endl;
    cerr << "delta: " << delta << endl;
    */


    for (int r=0; r<rows; ++r){
        float* dest_ptr=dest.ptr<float>(r);
        const float* src_ptr=src.ptr<const float>(r);
        for (int c=0; c<cols; ++c, ++dest_ptr, ++ src_ptr){
            float distance=*src_ptr;
            float& cost = *dest_ptr;
            if (distance<robot_radius){
                cost=max_cost;
                continue;
            }
            if (distance<safety_region){
                cost=slope*distance+offset;
                continue;
            }
            cost=min_cost;
        }
    }
    return;


    //    slope=10;
    //    for (int r=0; r<rows; ++r){
    //        float* dest_ptr=dest.ptr<float>(r);
    //        const float* src_ptr=src.ptr<const float>(r);
    //        for (int c=0; c<cols; ++c, ++dest_ptr, ++ src_ptr){
    //            float distance=*src_ptr;
    //            float& cost = *dest_ptr;
    //            if (distance<robot_radius){
    //                cost=max_cost;
    //                continue;
    //            }

    //            if(distance<safety_region){
    //                cost= (1/(1+exp(slope*(distance-delta))))*(max_cost-min_cost) + min_cost;
    //                continue;
    //            }

    //            cost=min_cost;

    //        }
    //    }
}

void distance2peaks(FloatImage& dest,
                    const FloatImage& src,
                    float offset,
                    int element_type,
                    int element_size){

    FloatImage marker;
    cv::subtract(src,cv::Scalar(offset),marker);

    cv::Mat dilated_image;
    cv::Mat element = cv::getStructuringElement(element_type,
                                                cv::Size (2*element_size + 1, 2*element_size + 1),
                                                cv::Point (element_size, element_size));

    bool iterate = true;
    while(iterate){
        cv::dilate(marker, dilated_image, element);
        cv::Mat min_image;
        cv::min(dilated_image,src,min_image);
        cv::Mat diff_image;
        cv::subtract(min_image,marker,diff_image);
        int diff = cv::countNonZero(diff_image);
        marker = min_image;
        if (diff == 0)
            iterate = false;
    }

    cv::subtract(src,marker,dest);

}

void peaks2clusters(ClustererPathSearch::ClusterVector& clusters,
                    const FloatImage &src){
    int rows=src.rows;
    int cols=src.cols;
    IntImage regions;
    regions.create(rows, cols);
    for (int r=0;r<rows; ++r) {
        int* regions_ptr=regions.ptr<int>(r);
        const float* src_ptr=src.ptr<const float>(r);
        for (int c=0;c<cols; ++c, ++src_ptr, ++ regions_ptr){
            *regions_ptr = (*src_ptr==0) ? -1 : 0;
        }
    }
    ClustererPathSearch clusterer;
    PathMap  output_map;
    clusterer.setOutputPathMap(output_map);
    clusterer.setRegionsImage(regions);
    clusterer.init();
    clusterer.compute();
    clusters = clusterer.clusters();

}

}
