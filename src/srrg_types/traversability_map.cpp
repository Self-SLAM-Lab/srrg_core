#include "traversability_map.h"

namespace srrg_core{

using namespace srrg_boss;

TraversabilityMap::TraversabilityMap(){
    _resolution = 0.05;
    _dimensions = Eigen::Vector3i::Zero();
    _origin = Eigen::Vector3f::Zero();
    _negate = 0;
    _occupied_threshold = 0.65;
    _free_threshold = 0.196;
    _default_value = 127;
    _robot_climb_step = 0.1;
    _robot_height = 0.5;
}

void TraversabilityMap::compute(Cloud3D *cloud_, const Eigen::Isometry3f &iso){
    srrg_core::IntImage indices;
    srrg_core::FloatImage elevations;
    srrg_core::Cloud3D transformed_cloud;

    srrg_core::UnsignedCharImage classified;
    indices.release();
    elevations.release();
    transformed_cloud.clear();
    if (! cloud_)
        return;
    transformed_cloud=*cloud_;

    transformed_cloud.transformInPlace(iso);

    float ires=1./_resolution;
    Eigen::Vector3f upper;
    transformed_cloud.computeBoundingBox(_origin, upper);

    Eigen::Vector3f range = upper - _origin;
    _dimensions = (range*ires).cast<int>();

    int cols=_dimensions.x();
    int rows=_dimensions.y();

    indices.create(rows,cols);
    elevations.create(rows,cols);
    classified.create(rows,cols);
    indices=-1;
    elevations=std::numeric_limits<float>::max();
    classified=_default_value;

    // compute the elevation of the surface
    for (size_t i=0; i<transformed_cloud.size(); i++){
        const RichPoint3D& p = transformed_cloud[i];
        float z = p.point().z();
        Eigen::Vector3f projected_point = (p.point() - _origin)*ires;

        //compute row and column of the projection
        int r=projected_point.y();
        int c=projected_point.x();

        if (r>=rows || r<0)
            continue;
        if (c>=cols || r<0)
            continue;

        float &h=elevations.at<float>(r,c);
        int& idx=indices.at<int>(r,c);
        if (z<h) {
            h=z;
            idx=i;
        }
    }

    // mark the cells that are obstacles
    for (size_t i=0; i<transformed_cloud.size(); i++){
        const RichPoint3D& p = transformed_cloud[i];
        Eigen::Vector3f projected_point = (p.point() - _origin)*ires;
        float z = p.point().z();

        if (p.normal().squaredNorm()< 0.1)
            continue;
        //compute row and column of the projection
        int r=projected_point.y();
        int c=projected_point.x();

        if (r>=rows || r<0)
            continue;
        if (c>=cols || r<0)
            continue;

        float &g=elevations.at<float>(r,c);
        int& idx=indices.at<int>(r,c);

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
    for (int r=0; r<rows; r++)
        for (int c=0; c<cols; c++) {
            int idx = indices.at<int>(r,c);
            if (idx==-1)
                continue;
            if (idx<-1){
                classified.at<unsigned char>(r,c)=255;
                continue;
            }
            classified.at<unsigned char>(r,c)=0;
        }


    // clean the spurious points
    for (int r=1; r<rows-1; r++)
        for (int c=1; c<cols-1; c++) {
            unsigned char & cell=classified.at<unsigned char>(r,c);
            if (cell!=255)
                continue;

            // seek for the 8 neighbors and isolate spurious points
            bool one_big=false;
            for (int rr=-1; rr<=1; rr++)
                for (int cc=-1; cc<=1; cc++) {
                    if (rr==0 && cc==0)
                        continue;
                    one_big |= classified.at<unsigned char>(r+rr,c+cc)==255;
                }
            if (! one_big) {
                cell=0;
            }
        }

    classified.copyTo(image()->image());
}

void TraversabilityMap::serialize(srrg_boss::ObjectData &data, srrg_boss::IdContext &context){

    data.setFloat("resolution",_resolution);

    _dimensions.toBOSS(data,"dimensions");

    _origin.toBOSS(data,"origin");

    data.setInt("negate",_negate);

    data.setFloat("occupied_threshold",_occupied_threshold);

    data.setFloat("free_threshold",_free_threshold);

    data.setInt("default_value",_default_value);

    data.setFloat("robot_climb_step",_robot_climb_step);

    data.setFloat("robot_height",_robot_height);

    ObjectData* imageBlobData=new ObjectData();
    data.setField("image", imageBlobData);
    _image_ref.serialize(*imageBlobData,context);
}

void TraversabilityMap::deserialize(srrg_boss::ObjectData &data, srrg_boss::IdContext &context){

    _resolution = data.getFloat("resolution");

    _dimensions.fromBOSS(data,"dimensions");

    _origin.toBOSS(data,"origin");

    _negate = data.getInt("negate");

    _occupied_threshold = data.getFloat("occupied_threshold");

    _free_threshold = data.getFloat("free_threshold");

    _default_value = data.getInt("default_value");

    _robot_climb_step = data.getFloat("robot_clim_step");

    _robot_height = data.getFloat("robot_height");

    ObjectData* imageBlobData = static_cast<ObjectData*>(data.getField("image"));
    _image_ref.deserialize(*imageBlobData,context);
}

}
