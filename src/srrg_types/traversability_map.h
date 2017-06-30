#pragma once

#include <iostream>

#include <srrg_types/defs.h>
#include <srrg_types/cloud_3d.h>

#include <srrg_boss/blob.h>

#include "image_data.h"

namespace srrg_core{

class TraversabilityMap : public srrg_boss::Serializable {
public:
    TraversabilityMap();

    void compute(srrg_core::Cloud3D* cloud_, const Eigen::Isometry3f& iso=Eigen::Isometry3f::Identity());

    virtual void serialize(srrg_boss::ObjectData& data, srrg_boss::IdContext& context);
    virtual void deserialize(srrg_boss::ObjectData& data, srrg_boss::IdContext& context);

    inline void setResolution(float resolution_){_resolution=resolution_;}
    inline float resolution() const {return _resolution;}

    inline void setDimensions(const Eigen::Vector3i dimensions_) { _dimensions = dimensions_;}
    inline const Eigen::Vector3i& dimensions() const { return _dimensions; }

    inline void setOrigin(const Eigen::Vector3f origin_) { _origin = origin_;}
    inline const Eigen::Vector3f& origin() const { return _origin; }

    inline void setNegate(int negate_){_negate=negate_;}
    inline int negate() const {return _negate;}

    inline void setOccupiedThreshold(float occupied_threshold_){_occupied_threshold=occupied_threshold_;}
    inline float occupiedThreshold() const {return _occupied_threshold;}

    inline void setFreeThreshold(float free_threshold_){_free_threshold=free_threshold_;}
    inline float freeThreshold() const {return _free_threshold;}

    inline void setDefaultValue(int default_value_){_default_value=default_value_;}
    inline int defaultValue() const {return _default_value;}

    inline void setRobotClimbStep(float robot_climb_step_){_robot_climb_step=robot_climb_step_;}
    inline float robotClimbStep() const {return _robot_climb_step;}

    inline void setRobotHeight(float robot_height_){_robot_height=robot_height_;}
    inline float robotHeight() const {return _robot_height;}

    inline ImageData* image() { return _image_ref.get();}
    inline void setImage(ImageData* i) { _image_ref.set(i);}
    inline ImageDataBLOBReference& imageReference() { return _image_ref;}

protected:
    float _resolution;
    Eigen::Vector3i _dimensions;
    Eigen::Vector3f _origin;
    int _negate;
    float _occupied_threshold;
    float _free_threshold;
    int _default_value;
    float _robot_climb_step;
    float _robot_height;

    ImageDataBLOBReference _image_ref;

};

}
