#include "joint_state_message.h"

#include <stdio.h>
#include "message_factory.h"
#include <iostream>

namespace srrg_core
{
 
JointStateMessage::JointStateMessage( const std::string& p_strTopic, const std::string& p_strFrameID, const int& p_uSeq, const double& p_dTimestamp ): BaseSensorMessage( p_strTopic, p_strFrameID, p_uSeq, p_dTimestamp )
{
    //ds nothing to do
}

  void JointStateMessage::serialize(srrg_boss::ObjectData& data, srrg_boss::IdContext& context) {
    BaseSensorMessage::serialize(data, context);
    srrg_boss::ArrayData* joints_array=new srrg_boss::ArrayData();
    for (size_t i=0; i<_joints.size(); ++i){
      JointStatus& joint=_joints[i];
      srrg_boss::ObjectData* joint_object=new srrg_boss::ObjectData;
      joint_object->setString("name", joint.name);
      joint_object->setDouble("position", joint.position);
      joint_object->setDouble("velocity", joint.velocity);
      joint_object->setDouble("effort", joint.effort);
      joints_array->push_back(joint_object);
    }
    data.setField("joints", joints_array);
  }
  
  void JointStateMessage::deserialize(srrg_boss::ObjectData& data, srrg_boss::IdContext& context) {
    BaseSensorMessage::deserialize(data, context);
    srrg_boss::ArrayData& joints_array=data.getField("joints")->getArray();
    _joints.resize(joints_array.size());
    for (size_t i=0; i<joints_array.size(); ++i){
      srrg_boss::ObjectData& joint_object=dynamic_cast<srrg_boss::ObjectData&>(joints_array[i]);
      JointStatus& joint=_joints[i];
      joint.name=joint_object.getString("name");
      joint.position=joint_object.getDouble("position");
      joint.velocity=joint_object.getDouble("velocity");
      joint.effort=joint_object.getDouble("effort");
    }
  }

const std::string JointStateMessage::m_strTag = "JOINT_STATE_MESSAGE";
  
void JointStateMessage::fromStream( std::istream& p_isMessage )
{
    //ds get base sensor info
    BaseSensorMessage::fromStream( p_isMessage );

    int num_joints;
    p_isMessage >> num_joints;
    _joints.resize(num_joints);
    for (size_t i=0; i<_joints.size(); i++){
      JointStatus& joint=_joints[i];
      p_isMessage >> joint.name >>  joint.position  >> joint.velocity  >> joint.effort;
    }
}
  
void JointStateMessage::toStream( std::ostream& p_osMessage ) const
{
    //ds stream the base elements
    BaseSensorMessage::toStream( p_osMessage );

    p_osMessage << " ";

    p_osMessage << _joints.size();
    for (size_t i=0; i<_joints.size(); i++){
      const JointStatus& joint=_joints[i];
      p_osMessage << " " << joint.name << " " << joint.position << " " << joint.velocity << " " << joint.effort << " ";
    }
}

  //ds register the message for reading
  static MessageFactory::MessageRegisterer< JointStateMessage > m_cRegisterer;

  BOSS_REGISTER_CLASS(JointStateMessage);
  
} //namespace srrg_core
