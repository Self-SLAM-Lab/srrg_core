#pragma once

#include "object_data.h"
#define EIGEN_MATRIXBASE_PLUGIN "srrg_boss/eigen_boss_plugin.hpp"
#include <Eigen/Core>

//ds default TO_BOSS and FROM_BOSS macros that enforce identical variable names for streaming and implementation
#define SRRG_TO_BOSS(OBJECT_DATA_, VARIABLE_NAME_) \
  _##VARIABLE_NAME_.toBOSS(data_, #VARIABLE_NAME_);

#define SRRG_FROM_BOSS(OBJECT_DATA_, VARIABLE_NAME_) \
  _##VARIABLE_NAME_.fromBOSS(data_, #VARIABLE_NAME_);

//ds Eigen.matrix() wrapping, TODO purge
#define SRRG_TO_BOSS_MATRIX(OBJECT_DATA_, VARIABLE_NAME_) \
  _##VARIABLE_NAME_.matrix().toBOSS(data_, #VARIABLE_NAME_);

#define SRRG_FROM_BOSS_MATRIX(OBJECT_DATA_, VARIABLE_NAME_) \
  _##VARIABLE_NAME_.matrix().fromBOSS(data_, #VARIABLE_NAME_);
