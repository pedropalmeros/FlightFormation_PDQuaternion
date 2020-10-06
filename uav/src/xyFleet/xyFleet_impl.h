// %flair:license{
// This file is part of the Flair framework distributed under the
// CECILL-C License, Version 1.0.
// %flair:license}
/*!
 * \file xyFleet_impl.h
 * \brief Classe permettant le calcul d'un Pid
 * \author Guillaume Sanahuja, Copyright Heudiasyc UMR UTC/CNRS 7253
 * \date 2011/05/01
 * \version 4.0
 */

#ifndef XYFLEET_IMPL_H
#define XYFLEET_IMPL_H

#include <Object.h>

namespace flair {
namespace core {
class Matrix;
class io_data;
}
namespace gui {
class LayoutPosition;
class DoubleSpinBox;
}
namespace filter {
class xyFleet;
}
}

/*! \class xyFleet_impl
* \brief Class defining a PID
*/

class xyFleet_impl {
public:
  xyFleet_impl(flair::filter::xyFleet *self, const flair::gui::LayoutPosition *position,
           std::string name);
  ~xyFleet_impl();
  void UseDefaultPlot(const flair::gui::LayoutPosition *position);
  void Data4Formation(const flair::gui::LayoutPosition *position);
  void UpdateFrom(const flair::core::io_data *data);
  float i;
  bool first_update;
  void OwnSat(float &signal, double saturation_value);
  float SatVal(float value, double saturation_value);

  float FormationProtocol(int id, std::vector<float> &states);



private:
  flair::filter::xyFleet *self;

  // matrix
  flair::core::Matrix *state;

  flair::gui::DoubleSpinBox *T, *kp, *ki, *kd, *sat, *sati;
  flair::gui::DoubleSpinBox *alpha_1, *alpha_2;
  flair::gui::DoubleSpinBox *sat_u;
  flair::gui::DoubleSpinBox *sat_ec;

  flair::gui::DoubleSpinBox *FGain_01;
};

#endif // xyFleet_impl_H
