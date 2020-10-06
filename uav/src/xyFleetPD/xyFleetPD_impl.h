// %flair:license{
// This file is part of the Flair framework distributed under the
// CECILL-C License, Version 1.0.
// %flair:license}
/*!
 * \file xyFleetPD_impl.h
 * \brief Classe permettant le calcul d'un Pid
 * \author Guillaume Sanahuja, Copyright Heudiasyc UMR UTC/CNRS 7253
 * \date 2011/05/01
 * \version 4.0
 */

#ifndef XYFLEETPD_IMPL_H
#define XYFLEETPD_IMPL_H

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
class xyFleetPD;
}
}

/*! \class xyFleetPD_impl
* \brief Class defining a PID
*/

class xyFleetPD_impl {
public:
  xyFleetPD_impl(flair::filter::xyFleetPD *self, const flair::gui::LayoutPosition *position,
           std::string name);
  ~xyFleetPD_impl();
  void UseDefaultPlot(const flair::gui::LayoutPosition *position);
  void Data4Formation(const flair::gui::LayoutPosition *position);
  void UpdateFrom(const flair::core::io_data *data);
  float i;
  bool first_update;
  void OwnSat(float &signal, double saturation_value);
  float SatVal(float value, double saturation_value);

  float FormationProtocol(int id, std::vector<float> &states);
  float FormationProtocol(int id, std::vector<float> &states, std::vector<float> &reldist);

  int Form_index = 0;

private:
  flair::filter::xyFleetPD *self;

  // matrix
  flair::core::Matrix *state;

  flair::gui::DoubleSpinBox *T, *kp, *ki, *kd, *sat, *sati;
  flair::gui::DoubleSpinBox *alpha_1, *alpha_2;
  flair::gui::DoubleSpinBox *sat_u;
  flair::gui::DoubleSpinBox *sat_ec;

  flair::gui::DoubleSpinBox *FGain_01;


};

#endif // xyFleetPD_impl_H
