// %flair:license{
// This file is part of the Flair framework distributed under the
// CECILL-C License, Version 1.0.
// %flair:license}
//  created:    2011/05/01
//  filename:   Pid.cpp
//
//  author:     Guillaume Sanahuja
//              Copyright Heudiasyc UMR UTC/CNRS 7253
//
//  version:    $Id: $
//
//  purpose:    Class defining a PID
//
//
/*********************************************************************/

#include "xyFleetPD.h"
#include "xyFleetPD_impl.h"
#include <Matrix.h>
#include <Layout.h>
#include <LayoutPosition.h>
#include <iostream>

using std::cout;
using std::endl;
using std::string;
using namespace flair::core;
using namespace flair::gui;

namespace flair {
namespace filter {

xyFleetPD::xyFleetPD(const LayoutPosition *position, string name)
    : ControlLaw(position->getLayout(), name) {
  pimpl_ = new xyFleetPD_impl(this, position, name);
  
  SetIsReady(true);
}

xyFleetPD::~xyFleetPD(void) { delete pimpl_; }

void xyFleetPD::UseDefaultPlot(const gui::LayoutPosition *position) {
  pimpl_->UseDefaultPlot(position);
}

void xyFleetPD::Data4Formation(const gui::LayoutPosition *position){
  pimpl_->Data4Formation(position);
}

void xyFleetPD::Reset(void) {
  pimpl_->i = 0;
  pimpl_->first_update = true;
}

float xyFleetPD::GetIntegral(void) const { return pimpl_->i; }

void xyFleetPD::UpdateFrom(const io_data *data) {
  pimpl_->UpdateFrom(data);
  ProcessUpdate(output);
}


void xyFleetPD::SetValues(float id, std::vector<float> States, std::vector<float> RelDist, 
                          float vel, float ref_vel) {
  input->SetValue(0, 0, id);
  input->SetValue(1, 0,States[0]);
  input->SetValue(2, 0,States[1]);
  input->SetValue(3, 0,States[2]);
  input->SetValue(4, 0,RelDist[0]);
  input->SetValue(5, 0,RelDist[1]);
  input->SetValue(6, 0,RelDist[2]);
  input->SetValue(7, 0, vel);
  input->SetValue(8, 0, ref_vel);
}

} // end namespace filter
} // end namespace flair
