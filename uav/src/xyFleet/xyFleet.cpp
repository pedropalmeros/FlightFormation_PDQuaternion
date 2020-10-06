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

#include "xyFleet.h"
#include "xyFleet_impl.h"
#include <Matrix.h>
#include <Layout.h>
#include <LayoutPosition.h>

using std::string;
using namespace flair::core;
using namespace flair::gui;

namespace flair {
namespace filter {

xyFleet::xyFleet(const LayoutPosition *position, string name)
    : ControlLaw(position->getLayout(), name) {
  pimpl_ = new xyFleet_impl(this, position, name);
  
  SetIsReady(true);
}

xyFleet::~xyFleet(void) { delete pimpl_; }

void xyFleet::UseDefaultPlot(const gui::LayoutPosition *position) {
  pimpl_->UseDefaultPlot(position);
}

void xyFleet::Data4Formation(const gui::LayoutPosition *position){
  pimpl_->Data4Formation(position);
}

void xyFleet::Reset(void) {
  pimpl_->i = 0;
  pimpl_->first_update = true;
}

float xyFleet::GetIntegral(void) const { return pimpl_->i; }

void xyFleet::UpdateFrom(const io_data *data) {
  pimpl_->UpdateFrom(data);
  ProcessUpdate(output);
}

void xyFleet::SetValues(float id, float a0, float a1, float a2, float vel, float ref_vel) {
  input->SetValue(0, 0, id);
  input->SetValue(1, 0, a0);
  input->SetValue(2, 0, a1);
  input->SetValue(3, 0, a2);
  input->SetValue(4, 0, vel);
  input->SetValue(5, 0, ref_vel);
}

} // end namespace filter
} // end namespace flair
