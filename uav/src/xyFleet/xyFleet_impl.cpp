// %flair:license{
// This file is part of the Flair framework distributed under the
// CECILL-C License, Version 1.0.
// %flair:license}
//  created:    2011/05/01
//  filename:   xyFleet_impl.cpp
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
#include "xyFleet_impl.h"
#include "xyFleet.h"
#include <Matrix.h>
#include <Layout.h>
#include <GroupBox.h>
#include <DoubleSpinBox.h>
#include <DataPlot1D.h>
#include <iostream>
#include <vector>

using namespace std;

using std::string;
using namespace flair::core;
using namespace flair::gui;
using namespace flair::filter;

xyFleet_impl::xyFleet_impl(xyFleet *self, const LayoutPosition *position, string name) {
  i = 0;
  first_update = true;
  this->self = self;

  // init matrix
  self->input = new Matrix(self, 6, 1, floatType, name);

  MatrixDescriptor *desc = new MatrixDescriptor(4, 1);
  desc->SetElementName(0, 0, "e1");
  desc->SetElementName(1, 0, "ctrl2virtual");
  desc->SetElementName(2, 0, "e2");
  desc->SetElementName(3, 0, "Ctrl");
  state = new Matrix(self, desc, floatType, name);
  delete desc;

  GroupBox *reglages_groupbox = new GroupBox(position, name);
  T = new DoubleSpinBox(reglages_groupbox->NewRow(), "period, 0 for auto", " s",0, 1, 0.01);
  alpha_1 = new DoubleSpinBox(reglages_groupbox->NewRow(), "not in use:", 0, 90000000, 0.01);
  //ki = new DoubleSpinBox(reglages_groupbox->NewRow(), "ki:", 0, 90000000, 0.01,3);
  //sati = new DoubleSpinBox(reglages_groupbox->LastRowLastCol(), "sat i:", 0, 1,0.01);
  alpha_2 = new DoubleSpinBox(reglages_groupbox->NewRow(), "kd:", -10, 90000000, 0.001);
  FGain_01 = new DoubleSpinBox(reglages_groupbox->NewRow(), "FG_01: ", -10, 10, 0.001);
  sat_u = new DoubleSpinBox(reglages_groupbox->NewRow(), "sat_u:", 0, 1, 0.1);
  sat_ec = new DoubleSpinBox(reglages_groupbox->NewRow(), "sat_ec:", 0, 1, 0.1);
}

xyFleet_impl::~xyFleet_impl(void) {
}

void xyFleet_impl::UseDefaultPlot(const LayoutPosition *position) {
  DataPlot1D *plot = new DataPlot1D(position, self->ObjectName(), -1, 1);
  plot->AddCurve(state->Element(0));
  plot->AddCurve(state->Element(1), DataPlot::Green);
  plot->AddCurve(state->Element(2), DataPlot::Blue);
  plot->AddCurve(state->Element(3), DataPlot::Black);

}

void xyFleet_impl::Data4Formation(const LayoutPosition *position){

}

void xyFleet_impl::UpdateFrom(const io_data *data) {

  const Matrix* input = dynamic_cast<const Matrix*>(data);
  
  if (!input) {
      self->Warn("casting %s to Matrix failed\n",data->ObjectName().c_str());
      return;
  }

  input->GetMutex();

  int id = (int)(input->ValueNoMutex(0,0));
  vector<float> states = {input->ValueNoMutex(1,0),
                          input->ValueNoMutex(2,0),
                          input->ValueNoMutex(3,0)};
  float vel = input->ValueNoMutex(4,0);
  float ref_vel = input->ValueNoMutex(5,0);

  float e_vel = vel - ref_vel;

  float u_total = FormationProtocol(id,states) + alpha_2->Value()*e_vel;

  OwnSat(u_total,sat_u->Value() );

  
  state->GetMutex();
  state->SetValueNoMutex(0, 0, states[0]);
  state->SetValueNoMutex(1, 0, states[1]);
  state->SetValueNoMutex(2, 0, states[2]);
  state->SetValueNoMutex(3, 0, u_total);
  state->ReleaseMutex();

  self->output->SetValue(0, 0, u_total);
  self->output->SetDataTime(data->DataTime());
}


void xyFleet_impl::OwnSat(float& signal, double saturation_value){
  if (signal > saturation_value){
    signal = saturation_value;
  }
  else if(signal < -saturation_value){
    signal -saturation_value;
  }
  else
    signal =  signal;
}

float xyFleet_impl::SatVal(float value, double saturation_value){
  if (value > saturation_value){
    return saturation_value;
  }
  else if(value < -saturation_value){
    return -saturation_value;
  }
  else
    return  value;
}

float xyFleet_impl::FormationProtocol(int id, std::vector<float> &states){
  float u = 0.0;
  for(float ag : states){
    u += SatVal(states[id] - ag,sat_ec->Value());
  }
  return -1*FGain_01->Value()*u;
}