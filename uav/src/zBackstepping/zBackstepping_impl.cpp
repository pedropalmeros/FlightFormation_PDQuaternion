// %flair:license{
// This file is part of the Flair framework distributed under the
// CECILL-C License, Version 1.0.
// %flair:license}
//  created:    2014/11/07
//  filename:   zBackstepping_impl.cpp
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
#include "zBackstepping_impl.h"
#include "zBackstepping.h"
#include <Matrix.h>
#include <Layout.h>
#include <GroupBox.h>
#include <DoubleSpinBox.h>
#include <DataPlot1D.h>

using std::string;
using namespace flair::core;
using namespace flair::gui;
using namespace flair::filter;

zBackstepping_impl::zBackstepping_impl(zBackstepping *self, const LayoutPosition *position,
                               string name) {
  i = 0;
  offset_g = 0;
  first_update = true;
  this->self = self;

  // init matrix
  self->input = new Matrix(self, 4, 1, floatType, name);

  MatrixDescriptor *desc = new MatrixDescriptor(5, 1);
  desc->SetElementName(0, 0, "e1");
  desc->SetElementName(1, 0, "ctrl2virtual");
  desc->SetElementName(2, 0, "e2");
  desc->SetElementName(3, 0, "u+i+d");
  desc->SetElementName(4, 0, "u+i+d+offset");
  state = new Matrix(self, desc, floatType, name);
  delete desc;

  GroupBox *reglages_groupbox = new GroupBox(position, name);
  T = new DoubleSpinBox(reglages_groupbox->NewRow(), "period, 0 for auto", " s",0, 1, 0.01);
  //kp = new DoubleSpinBox(reglages_groupbox->NewRow(), "kp:", 0, 90000000, 0.01);
  //ki = new DoubleSpinBox(reglages_groupbox->NewRow(), "ki:", 0, 90000000, 0.01);
  //sati = new DoubleSpinBox(reglages_groupbox->LastRowLastCol(), "sat i:", 0, 1,0.01);
  //kd = new DoubleSpinBox(reglages_groupbox->NewRow(), "kd:", 0, 90000000, 0.01);
  offset = new DoubleSpinBox(reglages_groupbox->LastRowLastCol(), "offset g:",0, 1, 0.01);
  sat = new DoubleSpinBox(reglages_groupbox->NewRow(), "sat:", 0, 1, 0.1);
  pas_offset = new DoubleSpinBox(reglages_groupbox->LastRowLastCol(),"offset step:", 0, 1, .0001, 4);

  alpha_1 = new DoubleSpinBox(reglages_groupbox->NewRow(), "alpha_1:", 0, 90000000, 0.01);
  alpha_2 = new DoubleSpinBox(reglages_groupbox->NewRow(), "alpha_2:", 0, 90000000, 0.01);
  sat_u = new DoubleSpinBox(reglages_groupbox->NewRow(), "sat_u:", 0, 90000000, 0.01);

}

zBackstepping_impl::~zBackstepping_impl(void) {}

void zBackstepping_impl::UseDefaultPlot(const LayoutPosition *position) {
  DataPlot1D *plot = new DataPlot1D(position, self->ObjectName(), -1, 1);
  plot->AddCurve(state->Element(0));
  plot->AddCurve(state->Element(1), DataPlot::Green);
  plot->AddCurve(state->Element(2), DataPlot::Blue);
  plot->AddCurve(state->Element(3), DataPlot::Black);
  plot->AddCurve(state->Element(4), DataPlot::Yellow);
}

void zBackstepping_impl::UpdateFrom(const io_data *data) {
  float p, d, total;
  float delta_t;
  const Matrix* input = dynamic_cast<const Matrix*>(data);
  
  if (!input) {
      self->Warn("casting %s to Matrix failed\n",data->ObjectName().c_str());
      return;
  }

  if (T->Value() == 0) {
    delta_t = (float)(data->DataDeltaTime()) / 1000000000.;
  } else {
    delta_t = T->Value();
  }
  if (first_update == true) {
    delta_t = 0;
    first_update = false;
  }

  input->GetMutex();
  float position = input->ValueNoMutex(0,0);
  float ref_position = input->ValueNoMutex(1,0);
  float velocity = input->ValueNoMutex(2,0);
  float ref_velocity = input->ValueNoMutex(3,0);

  //float error = position - ref_position;
  //float dot_error = velocity - ref_velocity;

  float e1 = position - ref_position;

  float ctrl2virtual = -alpha_1->Value()*e1 + ref_velocity;

  float e2 = velocity - ctrl2virtual;

  float u = - alpha_1->Value()*(e2 - alpha_1->Value()*e1) - alpha_2->Value()*e2  - e1;


  OwnSat(u,sat_u->Value());

  /*
  p = kp->Value() * error;
  i += ki->Value() * error * delta_t;
  if (i > sati->Value())
    i = sati->Value();
  if (i < -sati->Value())
    i = -sati->Value();
  d = kd->Value() * dot_error;*/
  input->ReleaseMutex();
  /*
  total = p + i + d;
  if (total > sat->Value())
    total = sat->Value();
  if (total < -sat->Value())
    total = -sat->Value();
  */
  total = u;

  state->GetMutex();
  state->SetValueNoMutex(0, 0, e1);
  state->SetValueNoMutex(1, 0, ctrl2virtual);
  state->SetValueNoMutex(2, 0, e2);
  state->SetValueNoMutex(3, 0, total);
  state->SetValueNoMutex(4, 0, total - offset_g * offset_g);
  state->ReleaseMutex();

  //-offset_g, car on met -u_z dans le multiplex
  // a revoir!
  self->output->SetValue(0, 0, total - offset_g * offset_g);
  self->output->SetDataTime(data->DataTime());
}


void zBackstepping_impl::OwnSat(float &signal, float saturation_value){
  if (signal>saturation_value)
    signal = saturation_value;
  if(signal<-saturation_value)
    signal = -saturation_value;
}
