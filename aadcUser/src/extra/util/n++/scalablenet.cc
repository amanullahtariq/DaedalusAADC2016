
/*
n++: neural network simulator (multi-layer-perceptrons)

Copyright (c) 2013, Martin Riedmiller, University of Osnabrueck
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.
   * Neither the name of the <ORGANIZATION> nor the names of its
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
*/

/******************************************************************************/
/* N++ : C++ class neural network simulator                                   */
/* (c) 2013 Martin Riedmiller                                                 */
/* last changed: 4.12.13                                                      */
/* File: scalablenet.cc                                                       */
/* Purpose: NFQ-conform extension of a neural network                         */
/******************************************************************************/

#include "n++/scalablenet.h"

namespace NPP
{

#define MAX_LINE_LEN 1000

ScalableNet::~ScalableNet()
{
  delete_structure();
}

void ScalableNet::create_units()
{
  Net::create_units();
  for (int i=1; i<=topo_data.unit_count; i++)
    unit[i].dEdnet = (double) 0;
}

void ScalableNet::set_input_scale(int position,int scale_function, double param1,double param2,double param3)
{
  scale_typ *scale_list;

  for (scale_list = scale_list_in; scale_list != NULL; scale_list = scale_list->next) {
    if(scale_list->position == position) { // element already exists; change parameters
      scale_list->scale_function = scale_function;
      scale_list->parameter1 = param1;
      scale_list->parameter2 = param2;
      scale_list->parameter3 = param3;
      return;
    }
  } // for all scale list entries
  // at that point, no element was found; create:
  insert_scale_element(&scale_list_in, position, scale_function, param1, param2, param3);
}

ScalableNet& ScalableNet::operator= (ScalableNet &orignet)
{
  if (topo_data.unit_count > 0) {
    delete_structure(); // delete old net
  }

  int new_topo[MAX_LAYERS];

  for (int l=0; l<orignet.topo_data.layer_count; l++)
    new_topo[l] = orignet.layer[l].unit_count;
  create_layers(orignet.topo_data.layer_count, new_topo);

  for (int l=1; l<orignet.topo_data.layer_count; l++) {
    // start at first hidden layer
    for (int i=0; i<orignet.layer[l].unit_count; i++) {
      int current_id = orignet.layer[l].unit[i].unit_no;
      set_unit_act_f(current_id,
                     orignet.layer[l].unit[i].act_id);
      weight_typ *wptr;
      for (wptr=orignet.layer[l].unit[i].weights; wptr!=NULL; wptr=wptr->next)
        connect_units(current_id,wptr->from_unit,wptr->value);
    }
  }

  scale_typ *scale_list;
  for (scale_list=orignet.scale_list_in; scale_list!=NULL;
       scale_list=scale_list->next)
    insert_scale_element(&scale_list_in,scale_list->position,
                         scale_list->scale_function,
                         scale_list->parameter1,
                         scale_list->parameter2,
                         scale_list->parameter3);

  for (scale_list=orignet.scale_list_out; scale_list!=NULL;
       scale_list=scale_list->next)
    insert_scale_element(&scale_list_out,scale_list->position,
                         scale_list->scale_function,
                         scale_list->parameter1,
                         scale_list->parameter2,
                         scale_list->parameter3);

  set_update_f(orignet.update_id,orignet.update_params);

  return *this;
}

};
