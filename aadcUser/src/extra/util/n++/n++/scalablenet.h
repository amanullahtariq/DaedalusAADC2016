
/******************************************************************************/
/* N++ : C++ class neural network simulator                                   */
/* (c) 1994, 2013 Martin Riedmiller                                           */
/* last changed: 4.12.13                                                      */
/* File: n++.h                                                                */
/* Purpose: extended network with functionality from libNFQ programs          */
/******************************************************************************/

#include "n++/n++.h"

#ifndef _NPP_SCALABLE_H_
#define _NPP_SCALABLE_H_

namespace NPP
{

/** Network with libNFQ-compliant functionality.
  * Allows scaling of inputs and learning only the last layer. */
class ScalableNet : public Net
{
 protected:
  bool last_layer_only;
 public:
  ScalableNet() : Net() {};
  ~ScalableNet();
  ScalableNet& operator= (ScalableNet& orignet);
  void set_input_scale(int position, int scale_function, FTYPE param1, FTYPE param2, FTYPE param3);
  void create_units();
};

};

#endif
