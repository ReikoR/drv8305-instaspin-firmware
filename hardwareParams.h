#ifndef _HARDWAREPARAMS_H_
#define _HARDWAREPARAMS_H_

#include "modules/motor/motor.h"

typedef struct _HW_Params_ {
  MOTOR_Type_e    motor_type;                   //!< Defines the motor type
  uint_least16_t  motor_numPolePairs;           //!< Defines the number of pole pairs for the motor

  float_t       motor_ratedFlux;              //!< Defines the rated flux of the motor, V/Hz
  float_t       motor_Rr;                     //!< Defines the rotor resistance, ohm
  float_t       motor_Rs;                     //!< Defines the stator resistance, ohm
  float_t       motor_Ls_d;                   //!< Defines the direct stator inductance, H
  float_t       motor_Ls_q;                   //!< Defines the quadrature stator inductance, H
  float_t       maxCurrent;                   //!< Defines the maximum current value, A
  float_t       maxCurrent_resEst;            //!< Defines the maximum current value for resistance estimation, A
  float_t       maxCurrent_indEst;            //!< Defines the maximum current value for inductance estimation, A
  float_t       fluxEstFreq_Hz;               //!< Defines the flux estimation frequency, Hz
} HW_Params;

#endif
