/********************************************************/
/*                                                      */
/*  Copley Motion Libraries                             */
/*                                                      */
/*  Copyright (c) 2002 Copley Controls Corp.            */
/*                     http://www.copleycontrols.com    */
/*                                                      */
/********************************************************/

/***************************************************************************/
/** \file
This file contains some rules used by the Amp object to determine if certain
features are supported by the amplifier based on it's model number and firmware
version number.
*/
/***************************************************************************/

#include "CML.h"

#define VERSION( major, minor )    ((major<<8) | minor)

CML_NAMESPACE_USE();

// local functions
static int ChkFeature807( AMP_FEATURE feature, uint16 hwType, uint16 fwVer  );
static int ChkFeature8367( AMP_FEATURE feature, uint16 hwType, uint16 fwVer  );
static int ChkFeatureNios( AMP_FEATURE feature, uint16 hwType, uint16 fwVer  );
static int ChkFeatureFPGA(AMP_FEATURE feature, uint16 hwType, uint16 fwVer);
static int ChkFeatureARM(AMP_FEATURE feature, uint16 hwType, uint16 fwVer);
static int ChkFeatureArmKM(AMP_FEATURE feature, uint16 hwType, uint16 fwVer);

int Amp::CheckFeature( AMP_FEATURE feature )
{
	switch (hwType & 0xFF00)
	{
	case 0x0200:
		return ChkFeature807(feature, hwType, SwVersionNum);

	case 0x0300:
		switch (hwType & 0x000F)
		{
		case 0x0004:
			return ChkFeatureARM(feature, hwType, SwVersionNum);
		case 0x0005:
			return ChkFeatureARM(feature, hwType, SwVersionNum);
		default:
			return ChkFeature8367(feature, hwType, SwVersionNum);
		}
	case 0x0400:
		return ChkFeatureARM(feature, hwType, SwVersionNum);
    case 0x1000:
		return ChkFeatureFPGA(feature, hwType, SwVersionNum);
    case 0x1100:
		switch (hwType & 0x00FF)
		{
		case 0x00A0:
			return ChkFeatureFPGA(feature, hwType, SwVersionNum);
		case 0x0000:
			return ChkFeatureFPGA(feature, hwType, SwVersionNum);
		case 0x0010:
			return ChkFeatureFPGA(feature, hwType, SwVersionNum);
		case 0x0018:
			return ChkFeatureFPGA(feature, hwType, SwVersionNum);
		case 0x0020:
			return ChkFeatureFPGA(feature, hwType, SwVersionNum);
		case 0x0028:
			return ChkFeatureFPGA(feature, hwType, SwVersionNum);
		case 0x0030:
			return ChkFeatureFPGA(feature, hwType, SwVersionNum);
		case 0x0050:
			return ChkFeatureFPGA(feature, hwType, SwVersionNum);
		case 0x0090:
			return ChkFeatureFPGA(feature, hwType, SwVersionNum);
		case 0x00F0:
			return ChkFeatureFPGA(feature, hwType, SwVersionNum);
		case 0x0070:
			return ChkFeatureFPGA(feature, hwType, SwVersionNum);
		case 0x0078:
			return ChkFeatureFPGA(feature, hwType, SwVersionNum);
		case 0x00B0:
			return ChkFeatureFPGA(feature, hwType, SwVersionNum);
		case 0x00B8:
			return ChkFeatureFPGA(feature, hwType, SwVersionNum);
		case 0x00C0:
			return ChkFeatureFPGA(feature, hwType, SwVersionNum);
		case 0x00D0:
			return ChkFeatureFPGA(feature, hwType, SwVersionNum);
		case 0x00E0:
			return ChkFeatureFPGA(feature, hwType, SwVersionNum);
		default: 
			return ChkFeatureNios(feature, hwType, SwVersionNum);
		}
	case 0x1200:
		return ChkFeatureFPGA(feature, hwType, SwVersionNum);
	case 0x2000:
		return ChkFeatureArmKM(feature, hwType, SwVersionNum);
	case 0x2300:
		return ChkFeatureARM(feature, hwType, SwVersionNum);
   }
   return 0;
}

/*
 * Check features on amplifiers using the 807 processor.
 */
static int ChkFeature807( AMP_FEATURE feature, uint16 hwType, uint16 fwVer  )
{
   switch( feature )
   {
      case FEATURE_CAN_OPTIONS:                  return ( fwVer >= VERSION(5,34) );
      case FEATURE_VI_DRAIN:                     return ( fwVer >= VERSION(5,18) );
      case FEATURE_CAN_SETTINGS:                 return ( fwVer >= VERSION(5,02) );
      case FEATURE_CAMMING:                      return ( fwVer >= VERSION(4,85) );
      case FEATURE_EXTENDED_OUTPUT_PIN_CONFIG:   return ( fwVer >= VERSION(4,77) );
      case FEATURE_ALGO_PHASE_INIT_CONFIG:       return ( fwVer >= VERSION(4,66) );
      case FEATURE_CURRENT_SLOPE:                return ( fwVer >= VERSION(4,60) );
      case FEATURE_SOFTLIM_ACCEL:                return ( fwVer >= VERSION(4,60) );
      case FEATURE_PWMIN_FREQ:                   return ( fwVer >= VERSION(4,56) );
      case FEATURE_VLOOP_CMD_FILT:               return ( fwVer >= VERSION(4,56) );
      case FEATURE_ALGO_PHASE_INIT_CUR:          return ( fwVer >= VERSION(4,56) );
      case FEATURE_HALL_VEL_SHIFT:               return ( fwVer >= VERSION(4,56) );
      case FEATURE_GEAR_RATIO:                   return ( fwVer >= VERSION(4,42) );
      case FEATURE_RESOLVER_CYCLES:              return ( fwVer >= VERSION(4,42) );
      case FEATURE_USTEP_OUTER_LOOP:             return ( fwVer >= VERSION(4,22) );
      case FEATURE_USTEP_CONFIG_STATUS:          return ( fwVer >= VERSION(4,03) );
      case FEATURE_PIN_MAP:                      return ( fwVer >= VERSION(3,35) );
      case FEATURE_STEP_DETENT_GAIN:             return ( fwVer >= VERSION(3,35) );
      case FEATURE_PLOOP_SCALE:                  return ( fwVer >= VERSION(3,30) );
      case FEATURE_STEPPER_CRNT:                 return ( fwVer >= VERSION(3,10) );
      default:                                   return 0;
   }
}

/*
 * Check features on amplifiers using the 8367 processor.
 */
static int ChkFeature8367( AMP_FEATURE feature, uint16 hwType, uint16 fwVer  )
{
   switch( feature )
   {
      case FEATURE_VI_DRAIN:                   return ( fwVer >= VERSION(1,86) );
      case FEATURE_AIN_FILT:                   return ( fwVer >= VERSION(1,34) );
      case FEATURE_NET_OPTIONS:                return ( fwVer >= VERSION(1,34) );
      case FEATURE_GAIN_SCHED:                 return ( fwVer >= VERSION(1,10) );
      case FEATURE_ENC_OPTIONS:                return ( fwVer >= VERSION(1,03) );
      case FEATURE_CAMMING:                    return ( fwVer >= VERSION(0,55) );
      case FEATURE_POS_WRAP:                   return ( fwVer >= VERSION(0,41) );
      case FEATURE_EXTENDED_OUTPUT_PIN_CONFIG: return ( fwVer >= VERSION(0,40) );
      case FEATURE_CAN_SETTINGS:               return ( fwVer >= VERSION(0,28) );
      case FEATURE_GEAR_RATIO:                 return 1;
      case FEATURE_RESOLVER_CYCLES:            return 1;
      case FEATURE_HALL_VEL_SHIFT:             return 1;
      case FEATURE_PLOOP_SCALE:                return 1;
      case FEATURE_STEPPER_CRNT:               return 1;
      case FEATURE_CURRENT_SLOPE:              return 1;
      case FEATURE_SOFTLIM_ACCEL:              return 1;
      case FEATURE_PWMIN_FREQ:                 return 1;
      case FEATURE_VLOOP_CMD_FILT:             return 1;
      case FEATURE_USTEP_OUTER_LOOP:           return 1;
      case FEATURE_USTEP_CONFIG_STATUS:        return 1;
      case FEATURE_STEP_DETENT_GAIN:           return 1;
      case FEATURE_ALGO_PHASE_INIT_CONFIG:     return 1;
      case FEATURE_ALGO_PHASE_INIT_CUR:        return 1;
      case FEATURE_PIN_MAP:                    return 1;
      case FEATURE_AXIS_CT:                    return 1;
      default:                                 return 0;
   }
}

/*
 * Check features on amplifiers using the Nios processor.
 */
static int ChkFeatureNios( AMP_FEATURE feature, uint16 hwType, uint16 fwVer  )
{
   switch( feature )
   {
      case FEATURE_IOPULLUP32:                 return ( fwVer >= VERSION(2,22) ); // iopullup was set to read-only in firmware until 2.22
      case FEATURE_CROSS_COUPLE:               return ( fwVer >= VERSION(1,62) ); // cross coupling functionality
      case FEATURE_MTR_OPEN_CRNT:              return ( fwVer >= VERSION(1,78) ); // motor wire check current
      case FEATURE_ENC_CAL:                    return ( fwVer >= VERSION(1,54) ); // encoder calibration feature
      case FEATURE_INPUT_SHAPING:              return ( fwVer >= VERSION(1,34) ); // Inputshaping filter
      case FEATURE_PWMIN_UVCFG:                return ( fwVer >= VERSION(1,20) ); // UV Configuration when running in UV mode
      case FEATURE_PWMIN_MIN_PULSE:            return ( fwVer >= VERSION(1,16) ); // Minimum PWM pulse width
      case FEATURE_PWMIN_MAX_PULSE:            return ( fwVer >= VERSION(1,16) ); // Maximum PWM pulse width
      case FEATURE_MTR_OVERTEMP:               return ( fwVer >= VERSION(1,16) ); // Limit for analog motor temp sensor
      case FEATURE_BRAKE_ENABLE_DELAY:         return ( fwVer >= VERSION(1,02) ); // Dleay time between enabling PWM outputs and releasing the brake
      case FEATURE_IO_OPTIONS:                 return ( fwVer >= VERSION(1,02) ); // I/O Options
      case FEATURE_SERVO_CONFIG:               return ( fwVer >= VERSION(0,38) ); // Servo loop config, pos loop ki, kd, vel loop cmd feed forward
      case FEATURE_VLOOP_OUT_FILT:             return ( fwVer >= VERSION(0,38) ); // Velocity loop output filters 2 and 3
      case FEATURE_ILOOP_CMD_FILT:             return ( fwVer >= VERSION(0,38) ); // Current loop cmd filters 1 and 2
      case FEATURE_PLOOP_KI:                   return ( fwVer >= VERSION(0,38) ); // Integral gain for position loop
      case FEATURE_PLOOP_KD:                   return ( fwVer >= VERSION(0,38) ); // Derivative gain for position loop
      case FEATURE_VLOOP_CMDFF:                return ( fwVer >= VERSION(0,38) ); // Command feed forward for velocity loop
      case FEATURE_DA_CONV_CONFIG:             return ( fwVer >= VERSION(0,10) ); // DA converter config, sets the mode for the DA converter 
      case FEATURE_GAIN_SCHED:                 return ( fwVer >= VERSION(1,36) ); // gain scheduling functionality
      case FEATURE_VI_DRAIN:                   return 1;
      case FEATURE_EXTENDED_OUTPUT_PIN_CONFIG: return 1;
      case FEATURE_GEAR_RATIO:                 return 1;
      case FEATURE_RESOLVER_CYCLES:            return 1;
      case FEATURE_HALL_VEL_SHIFT:             return 1;
      case FEATURE_PLOOP_SCALE:                return 1;
      case FEATURE_STEPPER_CRNT:               return 1;
      case FEATURE_CURRENT_SLOPE:              return 1;
      case FEATURE_SOFTLIM_ACCEL:              return 1;
      case FEATURE_PWMIN_FREQ:                 return 1;
      case FEATURE_VLOOP_CMD_FILT:             return 1;
      case FEATURE_USTEP_OUTER_LOOP:           return 1;
      case FEATURE_STEP_DETENT_GAIN:           return 1;
      case FEATURE_USTEP_CONFIG_STATUS:        return 1;
      case FEATURE_ALGO_PHASE_INIT_CUR:        return 1;
      case FEATURE_ALGO_PHASE_INIT_CONFIG:     return 1;
      case FEATURE_CAMMING:                    return 1;
      case FEATURE_POS_WRAP:                   return 1;
      case FEATURE_ENC_OPTIONS:                return 1;
      case FEATURE_PIN_MAP:                    return 1;
      case FEATURE_CAN_OPTIONS:                return 1;
      case FEATURE_CAN_SETTINGS:               return 1;
      case FEATURE_NET_OPTIONS:                return 1; 
      case FEATURE_AIN_FILT:                   return 1;
      case FEATURE_FLOAT_FILT_COEF:            return 1;
      case FEATURE_AXIS_CT:                    return 1;
      case FEATURE_POS_KI_DRAIN:               return 1;

      default:                                 return 0;
   }
}

/*
* Check features on amplifiers using an FPGA.
*/
static int ChkFeatureFPGA(AMP_FEATURE feature, uint16 hwType, uint16 fwVer)
{
	switch (feature)
	{
	case FEATURE_VI_DRAIN:						return 1;
	case FEATURE_EXTENDED_OUTPUT_PIN_CONFIG:	return 1;
	case FEATURE_GEAR_RATIO:					return 1;
	case FEATURE_RESOLVER_CYCLES:				return 1;
	case FEATURE_HALL_VEL_SHIFT:				return 1;
	case FEATURE_PLOOP_SCALE:					return 1;
	case FEATURE_STEPPER_CRNT:					return 1;
	case FEATURE_CURRENT_SLOPE:					return 1;
	case FEATURE_SOFTLIM_ACCEL:					return 1;
	case FEATURE_PWMIN_FREQ:					return 1;
	case FEATURE_VLOOP_CMD_FILT:				return 1;
	case FEATURE_USTEP_OUTER_LOOP:				return 1;
	case FEATURE_STEP_DETENT_GAIN:				return 1;
	case FEATURE_USTEP_CONFIG_STATUS:			return 1;
	case FEATURE_ALGO_PHASE_INIT_CUR:			return 1;
	case FEATURE_ALGO_PHASE_INIT_CONFIG:		return 1;
	case FEATURE_CAMMING:						return 1;
	case FEATURE_POS_WRAP:						return 1;
	case FEATURE_ENC_OPTIONS:					return 1;
	case FEATURE_GAIN_SCHED:					return 1;
	case FEATURE_PIN_MAP:						return 1;
	case FEATURE_CAN_OPTIONS:					return 1;
	case FEATURE_CAN_SETTINGS:					return 1;
	case FEATURE_AIN_FILT:						return 1;
	case FEATURE_VLOOP_OUT_FILT:				return 1;
	case FEATURE_ILOOP_CMD_FILT:				return 1;
	case FEATURE_PWMIN_MIN_PULSE:				return 1;
	case FEATURE_PWMIN_MAX_PULSE:				return 1;
	case FEATURE_DA_CONV_CONFIG:				return 1;
	case FEATURE_SERVO_CONFIG:					return 1;
	case FEATURE_MTR_OVERTEMP:					return 1;
	case FEATURE_NET_OPTIONS:					return 1;
	case FEATURE_PLOOP_KI:						return 1;
	case FEATURE_PLOOP_KD:						return 1;
	case FEATURE_VLOOP_CMDFF:					return 1;
	case FEATURE_BRAKE_ENABLE_DELAY:			return 1;
	case FEATURE_INPUT_SHAPING:					return 1;
	case FEATURE_FLOAT_FILT_COEF:				return 1;
	case FEATURE_IO_OPTIONS:					return 1;
	case FEATURE_PWMIN_UVCFG:					return 1;
	case FEATURE_AXIS_CT:						return 1;
	case FEATURE_ENC_CAL:						return 1;
	case FEATURE_MTR_OPEN_CRNT:					return 1;
	case FEATURE_CROSS_COUPLE:					return 1;
	case FEATURE_POS_KI_DRAIN:					return 1;
	case FEATURE_IOPULLUP32:					return 1;
	case FEATURE_PWMIN_DEADBAND:				return 1;
	case FEATURE_ENCODER_ERROR_CONFIGURATION:   return ( fwVer >= VERSION(2,34) );
	case FEATURE_TRAJECTORY_JERK_ABORT:			return 1;
	case FEATURE_SERVO_LOOP_CONFIG:				return 1;
	
	default:									return 0;
	}
}

/*
* Check features on amplifiers using the ARM processor.
*/
static int ChkFeatureARM(AMP_FEATURE feature, uint16 hwType, uint16 fwVer)
{
	switch (feature)
	{
	case FEATURE_VI_DRAIN:						return 1;
	case FEATURE_EXTENDED_OUTPUT_PIN_CONFIG:	return 1;
	case FEATURE_GEAR_RATIO:					return 1;
	case FEATURE_RESOLVER_CYCLES:				return 1;
	case FEATURE_HALL_VEL_SHIFT:				return 1;
	case FEATURE_PLOOP_SCALE:					return 1;
	case FEATURE_STEPPER_CRNT:					return 1;
	case FEATURE_CURRENT_SLOPE:					return 1;
	case FEATURE_SOFTLIM_ACCEL:					return 1;
	case FEATURE_VLOOP_CMD_FILT:				return 1;
	case FEATURE_USTEP_OUTER_LOOP:				return 1;
	case FEATURE_STEP_DETENT_GAIN:				return 1;
	case FEATURE_USTEP_CONFIG_STATUS:			return 1;
	case FEATURE_ALGO_PHASE_INIT_CUR:			return 1;
	case FEATURE_ALGO_PHASE_INIT_CONFIG:		return 1;
	case FEATURE_CAMMING:						return 1;
	case FEATURE_POS_WRAP:						return 1;
	case FEATURE_ENC_OPTIONS:					return 1;
	case FEATURE_GAIN_SCHED:					return 1;
	case FEATURE_PIN_MAP:						return 1;
	case FEATURE_CAN_OPTIONS:					return 1;
	case FEATURE_CAN_SETTINGS:					return 1;
	case FEATURE_AIN_FILT:						return 1;
	case FEATURE_VLOOP_OUT_FILT:				return 1;
	case FEATURE_ILOOP_CMD_FILT:				return 1;
	case FEATURE_SERVO_CONFIG:					return 1;
	case FEATURE_NET_OPTIONS:					return 1;
	case FEATURE_PLOOP_KI:						return 1;
	case FEATURE_PLOOP_KD:						return 1;
	case FEATURE_VLOOP_CMDFF:					return 1;
	case FEATURE_BRAKE_ENABLE_DELAY:			return 1;
	case FEATURE_INPUT_SHAPING:					return ( fwVer >= VERSION(1,14) );
	case FEATURE_FLOAT_FILT_COEF:				return 1;
	case FEATURE_IO_OPTIONS:					return 1;
	case FEATURE_PWMIN_UVCFG:					return 1;
	case FEATURE_AXIS_CT:						return 1;
	case FEATURE_IOPULLUP32:					return ( fwVer >= VERSION(1,18) ); // Marked as read only previously
	case FEATURE_PWMIN_DEADBAND:				return 1;
	case FEATURE_ENCODER_ERROR_CONFIGURATION:   return ( fwVer >= VERSION(1,26) );
	case FEATURE_TRAJECTORY_JERK_ABORT:			return 1;
	case FEATURE_SERVO_LOOP_CONFIG:				return 1;

	default:									return 0;
	}
}
/*
* Check features on kollmorgen amplifiers using the ARM processor.
*/
static int ChkFeatureArmKM(AMP_FEATURE feature, uint16 hwType, uint16 fwVer)
{
	switch (feature)
	{
	case FEATURE_VI_DRAIN:						return 1;
	case FEATURE_EXTENDED_OUTPUT_PIN_CONFIG:	return 1;
	case FEATURE_GEAR_RATIO:					return 1;
	case FEATURE_RESOLVER_CYCLES:				return 1;
	case FEATURE_HALL_VEL_SHIFT:				return 1;
	case FEATURE_PLOOP_SCALE:					return 1;
	case FEATURE_STEPPER_CRNT:					return 1;
	case FEATURE_CURRENT_SLOPE:					return 1;
	case FEATURE_SOFTLIM_ACCEL:					return 1;
	case FEATURE_VLOOP_CMD_FILT:				return 1;
	case FEATURE_USTEP_OUTER_LOOP:				return 1;
	case FEATURE_STEP_DETENT_GAIN:				return 1;
	case FEATURE_USTEP_CONFIG_STATUS:			return 1;
	case FEATURE_ALGO_PHASE_INIT_CUR:			return 1;
	case FEATURE_ALGO_PHASE_INIT_CONFIG:		return 1;
	case FEATURE_CAMMING:						return 1;
	case FEATURE_POS_WRAP:						return 1;
	case FEATURE_ENC_OPTIONS:					return 1;
	case FEATURE_GAIN_SCHED:					return 1;
	case FEATURE_PIN_MAP:						return 1;
	case FEATURE_CAN_OPTIONS:					return 1;
	case FEATURE_CAN_SETTINGS:					return 1;
	case FEATURE_AIN_FILT:						return 1;
	case FEATURE_VLOOP_OUT_FILT:				return 1;
	case FEATURE_ILOOP_CMD_FILT:				return 1;
	case FEATURE_SERVO_CONFIG:					return 1;
	case FEATURE_NET_OPTIONS:					return 1;
	case FEATURE_PLOOP_KI:						return 1;
	case FEATURE_PLOOP_KD:						return 1;
	case FEATURE_VLOOP_CMDFF:					return 1;
	case FEATURE_BRAKE_ENABLE_DELAY:			return 1;
	case FEATURE_INPUT_SHAPING:					return ( fwVer >= VERSION(1,14) );
	case FEATURE_FLOAT_FILT_COEF:				return 1;
	case FEATURE_IO_OPTIONS:					return 1;
	case FEATURE_PWMIN_UVCFG:					return 1;
	case FEATURE_AXIS_CT:						return 1;
	case FEATURE_IOPULLUP32:					return ( fwVer >= VERSION(1,18) ); // Marked as read only previously
	case FEATURE_PWMIN_DEADBAND:				return 1;
	case FEATURE_SPECIAL_KM_DWELL_TIME:			return 1;
	case FEATURE_SPECIAL_KM_FREQUENCY:			return 1;
	case FEATURE_SPECIAL_KM_MAX_TEMP:			return 1;
	case FEATURE_SPECIAL_KM_MAX_TEMP_2:			return 1;
	case FEATURE_SPECIAL_KM_SOLE_CONT_VOLTAGE:  return ( fwVer >= VERSION(1,21) );
	case FEATURE_SPECIAL_KM_SOLE_INIT_VOLTAGE:  return ( fwVer >= VERSION(1,21) );
	case FEATURE_ENCODER_ERROR_CONFIGURATION:   return ( fwVer >= VERSION(1,26) );
	case FEATURE_TRAJECTORY_JERK_ABORT:			return 1;
	case FEATURE_SERVO_LOOP_CONFIG:				return 1;

	default:									return 0;
	}
}
