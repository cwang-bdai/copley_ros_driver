/********************************************************/
/*                                                      */
/*  Copley Motion Libraries                             */
/*                                                      */
/*  Copyright (c) 2010 Copley Controls Corp.            */
/*                     http://www.copleycontrols.com    */
/*                                                      */
/********************************************************/

/** \file 

  Standard CANopen I/O module support.

*/

#ifndef _DEF_INC_COPLEYIO
#define _DEF_INC_COPLEYIO

#include "CML_Settings.h"
#include "CML_Node.h"
#include "CML_PDO.h"
#include "CML_IO.h"
#include "CML_Error.h"

CML_NAMESPACE_START()

#define COPLEYIO_MAX_STRING    41
#define COPLEYIO_DIO_BANKS     10
#define COPLEYIO_NUM_AIN       12
#define COPLEYIO_NUM_AOUT      2    // EtherCAT IO Module only.
#define COPLEYIO_NUM_PWM       12
#define COPLEYIO_CONFIG_WORDS  16
#define COPLEYIO_FILTER_32BIT_WORDS  7

/***************************************************************************/
/**
  Object dictionary ID values used on Copley I/O modules.
  */
/***************************************************************************/
enum CIO_OBJID
{
   CIOOBJID_INFO_SERIAL             = 0x3000,  ///<  Serial number
   CIOOBJID_INFO_MODEL              = 0x3001,  ///<  Model number string
   CIOOBJID_INFO_MFGINFO            = 0x3002,  ///<  Amplifier's manufacturing information string
   CIOOBJID_INFO_HWTYPE             = 0x3003,  ///<  Hardware type code
   CIOOBJID_INFO_LOOPRATE           = 0x3004,  ///<  Main loop update rate (Hz)
   CIOOBJID_INFO_TIMEPWRUP          = 0x3005,  ///<  Time since power up in ms
   CIOOBJID_INFO_NTWKOPTN           = 0x3006,  ///<  Network Options Parameter
   CIOOBJID_INFO_ECATALIAS          = 0x3007,  ///<  EtherCAT Alias for use at power-up (if bit 0 of Network Options Parameter is set)
   CIOOBJID_INFO_FWVERSION          = 0x3010,  ///<  Firmware version number
   CIOOBJID_INFO_BAUD               = 0x3011,  ///<  Serial port baud rate (bps)
   CIOOBJID_INFO_MAXWORDS           = 0x3012,  ///<  Maximum number of words sent with any command
   CIOOBJID_INFO_NAME               = 0x3013,  ///<  I/O module name
   CIOOBJID_INFO_HOSTCFG            = 0x3014,  ///<  Host configuration state (CME use only)
   CIOOBJID_INFO_NODECFG            = 0x3015,  ///<  CAN node ID configuration
   CIOOBJID_INFO_RATECFG            = 0x3016,  ///<  CAN bit rate configuration
   CIOOBJID_INFO_NODEID             = 0x3017,  ///<  CAN node ID
   CIOOBJID_INFO_STATUS             = 0x3018,  ///<  CAN network status word
   CIOOBJID_INFO_RATE               = 0x3019,  ///<  CAN network bit rate
   CIOOBJID_INFO_ANLGINT            = 0x301A,  ///<  Active analog interrupts
   CIOOBJID_INFO_ANLGINTENA         = 0x301B,  ///<  Analog input global interrupt enable
   CIOOBJID_INFO_DIGIINTENA         = 0x301C,  ///<  Digital input global interrupt enable
   CIOOBJID_INFO_PWMPERIODA         = 0x301E,  ///<  PWM bank A period
   CIOOBJID_INFO_PWMPERIODB         = 0x301F,  ///<  PWM bank B period

   CIOOBJID_DIGI_BANKMODE           = 0x3020,  ///<  Digital I/O bank mode
   CIOOBJID_DIGI_PULLUPMSK          = 0x3021,  ///<  Digital I/O pull-up resistor mask
   CIOOBJID_DIGI_TYPEMSK            = 0x3022,  ///<  Digital I/O output type mask
   CIOOBJID_DIGI_FAULTMSK           = 0x3023,  ///<  Digital I/O output fault state mask
   CIOOBJID_DIGI_INVMSK             = 0x3024,  ///<  Digital I/O inversion mask
   CIOOBJID_DIGI_VALUEMSK           = 0x3025,  ///<  Digital I/O data value mask
   CIOOBJID_DIGI_MODEMSK            = 0x3026,  ///<  Digital I/O output fault mode mask
   CIOOBJID_DIGI_RAWMSK             = 0x3027,  ///<  Digital I/O raw data value mask
   CIOOBJID_DIGI_HILOMSK            = 0x3028,  ///<  Digital I/O input low->high interrupt mask
   CIOOBJID_DIGI_LOHIMSK            = 0x3029,  ///<  Digital I/O input high->low interrupt mask
   CIOOBJID_DIGI_PLDWNRMASK         = 0x302A,  ///<  Digital I/O pull-down resistor mask
   CIOOBJID_DIGI_DEBOUNCE0          = 0x3030,  ///<  Digital I/O debounce time, bit 0
   CIOOBJID_DIGI_DEBOUNCE1          = 0x3031,  ///<  Digital I/O debounce time, bit 1
   CIOOBJID_DIGI_DEBOUNCE2          = 0x3032,  ///<  Digital I/O debounce time, bit 2
   CIOOBJID_DIGI_DEBOUNCE3          = 0x3033,  ///<  Digital I/O debounce time, bit 3
   CIOOBJID_DIGI_DEBOUNCE4          = 0x3034,  ///<  Digital I/O debounce time, bit 4
   CIOOBJID_DIGI_DEBOUNCE5          = 0x3035,  ///<  Digital I/O debounce time, bit 5
   CIOOBJID_DIGI_DEBOUNCE6          = 0x3036,  ///<  Digital I/O debounce time, bit 6
   CIOOBJID_DIGI_DEBOUNCE7          = 0x3037,  ///<  Digital I/O debounce time, bit 7

   CIOOBJID_ANLG_IRAW               = 0x3040,  ///<  Analog input raw value
   CIOOBJID_ANLG_ISCALED            = 0x3041,  ///<  Analog input scaled value
   CIOOBJID_ANLG_IFACTOR            = 0x3042,  ///<  Analog input scaling factor
   CIOOBJID_ANLG_IOFFSET            = 0x3043,  ///<  Analog input offset
   CIOOBJID_ANLG_IUPLIMIT           = 0x3044,  ///<  Analog input upper limit for interrupt
   CIOOBJID_ANLG_ILOLIMIT           = 0x3045,  ///<  Analog input lower limit for interrupt
   CIOOBJID_ANLG_IABSDELTA          = 0x3046,  ///<  Analog input absolute delta value for interrrupt
   CIOOBJID_ANLG_IPOSDELTA          = 0x3047,  ///<  Analog input positive delta value for interrrupt
   CIOOBJID_ANLG_INEGDELTA          = 0x3048,  ///<  Analog input negative delta value for interrrupt
   CIOOBJID_ANLG_IFLAGS             = 0x3049,  ///<  Analog input interrrupt flags
   CIOOBJID_ANLG_IMASK              = 0x304A,  ///<  Analog input interrrupt mask
   
   CIOOBJID_ANLG_IFILTER_A0           = 0x304B,  ///<  Programmable filter for analog input A0
   CIOOBJID_ANLG_IFILTER_A1           = 0x314B,  ///<  Programmable filter for analog input A1
   CIOOBJID_ANLG_IFILTER_A2           = 0x324B,  ///<  Programmable filter for analog input A2
   CIOOBJID_ANLG_IFILTER_A3           = 0x334B,  ///<  Programmable filter for analog input A3
   CIOOBJID_ANLG_IFILTER_A4           = 0x344B,  ///<  Programmable filter for analog input A4
   CIOOBJID_ANLG_IFILTER_A5           = 0x354B,  ///<  Programmable filter for analog input A5
   CIOOBJID_ANLG_IFILTER_B0           = 0x364B,  ///<  Programmable filter for analog input B0
   CIOOBJID_ANLG_IFILTER_B1           = 0x374B,  ///<  Programmable filter for analog input B1
   CIOOBJID_ANLG_IFILTER_B2           = 0x384B,  ///<  Programmable filter for analog input B2
   CIOOBJID_ANLG_IFILTER_B3           = 0x394B,  ///<  Programmable filter for analog input B3
   CIOOBJID_ANLG_IFILTER_B4           = 0x3A4B,  ///<  Programmable filter for analog input B4
   CIOOBJID_ANLG_IFILTER_B5           = 0x3B4B,  ///<  Programmable filter for analog input B5


   CIOOBJID_PWM_ORAW                = 0x3050,  ///<  PWM output raw value
   CIOOBJID_PWM_OSCALED             = 0x3051,  ///<  PWM output scaled value
   CIOOBJID_PWM_OFACTOR             = 0x3052,  ///<  PWM output scaling factor
   CIOOBJID_PWM_OOFFSET             = 0x3053,  ///<  PWM output offset
   
   CIOOBJID_DA_OUTRAWVAL            = 0x3058,   ///<  Raw D/A output value. (EtherCAT only)
   CIOOBJID_DA_OUTSCLVAL            = 0x3059,   ///<  DAC output scaled value. (EtherCAT only)
   CIOOBJID_DA_OUTSCLPARAM          = 0x305A,   ///<  DAC scaling parameter. (EtherCAT only)
   CIOOBJID_DA_OFFSET               = 0x305B,   ///<  DAC offset parameter. (EtherCAT only)
   CIOOBJID_DA_FLTMODE              = 0x305C,   ///<  DAC fault mode. (EtherCAT only)
   CIOOBJID_DA_FLTVAL               = 0x305D,   ///<  DAC fault value. (EtherCAT only)

   CIOOBJID_CONFIG_PARAM0           = 0x3060,   ///<  Digital I/O bank configuration parameter 0 for special modes of operation
   CIOOBJID_CONFIG_PARAM1           = 0x3061,   ///<  Digital I/O bank configuration parameter 1 for special modes of operation
   CIOOBJID_CONFIG_PARAM2           = 0x3062,   ///<  Digital I/O bank configuration parameter 2 for special modes of operation
   CIOOBJID_CONFIG_PARAM3           = 0x3063,   ///<  Digital I/O bank configuration parameter 3 for special modes of operation
   CIOOBJID_CONFIG_PARAM4           = 0x3064,   ///<  Digital I/O bank configuration parameter 4 for special modes of operation
   CIOOBJID_CONFIG_PARAM5           = 0x3065,   ///<  Digital I/O bank configuration parameter 5 for special modes of operation
   CIOOBJID_CONFIG_PARAM6           = 0x3066,   ///<  Digital I/O bank configuration parameter 6 for special modes of operation
   CIOOBJID_CONFIG_PARAM7           = 0x3067,   ///<  Digital I/O bank configuration parameter 7 for special modes of operation
   CIOOBJID_CONFIG_PARAM8           = 0x3068,   ///<  Digital I/O bank configuration parameter 8 for special modes of operation
   CIOOBJID_CONFIG_PARAM9           = 0x3069    ///<  Digital I/O bank configuration parameter 9 for special modes of operation
};

/***************************************************************************/
/**
  IO Module characteristics data structure.

  This structure is used to hold information about the IO Module such as it's
  model number, serial number, etc.  

  Use the IOModule::GetIOInfo method to retrieve this information from the module.
*/
/***************************************************************************/
struct CopleyIOInfo
{
   uint32 serial;                                   ///< Serial number
   char model[ COPLEYIO_MAX_STRING ];               ///< Model number string
   char mfgInfo[ COPLEYIO_MAX_STRING ];             ///< Amplifier's manufacturing information string
   uint16 hwType;                                   ///< Hardware type code
   uint16 loopRate;                                 ///< Main loop update rate (Hz)
   uint32 timePwrUp;                                ///< Time since power up in milliseconds
   uint16 networkOptions;                           ///< Network Options Paramter
   uint16 ecatAlias;                                ///< EtherCAT Node Alias for use at power-up (if bit 0 of Network Options Parameter is set)
   uint16 fwVersion;                                ///< Firmware version number
   uint32 baud;                                     ///< Serial port baud rate (bps)
   uint16 maxWords;                                 ///< Maximum number of words sent with any command
   char name[ COPLEYIO_MAX_STRING ];                ///< I/O module name
   char hostCfg[ COPLEYIO_MAX_STRING ];             ///< Host configuration state (CME use only)

   int16 nodeCfg;                                   ///< CAN node ID configuration
   uint16 rateCfg;                                  ///< CAN bit rate configuration
   uint16 nodeID;                                   ///< CAN node ID
   uint16 status;                                   ///< CAN network status word
   uint16 rate;                                     ///< CAN network bit rate

   uint16 anlgInt;                                  ///< Active analog interrupts
   uint16 anlgIntEna;                               ///< Analog input global interrupt enable
   uint16 digiIntEna;                               ///< Digital input global interrupt enable

   uint32 pwmPeriodA;                               ///< PWM bank A period
   uint32 pwmPeriodB;                               ///< PWM bank B period

};

/***************************************************************************/
/**
   This structure is used to return information about the digital I/O 
   of a Copley I/O module.
  */
/***************************************************************************/
struct CopleyIODigi
{
   uint16 bankMode[ COPLEYIO_DIO_BANKS ];           ///< Digital I/O bank mode
   uint16 pullupMsk[ COPLEYIO_DIO_BANKS ];          ///< Digital I/O pull-up resistor mask
   uint16 typeMsk[ COPLEYIO_DIO_BANKS ];            ///< Digital I/O output type mask
   uint16 faultMsk[ COPLEYIO_DIO_BANKS ];           ///< Digital I/O output fault state mask
   uint16 invMsk[ COPLEYIO_DIO_BANKS ];             ///< Digital I/O inversion mask
   uint16 valueMsk[ COPLEYIO_DIO_BANKS ];           ///< Digital I/O data value mask
   uint16 modeMsk[ COPLEYIO_DIO_BANKS ];            ///< Digital I/O output fault mode mask
   uint16 rawMsk[ COPLEYIO_DIO_BANKS ];             ///< Digital I/O raw data value mask
   uint16 hiLoMsk[ COPLEYIO_DIO_BANKS ];            ///< Digital I/O input low->high interrupt mask
   uint16 loHiMsk[ COPLEYIO_DIO_BANKS ];            ///< Digital I/O input high->low interrupt mask
   uint16 pullDwnResMask[ COPLEYIO_DIO_BANKS ];     ///< Digital I/O pull-down resistor mask
   uint16 debounce0[ COPLEYIO_DIO_BANKS ];          ///< Digital I/O debounce time, bit 0
   uint16 debounce1[ COPLEYIO_DIO_BANKS ];          ///< Digital I/O debounce time, bit 1
   uint16 debounce2[ COPLEYIO_DIO_BANKS ];          ///< Digital I/O debounce time, bit 2
   uint16 debounce3[ COPLEYIO_DIO_BANKS ];          ///< Digital I/O debounce time, bit 3
   uint16 debounce4[ COPLEYIO_DIO_BANKS ];          ///< Digital I/O debounce time, bit 4
   uint16 debounce5[ COPLEYIO_DIO_BANKS ];          ///< Digital I/O debounce time, bit 5
   uint16 debounce6[ COPLEYIO_DIO_BANKS ];          ///< Digital I/O debounce time, bit 6
   uint16 debounce7[ COPLEYIO_DIO_BANKS ];          ///< Digital I/O debounce time, bit 7
   uint16 bankConfigParam0[COPLEYIO_CONFIG_WORDS];     ///< Digital I/O bank configuration parameter 0 for special modes of operation
   uint16 bankConfigParam1[COPLEYIO_CONFIG_WORDS];     ///< Digital I/O bank configuration parameter 1 for special modes of operation
   uint16 bankConfigParam2[COPLEYIO_CONFIG_WORDS];     ///< Digital I/O bank configuration parameter 2 for special modes of operation
   uint16 bankConfigParam3[COPLEYIO_CONFIG_WORDS];     ///< Digital I/O bank configuration parameter 3 for special modes of operation
   uint16 bankConfigParam4[COPLEYIO_CONFIG_WORDS];     ///< Digital I/O bank configuration parameter 4 for special modes of operation
   uint16 bankConfigParam5[COPLEYIO_CONFIG_WORDS];     ///< Digital I/O bank configuration parameter 5 for special modes of operation
   uint16 bankConfigParam6[COPLEYIO_CONFIG_WORDS];     ///< Digital I/O bank configuration parameter 6 for special modes of operation
   uint16 bankConfigParam7[COPLEYIO_CONFIG_WORDS];     ///< Digital I/O bank configuration parameter 7 for special modes of operation
   uint16 bankConfigParam8[COPLEYIO_CONFIG_WORDS];     ///< Digital I/O bank configuration parameter 8 for special modes of operation
   uint16 bankConfigParam9[COPLEYIO_CONFIG_WORDS];     ///< Digital I/O bank configuration parameter 9 for special modes of operation
};

/***************************************************************************/
/**
   This structure is used to return information about the analog inputs
   of a Copley I/O module.
  */
/***************************************************************************/
struct CopleyIOAnlg
{
   uint16 iRaw[ COPLEYIO_NUM_AIN ];                 ///< Analog input raw value
   uint32 iScaled[ COPLEYIO_NUM_AIN ];              ///< Analog input scaled value
   uint32 iFactor[ COPLEYIO_NUM_AIN ];              ///< Analog input scaling factor
   uint32 iOffset[ COPLEYIO_NUM_AIN ];              ///< Analog input offset
   uint32 iUpLimit[ COPLEYIO_NUM_AIN ];             ///< Analog input upper limit for interrupt
   uint32 iLoLimit[ COPLEYIO_NUM_AIN ];             ///< Analog input lower limit for interrupt
   uint32 iAbsDelta[ COPLEYIO_NUM_AIN ];            ///< Analog input absolute delta value for interrrupt
   uint32 iPosDelta[ COPLEYIO_NUM_AIN ];            ///< Analog input positive delta value for interrrupt
   uint32 iNegDelta[ COPLEYIO_NUM_AIN ];            ///< Analog input negative delta value for interrrupt
   uint16 iFlags[ COPLEYIO_NUM_AIN ];               ///< Analog input interrrupt flags
   uint16 iMask[ COPLEYIO_NUM_AIN ];                ///< Analog input interrrupt mask
   int32  programmableFilter[COPLEYIO_NUM_AIN][COPLEYIO_FILTER_32BIT_WORDS]; ///< Programmable filter for analog input (available for 1.06 firmware)
   uint16 oRaw[ COPLEYIO_NUM_AOUT ];                ///< Raw D/A output value (EtherCAT only)
   int32  oScaleVal[ COPLEYIO_NUM_AOUT ];           ///< DAC output scaled value. (EtherCAT only)
   int32  oScaleParam[ COPLEYIO_NUM_AOUT ];         ///< DAC scaling parameter. (EtherCAT only)
   int32  oOffset[ COPLEYIO_NUM_AOUT ];             ///< DAC offset parameter. (EtherCAT only)
   uint16 oFaultMode[ COPLEYIO_NUM_AOUT ];          ///< DAC fault mode. (EtherCAT only)
   int32  oFaultValue[ COPLEYIO_NUM_AOUT ];         ///< DAC fault value. (EtherCAT only)
};

/***************************************************************************/
/**
   This structure is used to return information about the PWM outputs
   of a Copley I/O module.
  */
/***************************************************************************/
struct CopleyIOPWM
{
   uint16 oRaw[ COPLEYIO_NUM_PWM ];                 ///< PWM output raw value
   uint32 oScaled[ COPLEYIO_NUM_PWM ];              ///< PWM output scaled value
   uint32 oFactor[ COPLEYIO_NUM_PWM ];              ///< PWM output scaling factor
   int32 oOffset[ COPLEYIO_NUM_PWM ];               ///< PWM output offset
};

/***************************************************************************/
/**
  IO Module configuration structure.  This structure contains all user 
  configurable parameters used by an IO module which may be stored in non-volatile
  memory.
  */
/***************************************************************************/
struct CopleyIOCfg
{
   /// Global IO Module parameters
   CopleyIOInfo info;

   /// Digital IO parameters
   CopleyIODigi digi;

   /// Analog input parameters
   CopleyIOAnlg anlg;

   /// PWM output parameters
   CopleyIOPWM pwm;
};

/***************************************************************************/
/**
 * This class represents a Copley CANopen I/O module.  
 * It extendes the standard I/O module with methods that can be used to 
 * restore from a CME generated I/O settings file.
 */
/***************************************************************************/
class CopleyIO: public IOModule
{
public:
   CopleyIO( void );
   CopleyIO( Network &net, int16 nodeID );
   CopleyIO( Network &net, int16 nodeID, IOModuleSettings &settings );
   virtual ~CopleyIO();
   virtual const Error *Init( Network &net, int16 nodeID );
   virtual const Error *Init( Network &net, int16 nodeID, IOModuleSettings &settings );

   /************************************************************************/
   // Just added for .cci transfer
   /************************************************************************/
   const Error *GetIOInfo( CopleyIOInfo &info );
   const Error *GetIODigi( CopleyIODigi &digi );
   const Error *GetIOAnlg( CopleyIOAnlg &anlg );
   const Error *GetIOPWM( CopleyIOPWM &pwm, CopleyIOInfo &info );
   const Error *GetIOCfg( CopleyIOCfg &cfg );
   const Error *SetIOInfo( CopleyIOInfo &info );
   const Error *SetIODigi( CopleyIODigi &digi );
   const Error *SetIOAnlg( CopleyIOAnlg &anlg );
   const Error *SetIOPWM( CopleyIOPWM &pwm, CopleyIOInfo &info );
   const Error *SetIOConfig( CopleyIOCfg &cfg );
   const Error *SaveIOConfig( void );
   const Error *SaveIOConfig( CopleyIOCfg &cfg );
   const Error *LoadFromFile( const char *name, int &line );
   const Error *SerialCmd( uint8 opcode, uint8 &ct, uint8 max, uint16 *data );

   uint8 lastSerialError;

private:
    bool isEcat = false;
};

/***************************************************************************/
/**
This class represents error conditions that can occur when loading IO module
data from a data file.
*/
/***************************************************************************/
class IOFileError: public Error
{
   public:
      static const IOFileError bankInvalid;  ///< I/O bank invalid

   protected:
      /// Standard protected constructor
      IOFileError( uint16 id, const char *desc ): Error( id, desc ){}
};

CML_NAMESPACE_END()

#endif

