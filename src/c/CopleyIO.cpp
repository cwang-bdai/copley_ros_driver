/********************************************************/
/*                                                      */
/*  Copley Motion Libraries                             */
/*                                                      */
/*  Copyright (c) 2010 Copley Controls Corp.            */
/*                     http://www.copleycontrols.com    */
/*                                                      */
/********************************************************/

/***************************************************************************/
/** \file
This file contains the CopleyIO object methods used to upload / download 
structures containing groups of module parameters.
*/
/***************************************************************************/

#include "CML.h"

CML_NAMESPACE_USE();

/***************************************************************************/
/**
  Default constructor for a Copley I/O module.  
  */
/***************************************************************************/
CopleyIO::CopleyIO( void )
{
}

/***************************************************************************/
/**
  Construct a CopleyIO object and initialize it using default settings.
  @param net The Network object that this module is associated with.
  @param nodeID The node ID of the module on the network.
  */
/***************************************************************************/
CopleyIO::CopleyIO( Network &net, int16 nodeID ): IOModule( net, nodeID )
{
    if (net.GetNetworkType() == NET_TYPE_ETHERCAT) {
        isEcat = true;
    }
}

/***************************************************************************/
/**
  Construct a CopleyIO object and initialize it using custom settings.
  @param net The Network object that this module is associated with.
  @param nodeID The node ID of the module on the network.
  @param settings The settings to use when configuring the module
  */
/***************************************************************************/
CopleyIO::CopleyIO( Network &net, int16 nodeID, IOModuleSettings &settings ): IOModule( net, nodeID, settings )
{
    if (net.GetNetworkType() == NET_TYPE_ETHERCAT) {
        isEcat = true;
    }
}

/***************************************************************************/
/**
  Virtual destructor for the IOModule object.
  */
/***************************************************************************/
CopleyIO::~CopleyIO()
{
}

/***************************************************************************/
/**
  Initialize a Copley IO module using default settings.  This function associates 
  the object with the CANopen network it will be used on.

  @param net The Network object that this module is associated with.
  @param nodeID The node ID of the module on the network.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *CopleyIO::Init( Network &net, int16 nodeID )
{
    if (net.GetNetworkType() == NET_TYPE_ETHERCAT) {
        isEcat = true;
    }
    IOModuleSettings settings;
    return IOModule::Init( net, nodeID, settings );
}

/***************************************************************************/
/**
  Initialize an I/O module using custom settings.  This function associates the 
  object with the CANopen network it will be used on.

  @param net The Network object that this module is associated with.
  @param nodeID The node ID of the module on the network.
  @param settings The settings to use when configuring the module
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *CopleyIO::Init( Network &net, int16 nodeID, IOModuleSettings &settings )
{
    if (net.GetNetworkType() == NET_TYPE_ETHERCAT) {
        isEcat = true;
    }

   return IOModule::Init( net, nodeID, settings );
}

/***************************************************************************/
/***************************************************************************/
/***************************************************************************/
/***************************************************************************/
/*
                   I/O information
*/
/***************************************************************************/
/***************************************************************************/
/***************************************************************************/
/***************************************************************************/

/***************************************************************************/
/**
  Read the I/O Module information parameters from the drive.

  @param info A structure that will be filled with the I/O module info
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *CopleyIO::GetIOInfo( CopleyIOInfo &info )
{
   int32 l;

   const Error *err = sdo.Upld32( CIOOBJID_INFO_SERIAL,    0, info.serial     );
   if( !err ) err = sdo.UpldString( CIOOBJID_INFO_MODEL,   0, l=COPLEYIO_MAX_STRING, info.model    );
   if( !err ) err = sdo.UpldString( CIOOBJID_INFO_MFGINFO, 0, l=COPLEYIO_MAX_STRING, info.mfgInfo  );
   if( !err ) err = sdo.Upld16( CIOOBJID_INFO_HWTYPE,      0, info.hwType     );
   if( !err ) err = sdo.Upld16( CIOOBJID_INFO_LOOPRATE,    0, info.loopRate   );
   if( !err ) err = sdo.Upld16( CIOOBJID_INFO_FWVERSION,   0, info.fwVersion  );
   if( !err ) err = sdo.Upld32( CIOOBJID_INFO_BAUD,        0, info.baud       );
   if( !err ) err = sdo.Upld16( CIOOBJID_INFO_MAXWORDS,    0, info.maxWords   );
   if( !err ) err = sdo.UpldString( CIOOBJID_INFO_NAME,    0, l=COPLEYIO_MAX_STRING, info.name     );
   if( !err ) err = sdo.UpldString( CIOOBJID_INFO_HOSTCFG, 0, l=COPLEYIO_MAX_STRING, info.hostCfg );
   if( !err ) err = sdo.Upld16( CIOOBJID_INFO_NODECFG,     0, info.nodeCfg    );
   if( !err ) err = sdo.Upld16( CIOOBJID_INFO_STATUS,      0, info.status     );
   if( !err ) err = sdo.Upld16( CIOOBJID_INFO_ANLGINT,     0, info.anlgInt    );
   if( !err ) err = sdo.Upld32( CIOOBJID_INFO_TIMEPWRUP,   0, info.timePwrUp  );
   if( !err ) err = sdo.Upld16( CIOOBJID_INFO_NTWKOPTN,    0, info.networkOptions );

   // CANopen Module Only
   if (!isEcat) {
       if( !err ) err = sdo.Upld16( CIOOBJID_INFO_RATECFG, 0, info.rateCfg    );
       if( !err ) err = sdo.Upld16( CIOOBJID_INFO_NODEID,  0, info.nodeID     );
       if (!err) err = sdo.Upld16(CIOOBJID_INFO_RATE, 0, info.rate);
       if (!err) err = sdo.Upld16(CIOOBJID_INFO_ANLGINTENA, 0, info.anlgIntEna);
       if (!err) err = sdo.Upld16(CIOOBJID_INFO_DIGIINTENA, 0, info.digiIntEna);
       if (!err) err = sdo.Upld32(CIOOBJID_INFO_PWMPERIODA, 0, info.pwmPeriodA);
       if (!err) err = sdo.Upld32(CIOOBJID_INFO_PWMPERIODB, 0, info.pwmPeriodB);
   }
   // EtherCAT Module Only
   else {
       if( !err ) err = sdo.Upld16( CIOOBJID_INFO_ECATALIAS, 0, info.ecatAlias);
   }

   return err;
}

/***************************************************************************/
/***************************************************************************/
/***************************************************************************/
/***************************************************************************/
/*
                          I/O digital
*/
/***************************************************************************/
/***************************************************************************/
/***************************************************************************/
/***************************************************************************/

/***************************************************************************/
/**
  Read the I/O Module digital parameters from the drive.

  @param digi A structure that will be filled with the digital I/O values
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *CopleyIO::GetIODigi( CopleyIODigi &digi )
{
   const Error *err = 0;

   // get the firmware version of the IO module.
   uint16 fwVersion{ 0 };
   err = sdo.Upld16(CIOOBJID_INFO_FWVERSION, 0, fwVersion);

   for (int i = 0; i < COPLEYIO_DIO_BANKS; i++)
   {
       if (!err) err = sdo.Upld16(CIOOBJID_DIGI_BANKMODE, i + 1, digi.bankMode[i]);
       if (!err) err = sdo.Upld16(CIOOBJID_DIGI_PULLUPMSK, i + 1, digi.pullupMsk[i]);
       if (!err) err = sdo.Upld16(CIOOBJID_DIGI_TYPEMSK, i + 1, digi.typeMsk[i]);
       if (!err) err = sdo.Upld16(CIOOBJID_DIGI_FAULTMSK, i + 1, digi.faultMsk[i]);
       if (!err) err = sdo.Upld16(CIOOBJID_DIGI_INVMSK, i + 1, digi.invMsk[i]);
       if (!err) err = sdo.Upld16(CIOOBJID_DIGI_VALUEMSK, i + 1, digi.valueMsk[i]);
       if (!err) err = sdo.Upld16(CIOOBJID_DIGI_MODEMSK, i + 1, digi.modeMsk[i]);
       if (!err) err = sdo.Upld16(CIOOBJID_DIGI_RAWMSK, i + 1, digi.rawMsk[i]);
       if (!err) err = sdo.Upld16(CIOOBJID_DIGI_HILOMSK, i + 1, digi.hiLoMsk[i]);
       if (!err) err = sdo.Upld16(CIOOBJID_DIGI_LOHIMSK, i + 1, digi.loHiMsk[i]);
       if (!err) err = sdo.Upld16(CIOOBJID_DIGI_DEBOUNCE0, i + 1, digi.debounce0[i]);
       if (!err) err = sdo.Upld16(CIOOBJID_DIGI_DEBOUNCE1, i + 1, digi.debounce1[i]);
       if (!err) err = sdo.Upld16(CIOOBJID_DIGI_DEBOUNCE2, i + 1, digi.debounce2[i]);
       if (!err) err = sdo.Upld16(CIOOBJID_DIGI_DEBOUNCE3, i + 1, digi.debounce3[i]);
       if (!err) err = sdo.Upld16(CIOOBJID_DIGI_DEBOUNCE4, i + 1, digi.debounce4[i]);
       if (!err) err = sdo.Upld16(CIOOBJID_DIGI_DEBOUNCE5, i + 1, digi.debounce5[i]);
       if (!err) err = sdo.Upld16(CIOOBJID_DIGI_DEBOUNCE6, i + 1, digi.debounce6[i]);
       if (!err) err = sdo.Upld16(CIOOBJID_DIGI_DEBOUNCE7, i + 1, digi.debounce7[i]);

       // firmware version must be 1.06 or greater
       if ((!err) && (fwVersion >= 261)) {
           err = sdo.Upld16(CIOOBJID_DIGI_PLDWNRMASK, i + 1, digi.pullDwnResMask[i]);
       }
   }

   for (int i = 0; i < COPLEYIO_CONFIG_WORDS; i++)
   {
       if (!err) err = sdo.Upld16(CIOOBJID_CONFIG_PARAM0, i + 1, digi.bankConfigParam0[i]);
       if (!err) err = sdo.Upld16(CIOOBJID_CONFIG_PARAM1, i + 1, digi.bankConfigParam1[i]);
       if (!err) err = sdo.Upld16(CIOOBJID_CONFIG_PARAM2, i + 1, digi.bankConfigParam2[i]);
       if (!err) err = sdo.Upld16(CIOOBJID_CONFIG_PARAM3, i + 1, digi.bankConfigParam3[i]);
       if (!err) err = sdo.Upld16(CIOOBJID_CONFIG_PARAM4, i + 1, digi.bankConfigParam4[i]);
       if (!err) err = sdo.Upld16(CIOOBJID_CONFIG_PARAM5, i + 1, digi.bankConfigParam5[i]);
       if (!err) err = sdo.Upld16(CIOOBJID_CONFIG_PARAM6, i + 1, digi.bankConfigParam6[i]);
       if (!err) err = sdo.Upld16(CIOOBJID_CONFIG_PARAM7, i + 1, digi.bankConfigParam7[i]);
       if (!err) err = sdo.Upld16(CIOOBJID_CONFIG_PARAM8, i + 1, digi.bankConfigParam8[i]);
       if (!err) err = sdo.Upld16(CIOOBJID_CONFIG_PARAM9, i + 1, digi.bankConfigParam9[i]);
   }

   return err;
}

/***************************************************************************/
/***************************************************************************/
/***************************************************************************/
/***************************************************************************/
/*
                           I/O analog
*/
/***************************************************************************/
/***************************************************************************/
/***************************************************************************/
/***************************************************************************/

/***************************************************************************/
/**
  Read the I/O Module analog parameters from the drive.

  @param anlg A structure that will be filled with the analog I/O values
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *CopleyIO::GetIOAnlg( CopleyIOAnlg &anlg )
{
  const Error *err = 0;

   for( int i=0; i<COPLEYIO_NUM_AIN; i++ )
   {
      if( !err ) err = sdo.Upld16( CIOOBJID_ANLG_IRAW,      i+1, anlg.iRaw[i]      );
      if( !err ) err = sdo.Upld32( CIOOBJID_ANLG_ISCALED,   i+1, anlg.iScaled[i]   );
      if( !err ) err = sdo.Upld32( CIOOBJID_ANLG_IFACTOR,   i+1, anlg.iFactor[i]   );
      if( !err ) err = sdo.Upld32( CIOOBJID_ANLG_IOFFSET,   i+1, anlg.iOffset[i]   );
      if( !err ) err = sdo.Upld32( CIOOBJID_ANLG_IUPLIMIT,  i+1, anlg.iUpLimit[i]  );
      if( !err ) err = sdo.Upld32( CIOOBJID_ANLG_ILOLIMIT,  i+1, anlg.iLoLimit[i]  );
      if( !err ) err = sdo.Upld32( CIOOBJID_ANLG_IABSDELTA, i+1, anlg.iAbsDelta[i] );
      if( !err ) err = sdo.Upld32( CIOOBJID_ANLG_IPOSDELTA, i+1, anlg.iPosDelta[i] );
      if( !err ) err = sdo.Upld32( CIOOBJID_ANLG_INEGDELTA, i+1, anlg.iNegDelta[i] );
      if( !err ) err = sdo.Upld16( CIOOBJID_ANLG_IFLAGS,    i+1, anlg.iFlags[i]    );
      if( !err ) err = sdo.Upld16( CIOOBJID_ANLG_IMASK,     i+1, anlg.iMask[i]     );
   }

   // Only the EtherCAT I/O Module has 2 analog outputs.
   if (isEcat) {
       for (int i = 0; i < COPLEYIO_NUM_AOUT; i++) {
           if (!err) err = sdo.Upld16(CIOOBJID_DA_OUTRAWVAL, i + 1, anlg.oRaw[i]);
           if (!err) err = sdo.Upld32(CIOOBJID_DA_OUTSCLVAL, i + 1, anlg.oScaleVal[i]);
           if (!err) err = sdo.Upld32(CIOOBJID_DA_OUTSCLPARAM, i + 1, anlg.oScaleParam[i]);
           if (!err) err = sdo.Upld32(CIOOBJID_DA_OFFSET, i + 1, anlg.oOffset[i]);
           if (!err) err = sdo.Upld16(CIOOBJID_DA_FLTMODE, i + 1, anlg.oFaultMode[i]);
           if (!err) err = sdo.Upld32(CIOOBJID_DA_FLTVAL, i + 1, anlg.oFaultValue[i]);
       }
   }

   // set all 12 of the analog input filters
   const int byteArraySize{ 31 };
   byte byteArray[byteArraySize];
   int numberOfBytesReceived{ 31 };

   const int uploadCommandArraySize{ 3 };
   byte byteCmdArray[uploadCommandArraySize];

   // 0x2000 = CANopen/Ecat Object Index of the serial-binary interface.
   int16 index{ 0x2000 };
   int16 subIndex{ 0 };

   for (int i = 0; i < COPLEYIO_NUM_AIN; i++) {

       byteCmdArray[0] = 0x0c; // serial-binary op-code for the get command
       byteCmdArray[1] = 0x4b; // parameter ID of the programmable filter
       byteCmdArray[2] = i;

       err = sdo.Download(index, subIndex, uploadCommandArraySize, byteCmdArray);
       if (err) { return err; }

       err = sdo.Upload(index, subIndex, numberOfBytesReceived, byteArray);
       if (err) { return err; }

       for (int j = 0; j < COPLEYIO_FILTER_32BIT_WORDS; j++) {
           // the order is reversed just like when sending it to drive.
           // starts on the 2'nd byte received.
           // The first byte indicates error.
           byte byte1 = byteArray[(j * 4) + 2];
           byte byte2 = byteArray[(j * 4) + 1];
           byte byte3 = byteArray[(j * 4) + 4];
           byte byte4 = byteArray[(j * 4) + 3];

           int value = int((unsigned char)(byte1) << 24 |
               (unsigned char)(byte2) << 16 |
               (unsigned char)(byte3) << 8 |
               (unsigned char)(byte4));

           anlg.programmableFilter[i][j] = value;
       }
   }

   return err;
}

/***************************************************************************/
/***************************************************************************/
/***************************************************************************/
/***************************************************************************/
/*
                                 I/O PWM
*/
/***************************************************************************/
/***************************************************************************/
/***************************************************************************/
/***************************************************************************/

/***************************************************************************/
/**
  Read the I/O Module PWM parameters from the drive.

  @param pwm A structure that will be filled with the PWM I/O values
  @param info A structure containing additional information about the module.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *CopleyIO::GetIOPWM( CopleyIOPWM &pwm, CopleyIOInfo &info )
{
   const Error *err = 0;
   uint8 ct;
   uint8 max = 2;
   uint8 opcode = 12;
   uint16 data[2];
   uint16 paramID = 0x51;
   
   // CANopen Modules Only
   if (!isEcat) {

       /***********************NOTE************************/
       /* The following code was added to mask a firmware */
       /* bug that prevented access to CANID 0x3051 for   */
       /* versions < 23. This workaround allows access to */
       /* this feature through serial communication. See  */
       /* the Accenet Programmer's guide for more info.   */
       /* Firmware bug fixed 11/1/10, rev 24.             */
       /***************************************************/
       if( info.fwVersion <23 )
       {
          for( uint16 i=0; i<COPLEYIO_NUM_PWM; i++ )
          {
             data[0] = (i<<8) | (paramID);               //bank concatenated with ParamID
             data[1] = 0;
             ct = 1;

             if( !err ) err = SerialCmd( opcode, ct, max, data );
             pwm.oRaw[i] = ((uint32)data[1] << 16) | (uint32)data[0];
             if( !err ) err = sdo.Upld32( CIOOBJID_PWM_OSCALED,   i+1, pwm.oScaled[i]   );
             if( !err ) err = sdo.Upld32( CIOOBJID_PWM_OFACTOR,   i+1, pwm.oFactor[i]   );
             if( !err ) err = sdo.Upld32( CIOOBJID_PWM_OOFFSET,   i+1, pwm.oOffset[i]   );
          }
       }

       else
       {
          for( int i=0; i<COPLEYIO_NUM_PWM; i++ )
          {
             if( !err ) err = sdo.Upld16( CIOOBJID_PWM_ORAW,      i+1, pwm.oRaw[i]      );
             if( !err ) err = sdo.Upld32( CIOOBJID_PWM_OSCALED,   i+1, pwm.oScaled[i]   );
             if( !err ) err = sdo.Upld32( CIOOBJID_PWM_OFACTOR,   i+1, pwm.oFactor[i]   );
             if( !err ) err = sdo.Upld32( CIOOBJID_PWM_OOFFSET,   i+1, pwm.oOffset[i]   );
          }
       }
   }
   return err;
}

/***************************************************************************/
/**
  Read the complete I/O configuration from the module and return it
  in the passed structure.  This structure holds every module parameter that
  can be stored to the module's internal flash memory.  The contents of the
  structure represent the complete I/O configuration.
  @param cfg The structure which will hold the uploaded configuration.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *CopleyIO::GetIOCfg( CopleyIOCfg &cfg )
{
   const Error *err = GetIOInfo( cfg.info );
   if( !err ) err = GetIODigi( cfg.digi );
   if( !err ) err = GetIOAnlg( cfg.anlg ); 
   if( !err ) err = GetIOPWM( cfg.pwm, cfg.info ); 

   return err;
}

/***************************************************************************/
/***************************************************************************/
/***************************************************************************/
/***************************************************************************/
/*
                             I/O information
*/
/***************************************************************************/
/***************************************************************************/
/***************************************************************************/
/***************************************************************************/

/***************************************************************************/
/**
  Write the I/O Module information parameters to the drive.

  @param info A structure that will be filled with the I/O module info
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *CopleyIO::SetIOInfo( CopleyIOInfo &info )
{
   const Error *err = sdo.Download( CIOOBJID_INFO_NAME,    0, COPLEYIO_MAX_STRING-1, info.name );
   if( !err ) err = sdo.Download( CIOOBJID_INFO_HOSTCFG,   0, COPLEYIO_MAX_STRING-1, info.hostCfg );
   if( !err ) err = sdo.Dnld16( CIOOBJID_INFO_NODECFG,     0, info.nodeCfg    );

   // CANopen Module Only
   if (!isEcat) {
       if( !err ) err = sdo.Dnld16( CIOOBJID_INFO_RATECFG,     0, info.rateCfg    );
       if( !err ) err = sdo.Dnld16( CIOOBJID_INFO_ANLGINTENA,  0, info.anlgIntEna );
       if( !err ) err = sdo.Dnld16( CIOOBJID_INFO_DIGIINTENA,  0, info.digiIntEna );
       if( !err ) err = sdo.Dnld32( CIOOBJID_INFO_PWMPERIODA,  0, info.pwmPeriodA );
       if( !err ) err = sdo.Dnld32( CIOOBJID_INFO_PWMPERIODB,  0, info.pwmPeriodB );
   }

   return err;
}

/***************************************************************************/
/***************************************************************************/
/***************************************************************************/
/***************************************************************************/
/*
                             I/O digital
*/
/***************************************************************************/
/***************************************************************************/
/***************************************************************************/
/***************************************************************************/

/***************************************************************************/
/**
  Write the I/O Module digital parameters to the drive.

  @param digi A structure that will be filled with the digital I/O values
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *CopleyIO::SetIODigi( CopleyIODigi &digi )
{
   const Error *err = 0;

   uint16 fwVersion{ 0 };
   err = sdo.Upld16(CIOOBJID_INFO_FWVERSION, 0, fwVersion);

   for( int i=0; i<COPLEYIO_DIO_BANKS; i++ )
   {
       // CANopen Bank D (sub-index 4) of param 0x20 (bank mode) does not support 0x200 (PWM output mode) or 0x201 (quad encoder mode).
       if ((!isEcat) && (i == 3) && ((digi.bankMode[i] == 0x200) || (digi.bankMode[i] == 0x201))) {
           // nothing to do at this time.
       }
       else {
           if (!err) err = sdo.Dnld16(CIOOBJID_DIGI_BANKMODE, i + 1, digi.bankMode[i]);
       }
      if( !err ) err = sdo.Dnld16( CIOOBJID_DIGI_PULLUPMSK, i+1, digi.pullupMsk[i] );
      if( !err ) err = sdo.Dnld16( CIOOBJID_DIGI_TYPEMSK,   i+1, digi.typeMsk[i]   );
      if( !err ) err = sdo.Dnld16( CIOOBJID_DIGI_FAULTMSK,  i+1, digi.faultMsk[i]  );
      if( !err ) err = sdo.Dnld16( CIOOBJID_DIGI_INVMSK,    i+1, digi.invMsk[i]    );
      if( !err ) err = sdo.Dnld16( CIOOBJID_DIGI_VALUEMSK,  i+1, digi.valueMsk[i]  );
      if( !err ) err = sdo.Dnld16( CIOOBJID_DIGI_MODEMSK,   i+1, digi.modeMsk[i]   );
      if( !err ) err = sdo.Dnld16( CIOOBJID_DIGI_HILOMSK,   i+1, digi.hiLoMsk[i]   );
      if( !err ) err = sdo.Dnld16( CIOOBJID_DIGI_LOHIMSK,   i+1, digi.loHiMsk[i]   );
      if( !err ) err = sdo.Dnld16( CIOOBJID_DIGI_DEBOUNCE0, i+1, digi.debounce0[i] );
      if( !err ) err = sdo.Dnld16( CIOOBJID_DIGI_DEBOUNCE1, i+1, digi.debounce1[i] );
      if( !err ) err = sdo.Dnld16( CIOOBJID_DIGI_DEBOUNCE2, i+1, digi.debounce2[i] );
      if( !err ) err = sdo.Dnld16( CIOOBJID_DIGI_DEBOUNCE3, i+1, digi.debounce3[i] );
      if( !err ) err = sdo.Dnld16( CIOOBJID_DIGI_DEBOUNCE4, i+1, digi.debounce4[i] );
      if( !err ) err = sdo.Dnld16( CIOOBJID_DIGI_DEBOUNCE5, i+1, digi.debounce5[i] );
      if( !err ) err = sdo.Dnld16( CIOOBJID_DIGI_DEBOUNCE6, i+1, digi.debounce6[i] );
      if( !err ) err = sdo.Dnld16( CIOOBJID_DIGI_DEBOUNCE7, i+1, digi.debounce7[i] );
      
      // firmware must be 1.06 or greater.
      if( (!err ) && ( fwVersion >= 261 ) ) {
          err = sdo.Dnld16( CIOOBJID_DIGI_PLDWNRMASK, i+1, digi.pullDwnResMask[i] );
      }
   }

   for (int i = 0; i < COPLEYIO_CONFIG_WORDS; i++)
   {
       if (!err) err = sdo.Dnld16(CIOOBJID_CONFIG_PARAM0, i + 1, digi.bankConfigParam0[i]);
       if (!err) err = sdo.Dnld16(CIOOBJID_CONFIG_PARAM1, i + 1, digi.bankConfigParam1[i]);
       if (!err) err = sdo.Dnld16(CIOOBJID_CONFIG_PARAM2, i + 1, digi.bankConfigParam2[i]);
       if (!err) err = sdo.Dnld16(CIOOBJID_CONFIG_PARAM3, i + 1, digi.bankConfigParam3[i]);
       if (!err) err = sdo.Dnld16(CIOOBJID_CONFIG_PARAM4, i + 1, digi.bankConfigParam4[i]);
       if (!err) err = sdo.Dnld16(CIOOBJID_CONFIG_PARAM5, i + 1, digi.bankConfigParam5[i]);
       if (!err) err = sdo.Dnld16(CIOOBJID_CONFIG_PARAM6, i + 1, digi.bankConfigParam6[i]);
       if (!err) err = sdo.Dnld16(CIOOBJID_CONFIG_PARAM7, i + 1, digi.bankConfigParam7[i]);
       if (!err) err = sdo.Dnld16(CIOOBJID_CONFIG_PARAM8, i + 1, digi.bankConfigParam8[i]);
       if (!err) err = sdo.Dnld16(CIOOBJID_CONFIG_PARAM9, i + 1, digi.bankConfigParam9[i]);
   }

   return err;
}

/***************************************************************************/
/***************************************************************************/
/***************************************************************************/
/***************************************************************************/
/*
                            I/O analog
*/
/***************************************************************************/
/***************************************************************************/
/***************************************************************************/
/***************************************************************************/

/***************************************************************************/
/**
  Write the I/O Module analog parameters to the drive.

  @param anlg A structure that will be filled with the analog I/O values
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *CopleyIO::SetIOAnlg( CopleyIOAnlg &anlg )
{
   const Error *err = 0;

   for( int i=0; i<COPLEYIO_NUM_AIN; i++ )
   {
      if( !err ) err = sdo.Dnld32( CIOOBJID_ANLG_IFACTOR,   i+1, anlg.iFactor[i]   );
      if( !err ) err = sdo.Dnld32( CIOOBJID_ANLG_IOFFSET,   i+1, anlg.iOffset[i]   );
      if( !err ) err = sdo.Dnld32( CIOOBJID_ANLG_IUPLIMIT,  i+1, anlg.iUpLimit[i]  );
      if( !err ) err = sdo.Dnld32( CIOOBJID_ANLG_ILOLIMIT,  i+1, anlg.iLoLimit[i]  );
      if( !err ) err = sdo.Dnld32( CIOOBJID_ANLG_IABSDELTA, i+1, anlg.iAbsDelta[i] );
      if( !err ) err = sdo.Dnld32( CIOOBJID_ANLG_IPOSDELTA, i+1, anlg.iPosDelta[i] );
      if( !err ) err = sdo.Dnld32( CIOOBJID_ANLG_INEGDELTA, i+1, anlg.iNegDelta[i] );
      if( !err ) err = sdo.Dnld16( CIOOBJID_ANLG_IMASK,     i+1, anlg.iMask[i]     );
   }

   // EtherCAT module only
   if (isEcat) {
       for (int i = 0; i < COPLEYIO_NUM_AOUT; i++) {
            if (!err) err = sdo.Dnld32(CIOOBJID_DA_OUTSCLVAL, i + 1, anlg.oScaleVal[i]);
            if (!err) err = sdo.Dnld32(CIOOBJID_DA_OUTSCLPARAM, i + 1, anlg.oScaleParam[i]);
            if (!err) err = sdo.Dnld32(CIOOBJID_DA_OFFSET, i + 1, anlg.oOffset[i]);
            if (!err) err = sdo.Dnld16(CIOOBJID_DA_FLTMODE, i + 1, anlg.oFaultMode[i]);
            if (!err) err = sdo.Dnld32(CIOOBJID_DA_FLTVAL, i + 1, anlg.oFaultValue[i]);
       }
   }

   // set all 12 of the analog input filters
   const int byteArraySize = 31;
   byte byteArray[byteArraySize];
   
   // 0x2000 = CANopen/Ecat Object Index of the serial-binary interface.
   int16 index{ 0x2000 };
   int16 subIndex{ 0 };

   for (int i = 0; i < COPLEYIO_NUM_AIN; i++) {
       
       int value1 = anlg.programmableFilter[i][0];
       int value2 = anlg.programmableFilter[i][1];
       int value3 = anlg.programmableFilter[i][2];
       int value4 = anlg.programmableFilter[i][3];
       int value5 = anlg.programmableFilter[i][4];
       int value6 = anlg.programmableFilter[i][5];
       int value7 = anlg.programmableFilter[i][6];

       byteArray[0] = 0x0d; // op-code for set command
       byteArray[1] = 0x4b; // parameter of programmable filter 
       byteArray[2] = i;
       byteArray[3] = (value1 & 0x00ff0000) >> 16;
       byteArray[4] = (value1 & 0xff000000) >> 24;
       byteArray[5] = (value1 & 0x000000ff);
       byteArray[6] = (value1 & 0x0000ff00) >> 8;

       byteArray[7] = (value2 & 0x00ff0000) >> 16;
       byteArray[8] = (value2 & 0xff000000) >> 24;
       byteArray[9] = (value2 & 0x000000ff);
       byteArray[10] = (value2 & 0x0000ff00) >> 8;
       
       byteArray[11] = (value3 & 0x00ff0000) >> 16;
       byteArray[12] = (value3 & 0xff000000) >> 24;
       byteArray[13] = (value3 & 0x000000ff);
       byteArray[14] = (value3 & 0x0000ff00) >> 8;
       
       byteArray[15] = (value4 & 0x00ff0000) >> 16;
       byteArray[16] = (value4 & 0xff000000) >> 24;
       byteArray[17] = (value4 & 0x000000ff);
       byteArray[18] = (value4 & 0x0000ff00) >> 8;
       
       byteArray[19] = (value5 & 0x00ff0000) >> 16;
       byteArray[20] = (value5 & 0xff000000) >> 24;
       byteArray[21] = (value5 & 0x000000ff);
       byteArray[22] = (value5 & 0x0000ff00) >> 8;
       
       byteArray[23] = (value6 & 0x00ff0000) >> 16;
       byteArray[24] = (value6 & 0xff000000) >> 24;
       byteArray[25] = (value6 & 0x000000ff);
       byteArray[26] = (value6 & 0x0000ff00) >> 8;
       
       byteArray[27] = (value7 & 0x00ff0000) >> 16;
       byteArray[28] = (value7 & 0xff000000) >> 24;
       byteArray[29] = (value7 & 0x000000ff);
       byteArray[30] = (value7 & 0x0000ff00) >> 8;

       err = sdo.Download(index, subIndex, byteArraySize, byteArray);
       if (err) { return err; }
   }

   return err;
}

/***************************************************************************/
/***************************************************************************/
/***************************************************************************/
/***************************************************************************/
/*
                              I/O PWM
*/
/***************************************************************************/
/***************************************************************************/
/***************************************************************************/
/***************************************************************************/

/***************************************************************************/
/**
  Write the I/O Module PWM parameters to the drive.

  @param pwm A structure that will be filled with the PWM I/O values
  @param info A structure containing additional information about the module.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *CopleyIO::SetIOPWM( CopleyIOPWM &pwm, CopleyIOInfo &info )
{
   const Error *err = 0;
   uint8 ct;
   uint8 max = 3;
   uint8 opcode = 13;
   uint16 data[3];
   uint16 paramID = 0x51;

   // CANopen Module Only
   if (!isEcat) {

       /***********************NOTE************************/
       /* The following code was added to mask a firmware */
       /* bug that prevented access to CANID 0x3051 for   */
       /* versions < 23. This workaround allows access to */
       /* this feature through serial communication. See  */
       /* the Accenet Programmer's guide for more info.   */
       /* Firmware bug fixed 11/1/10, rev 24.             */
       /***************************************************/
       if( info.fwVersion < 23 )
       {
          for( uint16 i=0; i<COPLEYIO_NUM_PWM; i++ )
          {
             ct = 3;
             data[2] = (uint16)pwm.oScaled[i];          //lower half of oScaled
             data[1] = (uint16)(pwm.oScaled[i] >> 16);  //upper half of oScaled
             data[0] = (i<<8) | (paramID);              //bank concatenated with ParamID

             if( !err ) err = SerialCmd( opcode, ct, max, data );
             if( !err ) err = sdo.Dnld32( CIOOBJID_PWM_OFACTOR,   i+1, pwm.oFactor[i]   );
             if( !err ) err = sdo.Dnld32( CIOOBJID_PWM_OOFFSET,   i+1, pwm.oOffset[i]   );
          }
       }

       else
       {
          for( int i=0; i<COPLEYIO_NUM_PWM; i++ )
          {
             if( !err ) err = sdo.Dnld32( CIOOBJID_PWM_OSCALED,   i+1, pwm.oScaled[i]   );
             if( !err ) err = sdo.Dnld32( CIOOBJID_PWM_OFACTOR,   i+1, pwm.oFactor[i]   );
             if( !err ) err = sdo.Dnld32( CIOOBJID_PWM_OOFFSET,   i+1, pwm.oOffset[i]   );
          }
       }
   }

   return err;
}

/***************************************************************************/
/**
  Write the complete I/O configuration from the module and return it
  in the passed structure.  This structure holds every I/O parameter that
  can be stored to the module's internal flash memory.  The contents of the
  structure represent the complete I/O configuration.
  @param cfg The structure which will hold the uploaded configuration.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *CopleyIO::SetIOConfig( CopleyIOCfg &cfg )
{
   const Error *err = SetIOInfo( cfg.info );
   if( !err ) err = SetIODigi( cfg.digi );
   if( !err ) err = SetIOAnlg( cfg.anlg ); 
   if( !err ) err = SetIOPWM( cfg.pwm, cfg.info ); 

   return err;
}

/***************************************************************************/
/**
  Save all I/O parameters to internal flash memory.  Flash memory is a 
  type of non-volatile RAM which allows module parameters to be saved 
  between power cycles.  When this function is called, any module parameters
  that may be stored to flash will be copied from their working (RAM) locations
  to the stored (flash) locations.

  For a list of those I/O parameters which may be saved to flash memory,
  see the IOConfig structure.  Every member of that structure represents an
  io module parameter that may be saved to flash.

  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *CopleyIO::SaveIOConfig( void )
{
   return sdo.Dnld32( 0x1010, 1, 0x65766173 );
}

/***************************************************************************/
/**
  Upload the passed io module configuration to the module's working memory,
  and then copy that working memory to flash.

  @param cfg The structure which holds the new configuration.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *CopleyIO::SaveIOConfig( CopleyIOCfg &cfg )
{
   const Error *err = SetIOConfig( cfg );
   if( !err ) err = SaveIOConfig();
   return err;
}

/***************************************************************************/
/**
Send a serial port message to a Copley device over the CANopen network.

Copley devices use serial ports for some basic configuration purposes.  Most
of the functions available over the serial interface are also available in 
the CANopen object dictionary, but not everything.

This function allows a native serial port command to be sent over the CANopen
network.  It allows any remaining features of the device to be accessed when
only a CANopen connection is available.

@param opcode The command code to be sent to the amplifier.
@param ct     The number of 16-bit words of data to be sent to the amplifier.  On
              return, this parameter will hold the number of 16-bit words of response
              data passed back from the amplifier.
@param max    The maximum number of words of response data that the data array can 
              hold.
@param data   An array of data to be passed to the node with the command.  On return,
              any response data (up to max words) will be passed here.  
              If this parameter is not specified, then no data will be passed or returned
              regardless of the values passed in max and ct.

@return       An error object, or null on success.
*/
/***************************************************************************/
const Error *CopleyIO::SerialCmd( uint8 opcode, uint8 &ct, uint8 max, uint16 *data )
{
   #define MAX_MSG_BYTES 200

   if( !data ) max = ct = 0;

   if( ct > MAX_MSG_BYTES/2 )
      return &CopleyNodeError::SerialMsgLen;

   uint8 buff[ MAX_MSG_BYTES + 1 ];
   buff[0] = opcode;

   int i;
   for( i=0; i<ct; i++ )
   {
      buff[2*i+1] = (uint8)data[i];
      buff[2*i+2] = (uint8)(data[i]>>8);
   }

   const CML::Error *err = sdo.Download( 0x2000, 0, 2*ct+1, buff );
   if( err ) return err;

   int32 len = MAX_MSG_BYTES+1;
   err = sdo.Upload( 0x2000, 0, len, (uint8 *)buff );
   if( err ) return err;

   lastSerialError = buff[0];
   if( lastSerialError )
      return &CopleyNodeError::SerialError;

   ct = (len-1)/2;
   if( ct > max ) ct = max;

   for( i=0; i<ct; i++ )
      data[i] = bytes_to_uint16( &buff[2*i+1] );

   return 0;
}

