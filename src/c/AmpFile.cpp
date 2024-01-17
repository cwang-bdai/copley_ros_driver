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
  This file contains code used to read CME-2 .ccx and .ccd amplifier files.
  */
/***************************************************************************/

#include "CML.h"

#ifdef CML_FILE_ACCESS_OK
#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#endif

CML_NAMESPACE_USE();

CML_NEW_ERROR( AmpFileError, format,        "Amplifier file format error" );
CML_NEW_ERROR( AmpFileError, tooOld,        "Amplifier file format is too old, use CME version 3.1 or later" );
CML_NEW_ERROR( AmpFileError, noFileAccess,  "File access was not enabled at compile time.  See CML_Settings.h" );
CML_NEW_ERROR( AmpFileError, fileOpen,      "Error opening amplifier file" );
CML_NEW_ERROR( AmpFileError, range,         "A parameter in the amplifier file is out of range" );
CML_NEW_ERROR( AmpFileError, axis,          "Multi-axis ccx files are not supported" );
CML_NEW_ERROR( AmpFileError, axisCt,        "Amplifier axis count does not match ccx file" );
CML_NEW_ERROR( AmpFileError, FPGAtooOld,    "Amplifier firmware is too old, update to 2.72 or later");

// Handy macros
#define StrToLoadPos( str, pos )  { int32 i32; err=StrToInt32( str, i32 ); pos = PosLoad2User(i32); }
#define StrToLoadVel( str, vel )  { int32 i32; err=StrToInt32( str, i32 ); vel = VelLoad2User(i32); }
#define StrToLoadAcc( str, acc )  { int32 i32; err=StrToInt32( str, i32 ); acc = AccLoad2User(i32); }
#define StrToLoadJrk( str, jrk )  { int32 i32; err=StrToInt32( str, i32 ); jrk = JrkLoad2User(i32); }
#define StrToMtrPos( str, pos )  { int32 i32; err=StrToInt32( str, i32 ); pos = PosMtr2User(i32); }
#define StrToMtrVel( str, vel )  { int32 i32; err=StrToInt32( str, i32 ); vel = VelMtr2User(i32); }
#define StrToMtrAcc( str, acc )  { int32 i32; err=StrToInt32( str, i32 ); acc = AccMtr2User(i32); }
#define VERSION( major, minor )    ((major<<8) | minor)

// local functions
static COPLEY_HOME_METHOD HomeMethodConvert( uint16 x );
const Error *WriteCCDToEcatDrive(const char* name, EtherCAT &net, Amp &amp);
const Error *WriteCCDToCANDrive(const char *name, CanOpen &net, Amp &amp);
const Error* ReadCCDFromDrive(const char* name, Network& net, Amp& amp);

long GetCCDFileSize(FILE* fp);

/***************************************************************************/
/**
  Load the specified amplifier data file.  This function presently supports
  loading *.ccx files created by the CME-2 program, version 3.1 and later.

  @param name The name (and optionally path) of the file to load

  @param line If not NULL, the last line number read from the file is returned
              here.  This is useful for finding file format errors.

  @return A pointer to an error object, or NULL on success.
  */
/***************************************************************************/
const Error *Amp::LoadFromFile( const char *name, int &line )
{
#define MAX_LINE_LEN  200
#define MAX_LINE_SEGS 4


   line = 0;
#ifndef CML_FILE_ACCESS_OK
   return &AmpFileError::noFileAccess;
#else
   AmpConfig cfg;

   // Load the configuration structure with the current amplifier
   // configuration data.  This ensures that any parameters not 
   // specified in the file will remain unchanged.
   const Error *err = GetAmpConfig( cfg );
   if( err ) return err;

   // Open the file and read each parameter into my configuration
   // structure.
   FILE *fp;

   fp = fopen( name, "rt" );
   if( !fp )
      return &AmpFileError::fileOpen;

   char buff[MAX_LINE_LEN];
   char *seg[MAX_LINE_SEGS];
   int ct;
   int numOfSegs = 3;
   int segIndex = 2;

   int16 fileRev, mtrFamily, fileAxisCount;
   int16 i16;
   uint16 u16;
   int32 i32;

   // Read file version number
   line++;
   ReadLine( fp, buff, MAX_LINE_LEN );
   SplitLine( buff, seg, MAX_LINE_SEGS );
   err = StrToInt16( seg[0], fileRev, 10 );
   if( err ) return err;

   if( fileRev < 9 ) 
      return &AmpFileError::tooOld;

   line++;
   ReadLine( fp, buff, MAX_LINE_LEN );
   SplitLine( buff, seg, MAX_LINE_SEGS );

   if(fileRev < 12){
      // Read the motor family info
      err = StrToInt16( seg[0], mtrFamily, 10 );
      if( err ) return err;
   }

   if(fileRev == 12){
      // Read the motor family info
      SplitLine( seg[2], seg, 4, ':' );
      err = StrToInt16( seg[0], mtrFamily, 16 );
      if( err ) return err;
   }

   // Rev >= 13 supports multi-axis .ccx read/write
   if( fileRev >= 13 )
   {
      numOfSegs = 4;
      segIndex = 3;

      err = StrToInt16( seg[0], fileAxisCount, 10 );

      // Make sure axis number and axis ct match up
      if( fileAxisCount != 1 )
      {
         return &AmpFileError::axis;
      }

      // Just initialize motor family to zero.
      // This will be updated below when the host 
      // config string is read from the file
      mtrFamily = 0;
   } 

   if( mtrFamily < 0 || mtrFamily > 2 ) 
      return &AmpFileError::format;

   cfg.CME_Config[4] = (char)(mtrFamily>>8);
   cfg.CME_Config[5] = (char)(mtrFamily);

   // Read all parameters
   while( !feof(fp) && !err )
   {
      int16 param;

      line++; // Iterate through all lines in single axis

      ReadLine( fp, buff, MAX_LINE_LEN );

      ct = SplitLine( buff, seg, MAX_LINE_SEGS );
      if( ct == 0 )
         continue;

      if( ct != numOfSegs )
         err = &AmpFileError::format;
      else
         err = StrToInt16( seg[0], param, 16 );

      if( err ) break;

      switch( param )
      {
         case 0x000: err = StrToInt16( seg[segIndex], cfg.cLoop.kp ); break;
         case 0x001: err = StrToInt16( seg[segIndex], cfg.cLoop.ki ); break;
         case 0x002: err = StrToInt16( seg[segIndex], cfg.progCrnt ); break;

         case 0x019: err = StrToInt32( seg[segIndex], cfg.ref.scale  ); break;
         case 0x01a: err = StrToInt16( seg[segIndex], cfg.ref.offset ); break;

         case 0x021: err = StrToInt16( seg[segIndex], cfg.cLoop.peakLim ); break;
         case 0x022: err = StrToInt16( seg[segIndex], cfg.cLoop.contLim ); break;
         case 0x023: err = StrToInt16( seg[segIndex], cfg.cLoop.peakTime ); break;

         case 0x024:
            err = StrToInt16( seg[segIndex], i16 );
            cfg.controlMode = (AMP_MODE)(i16<<8);

            // The ccx file only holds the amplifier control method.
            // Build a proper amp mode out of this if it's one of the
            // CANopen control methods.
            if( (cfg.controlMode==AMPMODE_CAN_SERVO) || 
                (cfg.controlMode==AMPMODE_CAN_USTEP) )
               cfg.controlMode = (AMP_MODE)(cfg.controlMode | AMPMODE_CAN_PROFILE);
            break;

         case 0x026: err = StrToInt16( seg[segIndex], cfg.ref.deadband ); break;
         case 0x027: err = StrToInt16( seg[segIndex], cfg.vLoop.kp ); break;
         case 0x028: err = StrToInt16( seg[segIndex], cfg.vLoop.ki ); break;
         case 0x02e: err = StrToInt16( seg[segIndex], cfg.vLoop.kaff ); break;
         case 0x02f: StrToMtrVel( seg[segIndex], cfg.progVel ); break;
         case 0x030: err = StrToInt16( seg[segIndex], cfg.pLoop.kp    ); break;
         case 0x031: err = StrToInt16( seg[segIndex], cfg.vLoop.shift ); break;
         case 0x033: err = StrToInt16( seg[segIndex], cfg.pLoop.kvff  ); break;
         case 0x034: err = StrToInt16( seg[segIndex], cfg.pLoop.kaff  ); break;

         // These three parameters use non-standard units in the amplifier
         // thus the multiplication after converting them to user units.
         case 0x036:
            StrToMtrAcc( seg[segIndex], cfg.vLoop.maxAcc ); 
            cfg.vLoop.maxAcc *= 100;
            break;

         case 0x037: 
            StrToMtrAcc( seg[segIndex], cfg.vLoop.maxDec ); 
            cfg.vLoop.maxDec *= 100;
            break;

         case 0x039: 
            StrToMtrAcc( seg[segIndex], cfg.vLoop.estopDec ); 
            cfg.vLoop.estopDec *= 100;
            break;

         case 0x03a: StrToMtrVel( seg[segIndex], cfg.vLoop.maxVel ); break;
         case 0x03e: StrToMtrVel( seg[segIndex], cfg.window.velWarnWin ); break;
         case 0x03f: err = StrToUInt16( seg[segIndex], cfg.window.velWarnTime ); break;

         case 0x040: err = StrToUInt16( seg[segIndex], cfg.motor.type ); break;
         case 0x041: strncpy( cfg.motor.mfgName, seg[segIndex], COPLEY_MAX_STRING ); break;
         case 0x042: strncpy( cfg.motor.model,   seg[segIndex], COPLEY_MAX_STRING ); break;
         case 0x043:
            if( !strcmp( seg[segIndex], "Metric"  ) ) 
               cfg.motor.mtrUnits = 0;
            else if( !strcmp( seg[segIndex], "English"  ) ) 
               cfg.motor.mtrUnits = 1;
            else
               err = StrToInt16( seg[segIndex], cfg.motor.mtrUnits  );
            break;

         case 0x044: 
            err = StrToUInt32( seg[segIndex], cfg.motor.inertia ); 
            break;

         case 0x045: 
            err = StrToInt16( seg[segIndex], cfg.motor.poles ); 
            break;

         case 0x046:
            if( !strcmp( seg[segIndex], "No" ) ) 
               cfg.motor.hasBrake = false;

            else if( !strcmp( seg[segIndex], "Yes" ) ) 
               cfg.motor.hasBrake = true;

            else
            {
               err = StrToInt16( seg[segIndex], i16 );
               cfg.motor.hasBrake = (i16==0);
            }
            break;

         case 0x048: err = StrToUInt32( seg[segIndex], cfg.motor.trqConst ); break;
         case 0x049: err = StrToUInt16( seg[segIndex], cfg.motor.resistance ); break;
         case 0x04a: err = StrToUInt16( seg[segIndex], cfg.motor.inductance ); break; 
         case 0x04b: err = StrToUInt32( seg[segIndex], cfg.motor.trqPeak ); break;
         case 0x04c: err = StrToUInt32( seg[segIndex], cfg.motor.trqCont ); break;

         case 0x04d:
            StrToMtrVel( seg[segIndex], cfg.motor.velMax );
            break;

         case 0x04e: err = StrToInt16( seg[segIndex], i16 ); cfg.motor.mtrReverse = (i16!=0); break; 
         case 0x04f: err = StrToInt16( seg[segIndex], cfg.motor.hallOffset ); break;

         case 0x050: 
            if( !strcmp( seg[segIndex], "None"  ) ) cfg.motor.hallType = 0;
            else if( !strcmp( seg[segIndex], "Digital"  ) ) cfg.motor.hallType = 1;
            else if( !strcmp( seg[segIndex], "Analog"   ) ) cfg.motor.hallType = 2;
            else
               err = StrToInt16( seg[segIndex], cfg.motor.hallType   );
            break;

         case 0x052: 
            err = StrToInt16( seg[segIndex], cfg.motor.hallWiring ); 
            break;

         case 0x053: 
            err = StrToInt16( seg[segIndex], cfg.motor.stopTime ); 
            break;

         case 0x054: 
            err = StrToInt16( seg[segIndex], cfg.motor.brakeDelay ); 
            break;

         case 0x055: 
            StrToMtrVel( seg[segIndex], cfg.motor.brakeVel );
            break;

         case 0x056: 
            err = StrToUInt32( seg[segIndex], cfg.motor.backEMF ); 
            break;

         case 0x057:
            err = StrToInt32( seg[segIndex], cfg.motor.stepsPerRev );
            break;

         case 0x058:
            err = StrToInt32( seg[segIndex], cfg.motor.gearRatio );
            break;

         case 0x059:
            err = StrToInt16( seg[segIndex], cfg.motor.hallVelShift );
            break;

         case 0x5A:
            err = StrToUInt16( seg[segIndex], cfg.encoderOutCfg );
            break;

         case 0x05B:
            err = StrToInt32( seg[segIndex], cfg.motor.loadEncRes );
            break;

         case 0x05C:
            err = StrToInt16( seg[segIndex], i16 ); 
            cfg.motor.loadEncReverse = (i16!=0);
            break;

         case 0x05D:
            err = StrToInt16( seg[segIndex], cfg.motor.loadEncType );
            break;

         case 0x05f: 
            err = StrToFilter( seg[segIndex], cfg.vloopOutFltr );
            break;

         case 0x060:
            if( !strcmp( seg[segIndex], "Incremental"  ) ) cfg.motor.encType = 0;
            else if( !strcmp( seg[segIndex], "None"    ) ) cfg.motor.encType = 1;
            else if( !strcmp( seg[segIndex], "Analog"  ) ) cfg.motor.encType = 2;
            else if( !strcmp( seg[segIndex], "Absolute") ) cfg.motor.encType = 3;
            else
               err = StrToInt16( seg[segIndex], cfg.motor.encType );
            break;

         case 0x061: err = StrToInt16( seg[segIndex], cfg.motor.encUnits    ); break;
         case 0x062: err = StrToInt32( seg[segIndex], cfg.motor.ctsPerRev   ); break;
         case 0x063: err = StrToInt16( seg[segIndex], cfg.motor.encRes      ); break;
         case 0x064: err = StrToInt32( seg[segIndex], cfg.motor.eleDist     ); break;

         case 0x065: 
            err = StrToInt16( seg[segIndex], i16 ); 
            cfg.motor.encReverse = (i16!=0); 
            break; 

         case 0x066:
            err = StrToInt32( seg[segIndex], cfg.motor.ndxDist );
            break;

         case 0x067:
            err = StrToInt16( seg[segIndex], cfg.motor.encShift );
            break;

         case 0x06A:
            err = StrToInt32( seg[segIndex], cfg.cLoop.slope );
            break;

         case 0x06B:
            err = StrToFilter( seg[segIndex], cfg.vloopCmdFltr );
            break;

         case 0x06C:
            err = StrToUInt16( seg[segIndex], cfg.capCtrl );
            break;

         case 0x06e:
            err = StrToUInt16( seg[segIndex], cfg.motor.resolverCycles );
            break;

         case 0x06f:
            err = StrToInt16( seg[segIndex], i16 ); 
            cfg.pwmMode = (AMP_PWM_MODE)i16; 
            break;

         case 0x070:
            err = StrToOutCfg( seg[segIndex], cfg.io.outCfg[0], cfg.io.outMask[0], cfg.io.outMask1[0]  );
            break;

         case 0x071:
            err = StrToOutCfg( seg[segIndex], cfg.io.outCfg[1], cfg.io.outMask[1], cfg.io.outMask1[1]  );
            break;

         case 0x072:
            err = StrToOutCfg( seg[segIndex], cfg.io.outCfg[2], cfg.io.outMask[2], cfg.io.outMask1[2]  );
            break;

         case 0x073: 
            err = StrToOutCfg( seg[segIndex], cfg.io.outCfg[3], cfg.io.outMask[3], cfg.io.outMask1[3]  );
            break;

         case 0x074: 
            err = StrToOutCfg( seg[segIndex], cfg.io.outCfg[4], cfg.io.outMask[4], cfg.io.outMask1[4]  );
            break;

         case 0x075: 
            err = StrToOutCfg( seg[segIndex], cfg.io.outCfg[5], cfg.io.outMask[5], cfg.io.outMask1[5]  );
            break;

         case 0x076: 
            err = StrToOutCfg( seg[segIndex], cfg.io.outCfg[6], cfg.io.outMask[6], cfg.io.outMask1[6]  );
            break;

         case 0x077: 
            err = StrToOutCfg( seg[segIndex], cfg.io.outCfg[7], cfg.io.outMask[7], cfg.io.outMask1[7]  );
            break;

         case 0x078:
            err = StrToInt16( seg[segIndex], i16 );
            cfg.io.inCfg[ 0] = (INPUT_PIN_CONFIG)i16;
            break;

         case 0x079:
            err = StrToInt16( seg[segIndex], i16 );
            cfg.io.inCfg[ 1] = (INPUT_PIN_CONFIG)i16;
            break;

         case 0x07a: 
            err = StrToInt16( seg[segIndex], i16 );
            cfg.io.inCfg[ 2] = (INPUT_PIN_CONFIG)i16;
            break;

         case 0x07b:
            err = StrToInt16( seg[segIndex], i16 );
            cfg.io.inCfg[ 3] = (INPUT_PIN_CONFIG)i16;
            break;

         case 0x07c: 
            err = StrToInt16( seg[segIndex], i16 );
            cfg.io.inCfg[ 4] = (INPUT_PIN_CONFIG)i16;
            break;

         case 0x07d:
            err = StrToInt16( seg[segIndex], i16 );
            cfg.io.inCfg[ 5] = (INPUT_PIN_CONFIG)i16;
            break;

         case 0x07e:
            err = StrToInt16( seg[segIndex], i16 );
            cfg.io.inCfg[ 6] = (INPUT_PIN_CONFIG)i16;
            break;

         case 0x07f: 
            err = StrToInt16( seg[segIndex], i16 );
            cfg.io.inCfg[ 7] = (INPUT_PIN_CONFIG)i16;
            break;

         // This group of parameters are in the ccx file
         // for reference only.  They describe read-only
         // parameters of the amplifier.
         case 0x80: case 0x81: case 0x82: case 0x83:
         case 0x84: case 0x85: case 0x86: case 0x87:
         case 0x88: case 0x89: case 0x8A: case 0x8B:
         case 0x8C: case 0x8D: case 0x8E: case 0xAD:
            break;

         case 0x092: 
            strncpy( cfg.name, seg[segIndex], COPLEY_MAX_STRING );
            break;

         case 0x093: 
            err = StrToInt16( seg[segIndex], i16 );
            cfg.CME_Config[0] = (char)(i16>>8);
            cfg.CME_Config[1] = (char)(i16);
            break;

         case 0x095:
            err = StrToHostCfg( seg[segIndex], cfg.CME_Config );
            break;

         case 0x098: err = StrToInt16( seg[segIndex], cfg.fgen.cfg  ); break;
         case 0x099: err = StrToInt16( seg[segIndex], cfg.fgen.freq ); break;
         case 0x09a: err = StrToInt32( seg[segIndex], cfg.fgen.amp  ); break;
         case 0x09b: err = StrToInt16( seg[segIndex], cfg.fgen.duty ); break;
         case 0x0A5: err = StrToUInt16( seg[segIndex], cfg.io.inPullUpCfg ); break;

         case 0x0A7: 
            err = StrToInt32( seg[segIndex], i32 ); 
            cfg.faultMask = (AMP_FAULT)i32; 
            break;

         case 0x0A8: err = StrToInt16 ( seg[segIndex], cfg.pwmIn.cfg    ); break;
         case 0x0A9: err = StrToInt32 ( seg[segIndex], cfg.pwmIn.scale  ); break;
         case 0x0Ae: err = StrToInt16 ( seg[segIndex], cfg.cLoop.offset ); break;
         case 0x0Af: err = StrToUInt32( seg[segIndex], cfg.options      ); break;
         case 0x0B1: err = StrToInt16 ( seg[segIndex], cfg.stepRate     ); break;

         case 0x0B2: 
            err = StrToInt16( seg[segIndex], i16 ); 
            cfg.phaseMode = (AMP_PHASE_MODE)i16; 
            break;

         case 0xB3:
            // Read only
            //err = StrToInt16( seg[segIndex], cfg.);
            break;

         case 0x0B6:
            err = StrToInt16( seg[segIndex], cfg.pwmIn.freq ); 
            break;

         case 0x0B8:
            StrToLoadPos( seg[segIndex], cfg.limit.pos );
            break;

         case 0x0B9:
            StrToLoadPos( seg[segIndex], cfg.limit.neg );
            break;

         case 0x0BA:
            StrToLoadPos( seg[segIndex], cfg.window.trackErr );
            break;

         case 0x0BB:
            StrToLoadPos( seg[segIndex], cfg.window.trackWarn );
            break;

         case 0x0BC:
            StrToLoadPos( seg[segIndex], cfg.window.settlingWin );
            break;

         case 0x0BD: 
            err = StrToUInt16( seg[segIndex], cfg.window.settlingTime ); 
            break;

         case 0x0BE:
            StrToLoadAcc( seg[segIndex], cfg.limit.accel );
            break;

         case 0x0BF:
            err = StrToInt16( seg[segIndex], cfg.home.delay );
            break;

         case 0x0C2:
            err = StrToUInt16( seg[segIndex], u16 );
            cfg.home.extended = u16;
            cfg.home.method = HomeMethodConvert( u16 );
            break;

         case 0x0C3:
            StrToLoadVel( seg[segIndex], cfg.home.velFast );
            break;

         case 0x0C4:
            StrToLoadVel( seg[segIndex], cfg.home.velSlow );
            break;

         case 0x0C5:
            StrToLoadAcc( seg[segIndex], cfg.home.accel );
            break;

         case 0x0C6:
            StrToLoadPos( seg[segIndex], cfg.home.offset );
            break;

         case 0x0C7:
            err = StrToInt16( seg[segIndex], cfg.home.current );
            break;

         case 0x0C8: 
            // When setting the profile type over the CANopen interface
            // we use an encoding that is consistent with DSP402.  Convert
            // this here:
            err = StrToInt16( seg[segIndex], i16 );
            if( err ) break;

            switch( i16 & 7 )
            {
               case 0: cfg.profile.type = PROFILE_TRAP;   break;
               case 1: cfg.profile.type = PROFILE_SCURVE; break;
               case 2: cfg.profile.type = PROFILE_VEL;    break;
               case 3: cfg.profile.type = PROFILE_TRAP;   break;
			   // if set to csp mode convert it to trap - csp is illegal
			   // value for object id 0x6086 - also this mode gets set
			   // by etherCAT master.
               case 4: cfg.profile.type = PROFILE_TRAP;   break;
               default:
                  err = &AmpFileError::range;
                  break;
            }
            break;

         case 0x0CA: StrToLoadPos( seg[segIndex], cfg.profile.pos   ); break;
         case 0x0CB: StrToLoadVel( seg[segIndex], cfg.profile.vel   ); break;
         case 0x0CC: StrToLoadAcc( seg[segIndex], cfg.profile.acc   ); break;
         case 0x0CD: StrToLoadAcc( seg[segIndex], cfg.profile.dec   ); break;
         case 0x0CE: StrToLoadJrk( seg[segIndex], cfg.profile.jrk   ); break;
         case 0x0CF: StrToLoadAcc( seg[segIndex], cfg.profile.abort ); break;

         case 0x0D0: 
            err = StrToInt16( seg[segIndex], i16 ); 
            cfg.io.inCfg[ 8] = (INPUT_PIN_CONFIG)i16; 
            break;

         case 0x0D1: 
            err = StrToInt16( seg[segIndex], i16 ); 
            cfg.io.inCfg[ 9] = (INPUT_PIN_CONFIG)i16;
            break;

         case 0x0D2:
            err = StrToInt16( seg[segIndex], i16 );
            cfg.io.inCfg[10] = (INPUT_PIN_CONFIG)i16;
            break;

         case 0x0D3:
            err = StrToInt16( seg[segIndex], i16 );
            cfg.io.inCfg[11] = (INPUT_PIN_CONFIG)i16;
            break;

         case 0x0D4: 
            err = StrToInt16( seg[segIndex], i16 );
            cfg.io.inCfg[ 12] = (INPUT_PIN_CONFIG)i16;
            break;

         case 0x0D5: 
            err = StrToInt16( seg[segIndex], i16 );
            cfg.io.inCfg[ 13] = (INPUT_PIN_CONFIG)i16;
            break;

         case 0x0D6: 
            err = StrToInt16( seg[segIndex], i16 );
            cfg.io.inCfg[ 14] = (INPUT_PIN_CONFIG)i16;
            break;

         case 0x0D7: 
            err = StrToInt16( seg[segIndex], i16 );
            cfg.io.inCfg[ 15] = (INPUT_PIN_CONFIG)i16;
            break;

         case 0x0D8: 
            err = StrToUInt16( seg[segIndex], cfg.regen.resistance ); 
            break;

         case 0x0D9: 
            err = StrToUInt16( seg[segIndex], cfg.regen.contPower ); 
            break;

         case 0x0DA: 
            err = StrToUInt16( seg[segIndex], cfg.regen.peakPower ); 
            break;

         case 0x0DB: 
            err = StrToUInt16( seg[segIndex], cfg.regen.peakTime ); 
            break;

         case 0x0DC: 
            err = StrToUInt16( seg[segIndex], cfg.regen.vOn ); 
            break;

         case 0x0DD: 
            err = StrToUInt16( seg[segIndex], cfg.regen.vOff ); 
            break;

         case 0x0E1: 
            strncpy( cfg.regen.model, seg[segIndex], COPLEY_MAX_STRING );
            break;

         case 0x0E3:
            err = StrToInt16( seg[segIndex], cfg.pLoop.scale  );
            break;

         case 0x0E4:
            err = StrToUInt16( seg[segIndex], cfg.algoPhaseInit.phaseInitCurrent );
            break;

         case 0x0E5:
            err = StrToUInt16( seg[segIndex], cfg.algoPhaseInit.phaseInitTime );
            break;

         case 0x0E6: err = StrToUInt32( seg[segIndex], cfg.ustep.maxVelAdj ); break;
         case 0x0E7: err = StrToUInt16( seg[segIndex], cfg.ustep.ustepPGainOutLoop ); break;
         case 0x0E8: err = StrToUInt16( seg[segIndex], cfg.cLoop.stepHoldCurrent ); break;
         case 0x0E9: err = StrToUInt16( seg[segIndex], cfg.cLoop.stepRun2HoldTime ); break;
         case 0x0EA: err = StrToInt16( seg[segIndex], cfg.ustep.detentCorrectionGain ); break;
         case 0x0ED: err = StrToUInt16( seg[segIndex], cfg.cLoop.stepVolControlDelayTime ); break;
         case 0x0EE: err = StrToUInt16( seg[segIndex], cfg.ustep.ustepConfigAndStatus ); break;

         case 0x0F0: err = StrToInt16( seg[segIndex], cfg.io.inDebounce[ 0] ); break;
         case 0x0F1: err = StrToInt16( seg[segIndex], cfg.io.inDebounce[ 1] ); break;
         case 0x0F2: err = StrToInt16( seg[segIndex], cfg.io.inDebounce[ 2] ); break;
         case 0x0F3: err = StrToInt16( seg[segIndex], cfg.io.inDebounce[ 3] ); break;
         case 0x0F4: err = StrToInt16( seg[segIndex], cfg.io.inDebounce[ 4] ); break;
         case 0x0F5: err = StrToInt16( seg[segIndex], cfg.io.inDebounce[ 5] ); break;
         case 0x0F6: err = StrToInt16( seg[segIndex], cfg.io.inDebounce[ 6] ); break;
         case 0x0F7: err = StrToInt16( seg[segIndex], cfg.io.inDebounce[ 7] ); break;
         case 0x0F8: err = StrToInt16( seg[segIndex], cfg.io.inDebounce[ 8] ); break;
         case 0x0F9: err = StrToInt16( seg[segIndex], cfg.io.inDebounce[ 9] ); break;
         case 0x0FA: err = StrToInt16( seg[segIndex], cfg.io.inDebounce[10] ); break;
         case 0x0FB: err = StrToInt16( seg[segIndex], cfg.io.inDebounce[11] ); break;
         case 0x0FC: err = StrToInt16( seg[segIndex], cfg.io.inDebounce[12] ); break;
         case 0x0FD: err = StrToInt16( seg[segIndex], cfg.io.inDebounce[13] ); break;
         case 0x0FE: err = StrToInt16( seg[segIndex], cfg.io.inDebounce[14] ); break;
         case 0x0FF: err = StrToInt16( seg[segIndex], cfg.io.inDebounce[15] ); break;

         case 0x103:
            err = StrToUInt32( seg[segIndex], cfg.can.pinMapping );
            break;

         case 0x104:
            err = StrToUInt16( seg[segIndex], cfg.algoPhaseInit.phaseInitConfig );
            break;

         case 0x105:
            err = StrToUInt16( seg[segIndex], cfg.camming.cammingModeConfig );
            break;

         case 0x106:
            err = StrToUInt16( seg[segIndex], cfg.camming.cammingDelayForward );
            break;

         case 0x107:
            err = StrToUInt16( seg[segIndex], cfg.camming.cammingDelayReverse );
            break;

         case 0x109:
            err = StrToInt32( seg[segIndex], cfg.camming.cammingMasterVel );
            break;

         case 0x10C:
            err = StrToUInt16( seg[segIndex], cfg.can.heartbeat );
            break;

         case 0x10D:
            err = StrToUInt16( seg[segIndex], cfg.can.nodeGuard );
            break;

         case 0x10E:
            err = StrToUInt16( seg[segIndex], cfg.can.nodeGuardLife );
            break;

         case 0x114: err = StrToInt16( seg[segIndex], cfg.vLoop.viDrain ); break;

         case 0x116:
            err = StrToUInt16( seg[segIndex], cfg.can.quickStop );
            break;

         case 0x117:
            err = StrToUInt16( seg[segIndex], cfg.can.shutDownOption );
            break;

         case 0x118:
            err = StrToUInt16( seg[segIndex], cfg.can.disableOption );
            break;

         case 0x119:
            err = StrToUInt16( seg[segIndex], cfg.can.haltOption );
            break;

		 case 0x11D:
			 err = StrToUInt32( seg[segIndex], cfg.errFilterCfg.encoderErrorConfig );
			 break;

         case 0x121:
            err = StrToUInt16( seg[segIndex], cfg.netOptions.canBusConfig );
            break;

         case 0x123: StrToLoadPos( seg[segIndex], cfg.limit.motorPosWrap ); break;
         case 0x124: StrToLoadPos( seg[segIndex], cfg.limit.loadPosWrap ); break;

         case 0x127: 
            err = StrToUInt32( seg[segIndex], cfg.gainSched.gainSchedulingConfig );
            break;

         case 0x12A: err = StrToUInt32( seg[segIndex], cfg.motor.mtrEncOptions ); break;
         case 0x12B: err = StrToUInt32( seg[segIndex], cfg.motor.loadEncOptions ); break;

         case 0x12D: 
            err = StrToFilter( seg[segIndex], cfg.aInCmdFltr );
            break;

         case 0x134: 
            err = StrToInt32( seg[segIndex], cfg.daConfig.daConverterConfig ); 
            break;

         case 0x13B:
            err = StrToInt16( seg[segIndex], cfg.motor.overTempLimit );
            break;

         case 0x13C:
            err = StrToInt16( seg[segIndex], cfg.pwmIn.minPulseWidth );
            break;

         case 0x13D:
            err = StrToInt16( seg[segIndex], cfg.pwmIn.maxPulseWidth );
            break;
			
		 case 0x13F:
			err = StrToInt16( seg[segIndex], cfg.pwmIn.deadBand );
			break;

         case 0x150:
            err = StrToFilter( seg[segIndex], cfg.vloopOutFltr2 );
            break;

         case 0x151:
            err = StrToFilter( seg[segIndex], cfg.vloopOutFltr3 );
            break;

         case 0x152:
            err = StrToFilter( seg[segIndex], cfg.iloopCmdFltr );
            break;

         case 0x153:
            err = StrToFilter( seg[segIndex], cfg.iloopCmdFltr2 );
            break;

         case 0x154:
            err = StrToInt32( seg[segIndex], cfg.servoConfig.servoLoopConfig );
            break;

         case 0x155:
            err = StrToInt16( seg[segIndex], cfg.pLoop.ki );
            break;

         case 0x156:
            err = StrToInt16( seg[segIndex], cfg.pLoop.kd );
            break;

         case 0x157:
            err = StrToInt16( seg[segIndex], cfg.vLoop.velCmdff );
            break;

         case 0x158:
            err = StrToInt16( seg[segIndex], cfg.pLoop.kiDrain );
            break;

         case 0x15A:
            err = StrToInt32( seg[segIndex], cfg.io.ioOptions );
            break;

         case 0x15B:
            err = StrToInt16( seg[segIndex], cfg.motor.brakeEnableDelay );
            break;

         case 0x15E:
            err = StrToInt32( seg[segIndex], cfg.io.inPullUpCfg32 );
            break;

         case 0x160: 
            err = StrToInt16( seg[segIndex], i16 ); 
            cfg.io.inCfg[16] = (INPUT_PIN_CONFIG)i16; 
            break;

         case 0x161: 
            err = StrToInt16( seg[segIndex], i16 ); 
            cfg.io.inCfg[17] = (INPUT_PIN_CONFIG)i16;
            break;

         case 0x162:
            err = StrToInt16( seg[segIndex], i16 );
            cfg.io.inCfg[18] = (INPUT_PIN_CONFIG)i16;
            break;

         case 0x163:
            err = StrToInt16( seg[segIndex], i16 );
            cfg.io.inCfg[19] = (INPUT_PIN_CONFIG)i16;
            break;

         case 0x164: 
            err = StrToInt16( seg[segIndex], i16 );
            cfg.io.inCfg[20] = (INPUT_PIN_CONFIG)i16;
            break;

         case 0x165: 
            err = StrToInt16( seg[segIndex], i16 );
            cfg.io.inCfg[21] = (INPUT_PIN_CONFIG)i16;
            break;

         case 0x166: 
            err = StrToInt16( seg[segIndex], i16 );
            cfg.io.inCfg[22] = (INPUT_PIN_CONFIG)i16;
            break;

         case 0x167: 
            err = StrToInt16( seg[segIndex], i16 );
            cfg.io.inCfg[23] = (INPUT_PIN_CONFIG)i16;
            break;

         case 0x168: 
            err = StrToInt16( seg[segIndex], i16 );
            cfg.io.inCfg[24] = (INPUT_PIN_CONFIG)i16;
            break;

         case 0x169: 
            err = StrToInt16( seg[segIndex], i16 );
            cfg.io.inCfg[25] = (INPUT_PIN_CONFIG)i16;
            break;

         case 0x170: err = StrToInt16( seg[segIndex], cfg.io.inDebounce[16] ); break;
         case 0x171: err = StrToInt16( seg[segIndex], cfg.io.inDebounce[17] ); break;
         case 0x172: err = StrToInt16( seg[segIndex], cfg.io.inDebounce[18] ); break;
         case 0x173: err = StrToInt16( seg[segIndex], cfg.io.inDebounce[19] ); break;
         case 0x174: err = StrToInt16( seg[segIndex], cfg.io.inDebounce[20] ); break;
         case 0x175: err = StrToInt16( seg[segIndex], cfg.io.inDebounce[21] ); break;
         case 0x176: err = StrToInt16( seg[segIndex], cfg.io.inDebounce[22] ); break;
         case 0x177: err = StrToInt16( seg[segIndex], cfg.io.inDebounce[23] ); break;
         case 0x178: err = StrToInt16( seg[segIndex], cfg.io.inDebounce[24] ); break;
         case 0x179: err = StrToInt16( seg[segIndex], cfg.io.inDebounce[25] ); break;

         case 0x180:
            err = StrToInt32( seg[segIndex], cfg.pwmIn.uvCfg );
            break;

         case 0x184:
            err = StrToInputShaper( seg[segIndex], cfg.inputShaping );
            break;

         case 0x18F: 
            err = StrToInt16( seg[segIndex], cfg.motor.encSinOffset ); 
            break;

         case 0x190: 
            err = StrToInt16( seg[segIndex], cfg.motor.encCosOffset ); 
            break;

         case 0x191: 
            err = StrToInt16( seg[segIndex], cfg.motor.encCosScale ); 
            break;

         case 0x192:
            err = StrToUInt32( seg[segIndex], cfg.motor.mtrEncCal ); 
            break;

         case 0x193: 
            err = StrToUInt32( seg[segIndex], cfg.motor.ldEncCal ); 
            break;

		 case 0x195:
			 err = StrToUInt32(seg[segIndex], cfg.trjJrkAbort);
			 break;

         case 0x197: 
            err = StrToInt16( seg[segIndex], cfg.pLoop.xKp ); 
            break;

         case 0x198: 
            err = StrToInt16( seg[segIndex], cfg.pLoop.xKi ); 
            break;

         case 0x199: 
            err = StrToInt16( seg[segIndex], cfg.pLoop.xKd ); 
            break;

         case 0x19D: 
            err = StrToInt16( seg[segIndex], cfg.motor.openMtrCrnt ); 
            break;

         case 0x1A0:
            err = StrToOutCfg( seg[segIndex], cfg.io.outCfg[8], cfg.io.outMask[8], cfg.io.outMask1[8]  );
            break;

         case 0x1A1:
            err = StrToOutCfg( seg[segIndex], cfg.io.outCfg[9], cfg.io.outMask[9], cfg.io.outMask1[9]  );
            break;

         case 0x1A2:
            err = StrToOutCfg( seg[segIndex], cfg.io.outCfg[10], cfg.io.outMask[10], cfg.io.outMask1[10]  );
            break;

         case 0x1A3:
            err = StrToOutCfg( seg[segIndex], cfg.io.outCfg[11], cfg.io.outMask[11], cfg.io.outMask1[11]  );
            break;

         default:
            cml.Debug( "Unknown paramaeter in CCX file: 0x%02x\n", param );
            break;
      }
   }

   fclose(fp);

   if( err ) return err;

   // The file was read in successfully.  Now, upload the configuration
   // structure to the amplifier.
   return SetAmpConfig( cfg );
#endif
}

/***************************************************************************/
/**
Convert the homing method from the 16-bit value in the ccx file to a CANopen
standard homing method.
*/
/***************************************************************************/
static COPLEY_HOME_METHOD HomeMethodConvert( uint16 x )
{
   switch( x )
   {
      case 0x0031: return CHM_NLIM_ONDX;
      case 0x0021: return CHM_PLIM_ONDX;
      case 0x0022: return CHM_PHOME_ONDX;
      case 0x0062: return CHM_PHOME_INDX;
      case 0x0032: return CHM_NHOME_ONDX;
      case 0x0072: return CHM_NHOME_INDX;
      case 0x0123: return CHM_LHOME_ONDX_POS;
      case 0x0163: return CHM_LHOME_INDX_POS;
      case 0x0063: return CHM_UHOME_INDX_POS;
      case 0x0023: return CHM_UHOME_ONDX_POS;
      case 0x0033: return CHM_UHOME_ONDX_NEG;
      case 0x0073: return CHM_UHOME_INDX_NEG;
      case 0x0173: return CHM_LHOME_INDX_NEG;
      case 0x0133: return CHM_LHOME_ONDX_NEG;
      case 0x0011: return CHM_NLIM;
      case 0x0001: return CHM_PLIM;
      case 0x0002: return CHM_PHOME;
      case 0x0012: return CHM_NHOME;
      case 0x0103: return CHM_LHOME_POS;
      case 0x0003: return CHM_UHOME_POS;
      case 0x0013: return CHM_UHOME_NEG;
      case 0x0113: return CHM_LHOME_NEG;
      case 0x0030: return CHM_NDX_NEG;
      case 0x0020: return CHM_NDX_POS;
      case 0x0000: return CHM_NONE;
      case 0x0004: return CHM_HARDSTOP_POS;
      case 0x0014: return CHM_HARDSTOP_NEG;
      case 0x0024: return CHM_HARDSTOP_ONDX_POS;
      case 0x0034: return CHM_HARDSTOP_ONDX_NEG;
      default:     return CHM_EXTENDED;
   }
}


/***************************************************************************/
/**
Checks the network type and calls the correct function 
to write the ccd file to the amplifier.

@param name Name of the file to be passed.
@param net Reference to the network object.
@return The most recent error code returned by this node.
*/
/***************************************************************************/
const Error *Amp::LoadCCDFromFile(const char* name, Network &net)
{
   const Error *err = NULL;
   // Disable the drive before writing to flash.
   err = Disable();
   if (err) 
      return err;

   // Determine which loading protocol to use. (FoE or serial binary)
   if (GetNetworkType() == NET_TYPE_ETHERCAT)
   {
      // Check the firmware version.
      // Firmware versions prior to 2.72 can occasionally fail with a bad parameter
      // causing the drive to enter a bad state which CME2 cannot communicate with.
      // This firmware version check is specifically for FPGA drives (tested on a BE2).
      // This problem has not been encountered on ARM drives.

      switch (hwType & 0xFF00)
      {
         case 0x1000:
            // Has an FPGA.
            if (SwVersionNum < VERSION(2, 72))
               return &AmpFileError::FPGAtooOld;
            break;

         case 0x1200:
            // Has an FPGA.
            if (SwVersionNum < VERSION(2, 72))
               return &AmpFileError::FPGAtooOld;
            break;
      }

      err = WriteCCDToEcatDrive(name, (EtherCAT&)net, *this);
   }
   else if (GetNetworkType() == NET_TYPE_CANOPEN)
   {
      err = WriteCCDToCANDrive(name, (CanOpen&)net, *this);
   }
   if (err)
      return err;

   // Delay to ensure flash writing has finished.
   Thread::sleep(200);

   return err;
}

/***************************************************************************/
/**
Reads the ccd file from the drive using File Access over EtherCAT
and creates a file.

@param name Name of the file to be created.
@param net Reference to the EtherCAT network object.
@param amp Reference to the amplifier.
@return The most recent error code returned by this node.
*/
/***************************************************************************/
const Error *WriteCCDToEcatDrive(const char* name, EtherCAT &net, Amp &amp)
{
	int timeoutCt = 0;
	// Read the entire file into an array.
	FILE *fp;
	fp = fopen(name, "rt");
	if (!fp)
		return &AmpFileError::fileOpen;
	
	long size = 0;
	size = GetCCDFileSize(fp);
	byte* FileData = new byte[size + 1];
	char temp;
	bool endOfFile = false;

	for (int k = 0; k < size + 1; k++)
	{
		temp = getc(fp);

		FileData[k] = temp;

		if (temp == EOF)
			break;
	}
	fclose(fp);

	// Start to download the file to the drive.
	const Error *err = net.FoE_DnldStart(&amp, name, 0, 2000);
	if (err)
	{
		// Free the memory before returning.
		if (FileData)
		{
			delete[] FileData;
		}

		return err;
	}

	// Format the data before sending.
	char buff[244];
	int max = 244;
	int length = 0;

	for (int j = 0; ; j++)
	{
		length = 0;

		for (int k = 0; k < max; k++)
		{
			if (((j * 244) + k == size))
			{
				endOfFile = true;
				break;
			}
			temp = FileData[(j * 244) + k];
			buff[k] = temp;
			length++;
		}

		if (endOfFile)
		{
			break;
		}

		// Send the data.
		err = net.FoE_DnldData(&amp, length, (uint8*)buff, 2000, false);
		if (err == &ThreadError::Timeout)
		{
			// If there is a timeout, attempt to continue sending data.
			// The only timeouts that have occurred were caused by missing the ACKs.
			timeoutCt++;
			if (timeoutCt > 5)
			{
				// Free the memory before returning.
				if (FileData)
				{
					delete[] FileData;
				}

				return err;
			}
			else
				err = NULL;
		}
		if (err)
		{
			// Free the memory before returning.
			if (FileData)
			{
				delete[] FileData;
			}

			return err;
		}

		// Clear the data.
		for (int q = 0; q < max; q++)
			buff[q] = 0;
	}

	// Send the last group of data.
	err = net.FoE_DnldData(&amp, length, (uint8*)buff, 2000, true);
	if (err == &ThreadError::Timeout)
		err = NULL;
	// Free the memory before returning.
	if (FileData)
	{
		delete[] FileData;
	}
	return err;
}

/***************************************************************************/
/**
Writes the ccd file to the drive using serial binary over CANopen.

@param name Name of the file to be passed.
@param net Reference to the CANopen network object.
@param amp Reference to the amplifier.
@return The most recent error code returned by this node.
*/
/***************************************************************************/
const Error *WriteCCDToCANDrive(const char *name, CanOpen &net, Amp &amp)
{
	// Open the file. CANopen serial binary****************************
	FILE *fp;
	fp = fopen(name, "rb");
	if (!fp)
		return &AmpFileError::fileOpen;
	int max = 100;
	uint8 length = max;
	uint16 dataWord[100];
	uint16 dataWordSent[100]; 
	uint16 dataWordEnd[100];
	uint8 lastSerialError = 0;

	dataWordEnd[0] = 6;

	/********************CANopen writing to the drive*************************/
	// Send initial SDO to initiate writing to the drive.
	dataWord[0] = 2;

	// Read the entire file into an array.
	bool endOfFile = false;
	long size = 0;
	size = GetCCDFileSize(fp);
	char* FileData = new char[size];
	char temp[2];
	temp[1] = '\0';

	for (int k = 0; k < size; k++)
	{
		temp[0] = getc(fp);

		if (temp[0] == EOF)
		{
			break;
		}
		FileData[k] = temp[0];
	}
	// Once the data is read, close the files.
	fclose(fp);
	uint8 opcode = 33; // dynamic file command.
	
	// Send the initial SDO to initiate writing to the drive.
	for (int i = 1; i < 100; i++)
	{
		dataWord[i] = '\0';
	}
	length = 1;
	const Error *err = amp.SerialCmd(opcode, length, 100, dataWord);

	if (err == &CopleyNodeError::SerialError)
	{
		lastSerialError = amp.GetLastSerialError();
		while (lastSerialError == 0x32)
		{
			// 0x32 is a timeout. Resend the message.
			err = amp.SerialCmd(opcode, length, 100, dataWord);
			lastSerialError = amp.GetLastSerialError();
		}
	}

	if (err)
	{
		// End the file transfer.
		// This message should be sent after any file transfer whether
		// successful or not.
		length = 1;
		amp.SerialCmd(opcode, length, 100, dataWordEnd);
		// Free the memory before returning.
		if (FileData)
		{
			delete[] FileData;
		}
		return err;
	}


	/***********************************/
	// Convert the data into 16-bit words instead of bytes.
	uint16* FileWordsUInt = new uint16[size / 2];
	for (int j = 0; j < size / 2; j++)
	{
		FileWordsUInt[j] = (FileData[2 * j] << 8) | FileData[2 * j + 1];
	}

	// Break the file data up into groups of no more than 100 words.
	// 2 words are being used already for sub-function and length
	uint16 tempWord = 0;
	for (int j = 0; ; j++)
	{
		length = 0;
		max = 98;
		
		for (int k = 0; k < max; k++)
		{
			if (((j * 98) + k) == (size / 2))
			{
				endOfFile = true;
				break;
			}
			tempWord = FileWordsUInt[(j * 98) + k];
			dataWord[k] = tempWord;
			length++;
		}

		// Add the byte swapped data to the data being sent.
		// dataSent order:
		// First word: sub-function 3
		// Second word: number of bytes passed
		dataWordSent[0] = 3;
		dataWordSent[1] = (length + 2) * 2;
		for (int pq = 0; pq < max; pq++)
		{
			dataWordSent[pq + 2] = dataWord[pq];
		}

		// Clear the data.
		for (int q = 0; q < max; q++)
		{
			dataWord[q] = '\0';
		}

		if (endOfFile)
			break;

		// Send the data.
		length += 2;
		err = amp.SerialCmd(opcode, length, 100, dataWordSent);
		if (err == &CopleyNodeError::SerialError)
		{
			// Check the serial error code.
			lastSerialError = amp.GetLastSerialError();

			while (lastSerialError == 0x32)
			{
				// This is a timeout error. Resend the message.
				err = amp.SerialCmd(opcode, length, 100, dataWordSent);
				lastSerialError = amp.GetLastSerialError();
			}
		}

		if (err)
		{
			// End the file transfer.
			// This message should be sent after any file transfer whether
			// successful or not.
			length = 1;
			amp.SerialCmd(opcode, length, 100, (uint16*)dataWordEnd);

			// Free the memory before returning.
			if (FileData)
			{
				delete[] FileData;
			}

			if (FileWordsUInt)
			{
				delete[] FileWordsUInt;
			}
			return err;
		}

		max = 98;

		// Clear the data.
		for (int q = 0; q < max; q++)
		{
			dataWord[q] = 0;
		}
	}

	// Send the last group of data.
	length += 2;
	err = amp.SerialCmd(opcode, length, 100, dataWordSent);
	if (err == &CopleyNodeError::SerialError)
	{
		// Check the serial error code.
		lastSerialError = amp.GetLastSerialError();

		while (lastSerialError == 0x32)
		{
			// This is a timeout error. Resend the message.
			err = amp.SerialCmd(opcode, length, 100, dataWordSent);
			lastSerialError = amp.GetLastSerialError();
		}
	}

	if (err)
	{
		// End the file transfer.
		// This message should be sent after any file transfer whether
		// successful or not.
		length = 1;
		amp.SerialCmd(opcode, length, 100, (uint16*)dataWordEnd);

		// Free the memory before returning.
		if (FileData)
		{
			delete[] FileData;
		}

		if (FileWordsUInt)
		{
			delete[] FileWordsUInt;
		}
		return err;
	}

	//// End the file transfer.
	//// This message should be sent after any file transfer whether
	//// successful or not.
	length = 1;
	err = amp.SerialCmd(opcode, length, 100, dataWordEnd);

	// Free the memory before returning.
	if (FileData)
	{
		delete[] FileData;
	}

	if (FileWordsUInt)
	{
		delete[] FileWordsUInt;
	}

	return err;
}

/***************************************************************************/
/**
Checks the network type and calls the correct function
to read the ccd file from the amplifier.

@param name Name of the file to be created.
@param net Reference to the network object.
@return The most recent error code returned by this node.
*/
/***************************************************************************/
const Error* Amp::WriteCCDToFile(const char* name, Network& net)
{
    const Error* err = NULL;

    // Disable the drive before writing to flash.
    err = Disable();
    if (err) { return err; }

    // Determine which loading protocol to use. (FoE or serial binary)
    if (GetNetworkType() == NET_TYPE_ETHERCAT)
    {
        // Check the firmware version.
        // Firmware versions prior to 2.72 can occasionally fail with a bad parameter
        // causing the drive to enter a bad state which CME2 cannot communicate with.
        // This firmware version check is specifically for FPGA drives (tested on a BE2).
        // This problem has not been encountered on ARM drives.

        switch (hwType & 0xFF00)
        {
        case 0x1000:
            // Has an FPGA.
            if (SwVersionNum < VERSION(2, 72))
                return &AmpFileError::FPGAtooOld;
            break;

        case 0x1200:
            // Has an FPGA.
            if (SwVersionNum < VERSION(2, 72))
                return &AmpFileError::FPGAtooOld;
            break;
        }

        err = ReadCCDFromDrive(name, (EtherCAT&)net, *this);
    }
    else if (GetNetworkType() == NET_TYPE_CANOPEN)
    {
        err = ReadCCDFromDrive(name, (CanOpen&)net, *this);
    }
    if (err) { return err; }

    // Delay to ensure flash writing has finished.
    Thread::sleep(200);

    return err;
}

/**
Reads the ccd file from the drive using serial binary and creates a file.

@param name Name of the file to be created.
@param net Reference to the network object.
@param amp Reference to the Amp object.
@return The most recent error code returned by this node.
*/
const Error* ReadCCDFromDrive(const char* name, Network& net, Amp& amp)
{
    FILE* fp;
    fp = fopen(name, "wt");
    short index = 0x2000;
    short sub = 0;
    uint8 size;
    int max = 201;
    char data[202];
    char dataRequest[3];
    char dataPrint[199];
    char dataEnd[3];
    int simpleSize = 3;
    dataRequest[0] = 0x21; // op code 33 for dynamic file transfer.
    dataRequest[1] = 0x04; // sub-function code 4 for read data from file.
    dataRequest[2] = 0x00; // the rest of the word.
    dataEnd[0] = 0x21; // op code 33 for dynamic file transfer.
    dataEnd[1] = 0x06; // sub-function code 6 for finish the file transfer/
    dataEnd[2] = 0x00; // the rest of the word.
    for (int q = 0; q < 202; q++)
    {
        data[q] = '\0';
    }

    cml.Debug("Creating file: %s\n", name);

    /************Read from the file***********************/
    // Send the initial SDO.
    // bytes are swapped in each word.
    data[0] = 0x21; // op code 33 for dynamic file command.
    data[1] = 0x01; // sub-function code 1 for start reading a file.
    data[2] = 0x00; // the rest of the word.
    // The rest of the data for this message is the name of the file.
    data[3] = 0x72; // "r"d
    data[4] = 0x64; // "d"r
    data[5] = 0x76; // "v"i
    data[6] = 0x69; // "i"v
    data[7] = 0x63; // "c"e
    data[8] = 0x65; // "e"c
    data[9] = 0x6E; // "n"o
    data[10] = 0x6F; // "o"n
    data[11] = 0x69; // "i"f
    data[12] = 0x66; // "f"i
    data[13] = 0x2E; // "."g
    data[14] = 0x67; // "g".
    data[15] = 0x63; // "c"c
    data[16] = 0x63; // "c"c
    data[17] = 0x00; //    d
    data[18] = 0x64; // "d"
    size = 19;

    const Error* err = amp.Download(index, sub, size, (uint8*)data);
    if (err) {
        return err;
    }

    int length = max;

    for (;;)
    {
        // Request the data from the drive.
        err = amp.Download(index, sub, simpleSize, (byte*)dataRequest);
        if (err)
        {
            fclose(fp);

            // End the file transfer.
            // This message should be sent after any file transfer whether
            // successful or not.
            amp.Download(index, sub, simpleSize, (byte*)dataEnd);
            return err;
        }

        length = max;
        err = amp.Upload(index, sub, length, (byte*)data);
        if (err)
        {
            fclose(fp);

            // End the file transfer.
            // This message should be sent after any file transfer whether
            // successful or not.
            amp.Download(index, sub, simpleSize, (byte*)dataEnd);
            return err;
        }

        for (int pq = 0; pq < max - 2; pq++)
        {
            dataPrint[pq] = data[pq + 3];
        }

        if (length != max) { break; }

        // Printing
        fputs((char*)dataPrint, fp);

        for (int q = 2; q < max; q++)
        {
            data[q] = 0;
            if (q < max - 2) {
                dataPrint[q] = 0;
            }
        }
    }
    fputs((char*)dataPrint, fp);

    //// End the file transfer.
    //// This message should be sent after any file transfer whether
    //// successful or not.
    err = amp.Download(index, sub, simpleSize, (byte*)dataEnd);
    if (err) {
        return err;
    }

    cml.Debug("Finished creating file.\n");
    fclose(fp);

    return err;
}

/***************************************************************************/
/**
Gets the file size. The passed file needs to be opened in binary mode.

@param fp opened file to be passed.
@return The size of the file in bytes.
*/
/***************************************************************************/
long GetCCDFileSize(FILE* fp)
{
	long size;

	// Seek the end of the file.
	fseek(fp, 0, SEEK_END);
	// Get the current position in bytes.
	size = ftell(fp);
	// Return to the beginning of the file to begin reading.
	rewind(fp);
	return size;
}
