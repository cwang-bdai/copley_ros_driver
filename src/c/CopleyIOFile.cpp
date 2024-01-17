/********************************************************/
/*                                                      */
/*  Copley Motion Libraries                             */
/*                                                      */
/*  Copyright (c) 2010 Copley Controls Corp.            */
/*                     http://www.copleycontrols.com    */
/*                                                      */
/********************************************************/

/***************************************************************************
 File: IOFile.cpp                                                        
                                                                         
 This file contains code used to read a CME-2 .cci I/O file.             
                                                                     
***************************************************************************/


#include "CML.h"

#ifdef CML_FILE_ACCESS_OK
#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#endif


CML_NAMESPACE_USE();

CML_NEW_ERROR( IOFileError, bankInvalid,   "I/O bank is invalid" );

/***************************************************************************/
/**
  Load the specified io module data file.  This function presently supports
  loading *.cci files created by the CME-2 program, version 1 and later.
  @param name The name (and optionally path) of the file to load
  @param line If not NULL, the last line number read from the file is returned
  here.  This is useful for finding file format errors.
  @return A pointer to an error object, or NULL on success.
  */
/***************************************************************************/
const Error *CopleyIO::LoadFromFile( const char *name, int &line )
{
#define MAX_LINE_LEN  200
#define MAX_LINE_SEGS 4

   line = 0;
#ifndef CML_FILE_ACCESS_OK
   return &AmpFileError::noFileAccess;
#else
   CopleyIOCfg cfg;

   // Load the configuration structure with the current I/O module
   // configuration data.  This ensures that any parameters not 
   // specified in the file will remain unchanged.
   const Error *err = GetIOCfg( cfg );
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
   int numOfSegs = 4;
   int segIndex = 3;

   int16 fileRev;

   // Read file version number
   line++;
   ReadLine( fp, buff, MAX_LINE_LEN );
   SplitLine( buff, seg, MAX_LINE_SEGS );

   //Third parameter indicates base of returned value (10)
   err = StrToInt16( seg[0], fileRev, 10 ); 
   if( err ) return err;

   if( fileRev > 1 ) 
      return &AmpFileError::format;

   // Read all parameters
   while( !feof(fp) && !err )
   {
      int16 param;
      int16 bank;

      line++;
      ReadLine( fp, buff, MAX_LINE_LEN );

      ct = SplitLine( buff, seg, MAX_LINE_SEGS );
      if( ct == 0 )
         continue;

      if( ct != numOfSegs )
         err = &AmpFileError::format;
      else
      {
          err = StrToInt16(seg[0], param, 16);
          if (err) break;

          err = StrToInt16( seg[1], bank, 16 );
         if( err ) break;

         // do not include the config parameters. There are 10 of them and each has 16 values, not 12.
         if ((bank > 12) && (param != 0x60) && (param != 0x61) && (param != 0x62) &&
             (param != 0x63) && (param != 0x64) && (param != 0x65) && (param != 0x66)
             && (param != 0x67) && (param != 0x68) && (param != 0x69)) {
            return &IOFileError::bankInvalid;
         }
      }

      if( err ) break;

      // handle the programmable filter here
      if (param == 0x04b) {
          int buffIndex{ 0 };
          int nullCount{ 0 };

          // parse the buffer and find the data portion of the char buffer (after the third '\0' char)
          while ((buffIndex < MAX_LINE_LEN) && (nullCount < 3)) {
              if (buff[buffIndex] == '\0') {
                  nullCount = nullCount + 1;
              }
              buffIndex = buffIndex + 1;
          }

          // copy the data portion to a character array.
          char dataPortion[MAX_LINE_LEN];
          int dataPortionIndex{ 0 };
          while ((dataPortionIndex < MAX_LINE_LEN) && (buff[buffIndex] != '\0')) {
              dataPortion[dataPortionIndex] = buff[buffIndex];
              buffIndex = buffIndex + 1;
              dataPortionIndex = dataPortionIndex + 1;
          }

          // end the array with the null character
          dataPortion[dataPortionIndex] = '\0';

          char dataCharArray[COPLEYIO_FILTER_32BIT_WORDS][20];
          int dataIndex{ 0 };
          int dataCount{ 0 };
          int dataCountIndex{ 0 };
          while (dataIndex < dataPortionIndex + 1) {
              if (dataPortion[dataIndex] != ':') {
                  dataCharArray[dataCount][dataCountIndex] = dataPortion[dataIndex];
              }
              else {
                  dataCharArray[dataCount][dataCountIndex] = '\0'; // end the char array with the NULL character.
                  dataCount = dataCount + 1; // move on to the next int32
                  dataCountIndex = -1;        // reset the index for the next int32
              }

              dataIndex = dataIndex + 1;
              dataCountIndex = dataCountIndex + 1;
          }

          for (int i = 0; i < COPLEYIO_FILTER_32BIT_WORDS; i++) {
              err = StrToInt32(dataCharArray[i], cfg.anlg.programmableFilter[bank][i]);
              if (err) return err;
          }
      }
      else {
          switch( param )
          {
             /// Begin general parameters
             case 0x000: err = StrToUInt32( seg[segIndex], cfg.info.serial ); break;
             case 0x001: strncpy( cfg.info.model, seg[segIndex], COPLEY_MAX_STRING ); break;
             case 0x002: strncpy( cfg.info.mfgInfo, seg[segIndex], COPLEY_MAX_STRING ); break;
             case 0x003: err = StrToUInt16( seg[segIndex], cfg.info.hwType ); break;
             case 0x004: err = StrToUInt16( seg[segIndex], cfg.info.loopRate ); break;
             case 0x005: err = StrToUInt32( seg[segIndex], cfg.info.timePwrUp ); break;
             case 0x006: err = StrToUInt16( seg[segIndex], cfg.info.networkOptions ); break;
             case 0x007: err = StrToUInt16( seg[segIndex], cfg.info.ecatAlias ); break;
             case 0x010: err = StrToUInt16( seg[segIndex], cfg.info.fwVersion ); break;
             case 0x011: err = StrToUInt32( seg[segIndex], cfg.info.baud ); break;
             case 0x012: err = StrToUInt16( seg[segIndex], cfg.info.maxWords ); break;
             case 0x013: strncpy( cfg.info.name, seg[segIndex], COPLEY_MAX_STRING ); break;
             case 0x014: err = StrToHostCfg( seg[segIndex], cfg.info.hostCfg ); break;
             case 0x015: err = StrToInt16( seg[segIndex], cfg.info.nodeCfg ); break;
             case 0x016: err = StrToUInt16( seg[segIndex], cfg.info.rateCfg ); break;
             case 0x017: err = StrToUInt16( seg[segIndex], cfg.info.nodeID ); break;
             case 0x018: err = StrToUInt16( seg[segIndex], cfg.info.status ); break;
             case 0x019: err = StrToUInt16( seg[segIndex], cfg.info.rate ); break;
             case 0x01a: err = StrToUInt16( seg[segIndex], cfg.info.anlgInt ); break;
             case 0x01b: err = StrToUInt16( seg[segIndex], cfg.info.anlgIntEna ); break;
             case 0x01c: err = StrToUInt16( seg[segIndex], cfg.info.digiIntEna ); break;
             case 0x01e: err = StrToUInt32( seg[segIndex], cfg.info.pwmPeriodA ); break;
             case 0x01f: err = StrToUInt32( seg[segIndex], cfg.info.pwmPeriodB ); break;

                         /// Begin digital I/O parameters
             case 0x020: err = StrToUInt16( seg[segIndex], cfg.digi.bankMode[bank] ); break;
             case 0x021: err = StrToUInt16( seg[segIndex], cfg.digi.pullupMsk[bank] ); break;
             case 0x022: err = StrToUInt16( seg[segIndex], cfg.digi.typeMsk[bank] ); break;
             case 0x023: err = StrToUInt16( seg[segIndex], cfg.digi.faultMsk[bank] ); break;
             case 0x024: err = StrToUInt16( seg[segIndex], cfg.digi.invMsk[bank] ); break;
             case 0x025: err = StrToUInt16( seg[segIndex], cfg.digi.valueMsk[bank] ); break;
             case 0x026: err = StrToUInt16( seg[segIndex], cfg.digi.modeMsk[bank] ); break;
             case 0x027: err = StrToUInt16( seg[segIndex], cfg.digi.rawMsk[bank] ); break;
             case 0x028: err = StrToUInt16( seg[segIndex], cfg.digi.hiLoMsk[bank] ); break;
             case 0x029: err = StrToUInt16( seg[segIndex], cfg.digi.loHiMsk[bank] ); break;
             case 0x02a: err = StrToUInt16( seg[segIndex], cfg.digi.pullDwnResMask[bank] ); break;
             case 0x030: err = StrToUInt16( seg[segIndex], cfg.digi.debounce0[bank] ); break;
             case 0x031: err = StrToUInt16( seg[segIndex], cfg.digi.debounce1[bank] ); break;
             case 0x032: err = StrToUInt16( seg[segIndex], cfg.digi.debounce2[bank] ); break;
             case 0x033: err = StrToUInt16( seg[segIndex], cfg.digi.debounce3[bank] ); break;
             case 0x034: err = StrToUInt16( seg[segIndex], cfg.digi.debounce4[bank] ); break;
             case 0x035: err = StrToUInt16( seg[segIndex], cfg.digi.debounce5[bank] ); break;
             case 0x036: err = StrToUInt16( seg[segIndex], cfg.digi.debounce6[bank] ); break;
             case 0x037: err = StrToUInt16( seg[segIndex], cfg.digi.debounce7[bank] ); break;

                         /// Begin analog input parameters
             case 0x040: err = StrToUInt16( seg[segIndex], cfg.anlg.iRaw[bank] ); break;
             case 0x041: err = StrToUInt32( seg[segIndex], cfg.anlg.iScaled[bank] ); break;
             case 0x042: err = StrToUInt32( seg[segIndex], cfg.anlg.iFactor[bank] ); break;
             case 0x043: err = StrToUInt32( seg[segIndex], cfg.anlg.iOffset[bank] ); break;
             case 0x044: err = StrToUInt32( seg[segIndex], cfg.anlg.iUpLimit[bank] ); break;
             case 0x045: err = StrToUInt32( seg[segIndex], cfg.anlg.iLoLimit[bank] ); break;
             case 0x046: err = StrToUInt32( seg[segIndex], cfg.anlg.iAbsDelta[bank] ); break;
             case 0x047: err = StrToUInt32( seg[segIndex], cfg.anlg.iPosDelta[bank] ); break;
             case 0x048: err = StrToUInt32( seg[segIndex], cfg.anlg.iNegDelta[bank] ); break;
             case 0x049: err = StrToUInt16( seg[segIndex], cfg.anlg.iFlags[bank] ); break;
             case 0x04a: err = StrToUInt16( seg[segIndex], cfg.anlg.iMask[bank] ); break;

                         /// Begin PWM output parameters
             case 0x050: err = StrToUInt16( seg[segIndex], cfg.pwm.oRaw[bank] ); break;
             case 0x051: err = StrToUInt32( seg[segIndex], cfg.pwm.oScaled[bank] ); break;
             case 0x052: err = StrToUInt32( seg[segIndex], cfg.pwm.oFactor[bank] ); break;
             case 0x053: err = StrToInt32( seg[segIndex], cfg.pwm.oOffset[bank] ); break;
                    
                        /// Begin DAC output parameters
             case 0x058: err = StrToUInt16(seg[segIndex], cfg.anlg.oRaw[bank]); break;
             case 0x059: err = StrToInt32(seg[segIndex], cfg.anlg.oScaleVal[bank]); break;
             case 0x05a: err = StrToInt32(seg[segIndex], cfg.anlg.oScaleParam[bank]); break;
             case 0x05b: err = StrToInt32(seg[segIndex], cfg.anlg.oOffset[bank]); break;
             case 0x05c: err = StrToUInt16(seg[segIndex], cfg.anlg.oFaultMode[bank]); break;
             case 0x05d: err = StrToInt32(seg[segIndex], cfg.anlg.oFaultValue[bank]); break;

                        /// Begin Digital I/O bank configuration parameters
             case 0x060: err = StrToUInt16(seg[segIndex], cfg.digi.bankConfigParam0[bank]); break;
             case 0x061: err = StrToUInt16(seg[segIndex], cfg.digi.bankConfigParam1[bank]); break;
             case 0x062: err = StrToUInt16(seg[segIndex], cfg.digi.bankConfigParam2[bank]); break;
             case 0x063: err = StrToUInt16(seg[segIndex], cfg.digi.bankConfigParam3[bank]); break;
             case 0x064: err = StrToUInt16(seg[segIndex], cfg.digi.bankConfigParam4[bank]); break;
             case 0x065: err = StrToUInt16(seg[segIndex], cfg.digi.bankConfigParam5[bank]); break;
             case 0x066: err = StrToUInt16(seg[segIndex], cfg.digi.bankConfigParam6[bank]); break;
             case 0x067: err = StrToUInt16(seg[segIndex], cfg.digi.bankConfigParam7[bank]); break;
             case 0x068: err = StrToUInt16(seg[segIndex], cfg.digi.bankConfigParam8[bank]); break;
             case 0x069: err = StrToUInt16(seg[segIndex], cfg.digi.bankConfigParam9[bank]); break;

             default:
                cml.Debug( "Unknown paramaeter in CCX file: 0x%02x\n", param );
                break;
          }
      }
   }

   fclose(fp);

   if( err ) return err;

   // The file was read in successfully.  Now, upload the configuration
   // structure to the I/O module.*/
   return SetIOConfig( cfg );
#endif
}
