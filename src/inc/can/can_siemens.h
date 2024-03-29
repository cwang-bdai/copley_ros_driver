/********************************************************/
/*                                                      */
/*  Copley Motion Libraries                             */
/*                                                      */
/*  Copyright (c) 2002 Copley Controls Corp.            */
/*                     http://www.copleycontrols.com    */
/*                                                      */
/********************************************************/

/** \file

CAN hardware interface for the SiemensPCIeToCAN CAN driver

*/



#ifndef _DEF_INC_CAN_SIEMENS
#define _DEF_INC_CAN_SIEMENS

#include "..\CML_Settings.h"
#include "..\CML_Can.h"

#include "..\CML_Threads.h"

CML_NAMESPACE_START()

/**
SiemensPCIeToCAN specific CAN interface.

This class extends the generic CanInterface class into a working
interface for the SiemensPCIeToCAN can device driver.

*/

class SiemensCAN : public CanInterface
{
public:
   SiemensCAN( void );
   SiemensCAN( const char *port );
   virtual ~SiemensCAN( void );

   const Error *Open( const char *name )
   {
      SetName(name);
      return Open();
   }
   const Error *Open( void );
   const Error *Close( void );
   const Error *SetBaud( int32 baud );

protected:
   const Error *RecvFrame( CanFrame &frame, Timeout timeout );
   const Error *XmitFrame( CanFrame &frame, Timeout timeout );

   /// tracks the state of the interface as open or closed.
   int open;

   /// Holds a copy of the last baud rate set
   int32 baud;

   /// Holds a value the SiemensPCIeToCan driver uses to identify bit rate
   int kvBaud;

   /// File handle used to configure and read from the CAN channel
   int Handle_Rd;

   /// File handle used to write to the CAN channel
   int Handle_Wr;

   const Error *ConvertError( int err );

   Mutex mutex;

private:
   /// Mutex used by reading threads to ensure clean exit
   Mutex readMutex;

   /// Counter used by reading threads
   int readCount;

   void IncReadCount( int ct );
   void WaitReadCount( void );
};

CML_NAMESPACE_END()

#endif



