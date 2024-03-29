/********************************************************/
/*                                                      */
/*  Copley Motion Libraries                             */
/*                                                      */
/*  Copyright (c) 2002 Copley Controls Corp.            */
/*                     http://www.copleycontrols.com    */
/*                                                      */
/********************************************************/

#include <stdio.h>

#ifdef _WIN32
#  include <windows.h>
#  include <winioctl.h>
#endif

#include "CML.h"
#include "can_copley.h"

CML_NAMESPACE_USE();

#ifdef _WIN32
   typedef int32  int32_t;
   typedef uint32 uint32_t;
   typedef uint8  uint8_t;
#else
#  include <stdint.h>
#endif

#include "copley_internal.h"

// local functions
static void *AllocLocals( void );
static void  FreeLocals( void *lcl );
static const Error *OpenFile( int port, void *lcl );
static const Error *CloseFile( void *lcl );
static const Error *SendCMD( void *lcl, uint32 *cmd );
static const Error *RecvMsg( void *lcl, CANCARD_MSG &can );
static const Error *SendMsg( void *lcl, CANCARD_MSG &can );

/**
Construct a CAN object. Calls CanInterface constructor.
*/
CopleyCAN::CopleyCAN( void ) : CanInterface()
{
   // Default baud to 1,000,000 bps
   baud = COPLEYCAN_BITRATE_1000000;
   open = 0;
   local = AllocLocals();
}

/**
Construct a new CAN object. Calls CanInterface constuctor.
*/
CopleyCAN::CopleyCAN( const char *port ) : CanInterface(port)
{
   // Default baud to 1,000,000 bps
   baud = COPLEYCAN_BITRATE_1000000;
   open = 0;
   local = AllocLocals();
}

/**
 Destructor for CopleyCAN object.
 */
CopleyCAN::~CopleyCAN( void )
{
   KillRef();
   Close();
   FreeLocals( local );
}

/**
Open the CAN bus
@return A CAN error object identifying the error.
*/
const Error *CopleyCAN::Open( void )
{
   const Error *err;

   if( open )
      return &CanError::AlreadyOpen;

   if( !local )
      return &CanError::Alloc;

   /**************************************************
    * Find the port number to open.
    **************************************************/
   int port = FindPortNumber( "CAN" );
   if( port < 0 ) port = FindPortNumber( "COPLEY" );
   if( port < 0 )
      return &CanError::BadPortName;

   // Create the device file name
   /**************************************************
    * Open the driver.
    **************************************************/
   cml.Debug( "Opening Copley CAN port: %d\n", port );
   err = OpenFile( port, local );
   if( err ) return err;

   uint32 cmd[64];
   cmd[0] = (CANCARD_CMD_SETBPS<<16) | 1;
   cmd[1] = baud;

   err = SendCMD( local, cmd );
   if( err )
   {
      cml.Debug( "Error setting CAN baud rate: %s\n", err->toString() );
      CloseFile( local );
      return err;
   }

   cmd[0] = (CANCARD_CMD_OPENPORT<<16);
   err = SendCMD( local, cmd );

   if( err )
   {
      cml.Debug( "Error opening CAN port: %s\n", err->toString() );
      CloseFile( local );
      return err;
   }

   // Set the interrupt inhibit time to zero.  We want interrupts 
   // as quickly as possible
   cmd[0] = (CANCARD_CMD_SETPARAM<<16) | 2;
   cmd[1] = COPLEYCAN_PARAM_INTINHIBIT;
   cmd[2] = 0;
   SendCMD( local, cmd );

   open = 1;
   return 0;
}

/**
Close the CAN interface.
@return A CAN error object identifying the error.
*/
const Error *CopleyCAN::Close( void )
{
   if( !open )
      return 0;

   uint32 cmd[64];
   cmd[0] = (CANCARD_CMD_CLOSEPORT<<16);
   SendCMD( local, cmd );

   open = 0;
   return CloseFile( local );
}

/**
Set the CAN interface baud rate.  This should be set 
before the CAN interface is open.
@param b The baud rate to set.
@return A CAN error object identifying the error.
*/
const Error *CopleyCAN::SetBaud( int32 b )
{
   if( open )
      return &CanError::AlreadyOpen;

   switch( b )
   {
      case 1000000: baud = COPLEYCAN_BITRATE_1000000; break;
      case  800000: baud = COPLEYCAN_BITRATE_800000;  break;
      case  500000: baud = COPLEYCAN_BITRATE_500000;  break;
      case  250000: baud = COPLEYCAN_BITRATE_250000;  break;
      case  125000: baud = COPLEYCAN_BITRATE_125000;  break;
      case  100000: baud = COPLEYCAN_BITRATE_100000;  break;
      case   50000: baud = COPLEYCAN_BITRATE_50000;   break;
      case   20000: baud = COPLEYCAN_BITRATE_20000;   break;

      default:
         return &CanError::BadParam;
   }

   return 0;
}

/**
Receive the next CAN frame.  
@param frame A reference to the frame object that will be filled by the read.
@param timeout The timeout (ms) to wait for the frame.  A timeout of 0 will
       return immediately if no data is available.  A timeout of < 0 will 
       wait forever.
@return A CAN error object identifying the error.
*/
const Error *CopleyCAN::RecvFrame( CanFrame &frame, Timeout timeout )
{
   CANCARD_MSG can;
   can.timeout = (int32)timeout;

   if( !open )
      return &CanError::NotOpen;

   const Error *err = RecvMsg( local, can );
   if( err ) return err;

   frame.id        = can.id;
   frame.length    = can.flags & COPLEYCAN_CANFLG_LENGTH;
   frame.timestamp = can.timestamp;

   switch( can.flags & (COPLEYCAN_CANFLG_ERROR|COPLEYCAN_CANFLG_RTR) )
   {
      case COPLEYCAN_CANFLG_RTR:   frame.type = CAN_FRAME_REMOTE; break;
      case COPLEYCAN_CANFLG_ERROR: frame.type = CAN_FRAME_ERROR;  break;
      default:                     frame.type = CAN_FRAME_DATA;   break;
   }

   if( frame.length > 8 ) frame.length = 8;
   if( can.flags & COPLEYCAN_CANFLG_EXTENDED )
      frame.id |= 0x20000000;

   for( int i=0; i<frame.length; i++ )
      frame.data[i] = can.data[i];

   return 0;
}

/**
Write a CAN frame to the CAN network.
@param frame A reference to the frame to write.
@param timeout The time to wait for the frame to be successfully sent.
       If the timeout is 0, the frame is written to the output queue and
       the function returns without waiting for it to be sent.
       If the timeout is <0 then the function will delay forever.
@return A CAN error object identifying the error.
*/
const Error *CopleyCAN::XmitFrame( CanFrame &frame, Timeout timeout )
{
   if( !open )
      return &CanError::NotOpen;

   if( frame.length > 8 )
      return &CanError::BadParam;

   CANCARD_MSG can;
   can.timeout = (int32)timeout;
   can.id = frame.id;
   can.flags = frame.length;

   switch( frame.type )
   {
      case CAN_FRAME_DATA:
         break;

      case CAN_FRAME_REMOTE:
         can.flags |= COPLEYCAN_CANFLG_RTR;
         break;

      default:
         return &CanError::BadParam;
   }

   if( frame.id & 0x20000000 )
      can.flags |= COPLEYCAN_CANFLG_EXTENDED;

   for( int i=0; i<frame.length; i++ )
      can.data[i] = frame.data[i];

   // Send the message
   return SendMsg( local, can );
}

/**
Convert error codes defined by the Vector CAN library into 
the standard error codes used by the motion library.
@param err The Vector style status code
@return A CAN error object.
*/
static const Error *ConvertError( int err )
{
   switch( err )
   {
      case COPLEYCAN_ERR_OK:                return 0;
      case COPLEYCAN_ERR_UNKNOWN_CMD:       return &CanError::Driver;
      case COPLEYCAN_ERR_BAD_PARAM:         return &CanError::BadParam;
      case COPLEYCAN_ERR_PORT_OPEN:         return &CanError::AlreadyOpen;
      case COPLEYCAN_ERR_PORT_CLOSED:       return &CanError::NotOpen;
      case COPLEYCAN_ERR_CARD_BUSY:         return &CanError::Driver;
      case COPLEYCAN_ERR_INTERNAL:          return &CanError::Driver;
      case COPLEYCAN_ERR_TIMEOUT:           return &CanError::Timeout; 
      case COPLEYCAN_ERR_SIGNAL:            return &CanError::Driver;
      case COPLEYCAN_ERR_MISSING_DATA:      return &CanError::Driver;
      case COPLEYCAN_ERR_CMDMUTEX_HELD:     return &CanError::Driver;
      case COPLEYCAN_ERR_QUEUECTRL:         return &CanError::Driver;
      case COPLEYCAN_ERR_FLASH:             return &CanError::Driver;
      case COPLEYCAN_ERR_NOTERASED:         return &CanError::Driver;
      case COPLEYCAN_ERR_FLASHFULL:         return &CanError::Driver;
      default:                              return &CanError::Unknown;
   }
}

/*********************************************************************************
 * Code below this section is operating system specific.
 ********************************************************************************/
#ifdef _WIN32

#include <windows.h>

// External helper function defined in
// Thread_w32.cpp
extern const Error *WaitOnWindowsObject( HANDLE hndl, int32 timeout );

typedef struct
{
   HANDLE hndl;
} DriverLocal;

// Allocate a structure to hold operating system specific local data
static void *AllocLocals( void )
{
   DriverLocal *locals = new DriverLocal;
   return (void*)locals;
}

static void FreeLocals( void *lcl )
{
   DriverLocal *local = (DriverLocal *)lcl;
   delete local;
}

#define MAX_PORTS_TOTAL        20
#define MAX_PORT_NAME          40
static int totPorts = -1;
static char portNames[ MAX_PORTS_TOTAL ][ MAX_PORT_NAME ];


// Figure out how many CAN ports the specified card has 
// by reading it's hardware type
static int CountPorts( const char *name, int card )
{
   char buff[MAX_PORT_NAME];
   const Error *err;
   uint32_t cmd[64];
   DriverLocal lcl;

   _snprintf( buff, sizeof(buff), "\\\\.\\%s%02d\\0", name, card );
   lcl.hndl = CreateFile( buff, GENERIC_READ|GENERIC_WRITE, FILE_SHARE_READ|FILE_SHARE_WRITE, 
                          NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL|FILE_FLAG_OVERLAPPED, NULL );
   if( lcl.hndl == INVALID_HANDLE_VALUE )
      return 0;

   // Read version info
   cmd[0] = (CANCARD_CMD_GETVERSION<<16);
   err = SendCMD( &lcl, cmd );
   CloseHandle( lcl.hndl );

   if( err )
   {
      cml.Debug( "Error reading CAN card version info: %s\n", err->toString() );
      return 0;
   }

   switch( cmd[1] )
   {
      case 0x00000000:
      case 0x00000001:
      case 0x00000002:
      case 0x00000100:
      case 0x00000200:
         return 2;

      default:
         return 1;
   }
}

static void ScanPorts( void )
{
   // Depending on the driver used, the device files could have different names.
   const char *cardNames[] = { "copleycan", "eican", 0 };
   int i, c, p;

   totPorts = 0;

   for( i=0; cardNames[i]; i++ )
   {
      for( c=0; totPorts<MAX_PORTS_TOTAL; c++ )
      {
         int ports = CountPorts( cardNames[i], c );
         if( ports < 1 ) break;

         for( p=0; (p<ports) && (totPorts<MAX_PORTS_TOTAL); p++, totPorts++ )
            _snprintf( portNames[totPorts], MAX_PORT_NAME, "\\\\.\\%s%02d\\%d", cardNames[i], c, p );
      }
   }
}

// Open the specified driver and initialize the local data structure 
// as necessary.
static const Error *OpenFile( int port, void *lcl )
{
   DriverLocal *local = (DriverLocal *)lcl;

   ScanPorts();
   if( port >= totPorts )
      return &CanError::BadPortName;

   // Open the device file for this port
   local->hndl = CreateFile( portNames[port], GENERIC_READ|GENERIC_WRITE, FILE_SHARE_READ|FILE_SHARE_WRITE, 
                              NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL|FILE_FLAG_OVERLAPPED, NULL );
   if( local->hndl == INVALID_HANDLE_VALUE )
      return &CanError::BadPortName;
   return 0;
}

static const Error *CloseFile( void *lcl )
{
   DriverLocal *local = (DriverLocal *)lcl;
   CloseHandle( local->hndl );
   return 0;
}

static const Error *DoIoCtl( DriverLocal *local, uint32 code, void *buff, uint32 inBytes, uint32 *outBytes, int timeout )
{
   OVERLAPPED overlap;
   DWORD tot;

   memset( &overlap, 0, sizeof(OVERLAPPED) );
   overlap.hEvent = CreateEvent( NULL, TRUE, FALSE, NULL );

   int err;
   BOOL ok = DeviceIoControl( local->hndl, code, buff, inBytes, buff, *outBytes, &tot, &overlap );

   if( ok )
   {
      cml.Warn( "IOCTL to driver unexpectedly successful, should have pended\n" );
      return &CanError::Driver;
   }

   err = GetLastError();
   if( err != ERROR_IO_PENDING )
   {
      cml.Warn( "Unexpected error received (%d) communicating with CAN driver\n", err );
      return &CanError::Driver;
   }

   const Error *eptr = WaitOnWindowsObject( overlap.hEvent, timeout );
   if( eptr )
   {
      CancelIo( local->hndl );
      CloseHandle( overlap.hEvent );
      return eptr;
   }

   ok = GetOverlappedResult( local->hndl, &overlap, &tot, FALSE );

   *outBytes = tot;

   CloseHandle( overlap.hEvent );
   return eptr;
}

/*
 * Write a command to the card.
 * @param lcl Local parameters
 * @param cmd Array of command data.  This array should be at least 64 words in length.
 * @return An error, or null on success.
 */
static const Error *SendCMD( void *lcl, uint32 *cmd )
{
   DriverLocal *local = (DriverLocal *)lcl;

   // Find the number of bytes of data sent with the message.
   int sendBytes = 4 * ((cmd[0] & 0x3F) + 1);

   uint32 tot = 256;
   const Error *err = DoIoCtl( local, COPLEYCAN_IOCTL_CMD, cmd, sendBytes, &tot, 5000 );
   if( err ) return err;

   return ConvertError( cmd[0]>>16 );
}

static const Error *RecvMsg( void *lcl, CANCARD_MSG &can )
{
   DriverLocal *local = (DriverLocal *)lcl;
   uint32 tot = sizeof(CANCARD_MSG);

   const Error *err = DoIoCtl( local, COPLEYCAN_IOCTL_RECVCAN, &can, sizeof(CANCARD_MSG), &tot, can.timeout );
   if( err ) return err;

   if( tot == 4 )
   {
      int err = *(int32 *)&can;
      return ConvertError( err );
   }
   return 0;
}

static const Error *SendMsg( void *lcl, CANCARD_MSG &can )
{
   DriverLocal *local = (DriverLocal *)lcl;
   uint32 tot = sizeof(CANCARD_MSG);

   const Error *err = DoIoCtl( local, COPLEYCAN_IOCTL_SENDCAN, &can, sizeof(CANCARD_MSG), &tot, can.timeout );
   if( err ) return err;

   if( tot == 4 )
   {
      int err = *(int32 *)&can;
      return ConvertError( err );
   }
   return 0;
}

#else

#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>

typedef struct
{
   int fd;
} DriverLocal;

// Allocate a structure to hold operating system specific local data
static void *AllocLocals( void )
{
   DriverLocal *locals = new DriverLocal;
   return (void*)locals;
}

static void FreeLocals( void *lcl )
{
   DriverLocal *local = (DriverLocal *)lcl;
   delete local;
}

// Open the specified driver and initialize the local data structure 
// as necessary.
static const Error *OpenFile( int port, void *lcl )
{
   DriverLocal *local = (DriverLocal *)lcl;

   // Convert the name into a device name
   char name[400];
   snprintf( name, sizeof(name), "/dev/copleycan%02d", port );
   cml.Debug( "Opening device file: %s\n", name );

   local->fd = ::open( name, O_RDWR );
   if( local->fd < 0 )
   {
      snprintf( name, sizeof(name), "/dev/eican%02d", port );
      cml.Debug( "Opening device file: %s\n", name );

      local->fd = ::open( name, O_RDWR );
   }

   if( local->fd < 0 )
   {
      cml.Error( "Unable to open device %s\n", name );
      return &CanError::BadPortName;
   }
   else
      return 0;
}

static const Error *CloseFile( void *lcl )
{
   DriverLocal *local = (DriverLocal *)lcl;
   ::close(local->fd);
   return 0;
}

/**
 * Write a command to the card
 */
static const Error *SendCMD( void *lcl, uint32 *cmd )
{
   DriverLocal *local = (DriverLocal *)lcl;
   int err = ioctl( local->fd, COPLEYCAN_IOCTL_CMD, cmd );
   if( !err ) err = cmd[0]>>16;
   return ConvertError( err );
}

static const Error *RecvMsg( void *lcl, CANCARD_MSG &can )
{
   Thread::CheckExit();
   DriverLocal *local = (DriverLocal *)lcl;
   int err = ioctl( local->fd, COPLEYCAN_IOCTL_RECVCAN, &can );
   return ConvertError( err );
}

static const Error *SendMsg( void *lcl, CANCARD_MSG &can )
{
   DriverLocal *local = (DriverLocal *)lcl;
   int err = ioctl( local->fd, COPLEYCAN_IOCTL_SENDCAN, &can );
   return ConvertError( err );
}

#endif

