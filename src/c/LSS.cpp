/********************************************************/
/*                                                      */
/*  Copley Motion Libraries                             */
/*                                                      */
/*  Copyright (c) 2002 Copley Controls Corp.            */
/*                     http://www.copleycontrols.com    */
/*                                                      */
/********************************************************/

#include "CML.h"

CML_NAMESPACE_USE();

/***************************************************************************/
/**
  Default constructor for the LSS object.
  @param co A reference to the CANopen network object over which this 
  protocol will run.
  */
/***************************************************************************/
LSS::LSS( CanOpen &co )
{
   recvCS = 0;
   to     = 15;
   coRef  = co.GrabRef();
   co.EnableReceiver( 2020, this );
   vendorID = 0xAB;
}

LSS::~LSS()
{
   ReleaseRef( coRef );
}

/***************************************************************************/
/**
  Set all devices on the network into either LSS configuration mode or LSS
  operational mode.
  @param config  If true, put all devices into configuration mode.  Otherwise,
  put everything into operational mode.
  @return A pointer to an error object, or NULL on success.
 */
/***************************************************************************/
const Error *LSS::SwitchModeGlobal( bool config )
{
   if( config )
      return Xmit( 4, 1 );
   else
      return Xmit( 4, 0 );
}

/***************************************************************************/
/**
  Search the CANopen network for Copley amplifiers.  This function uses the
  CANopen Layer Setting Services (LSS) protocol to find amplifiers
  on the network.  All Copley amplifiers on the CANopen network can be
  identified using this protocol, even if they do not have a valid CANopen 
  node ID number configured.

  On return from this function, the passed array will have been filled with
  the serial numbers of all the amplifiers found.  These serial numbers may
  then be passed to LSS::SetAmpNodeID to assign a node ID number to the 
  amplifier.

  Note that firmware support for the LSS protocol was added starting with 
  version 4.04.  Any amplifier on the network with earlier firmware will not
  be discovered using this technique.

  @param max     The maximum number of amplifier serial numbers to be returned.
  @param serial  An array where the amplifier serial numbers will be returned.
  This array must be at least max elements long. 

  @return The number of amplifiers actually found.  This is not limited to the
  max parameter value
  */
/***************************************************************************/
int LSS::FindAmplifiers( int max, uint32 serial[] )
{
   const Error *err;

   // Switch all devices on the network to LSS configure mode
   err = SwitchModeGlobal( true );
   uint32 ampCt = 0;
   
   // Three vendor IDs are supported. All need to be checked before deciding that
   // no amplifiers are on the network.
   uint32 supportedVendors[3];
   supportedVendors[0] = 0xAB;
   supportedVendors[1] = 0x6A;
   supportedVendors[2] = 0x247;

   for (int index = 0; index < 3; index++)
   {
	   // Pass the vendor and product codes to search
	   if (!err) err = Xmit(70, supportedVendors[index]);
	   if (!err) err = Xmit(71, 0);

	   // Search for any product revision number
	   if (!err) err = Xmit(72, 0);
	   if (!err) err = Xmit(73, 0xFFFFFFFF);

	   if (err) return 0;

	   tot = 0;
	   this->max = max;
	   this->serial = serial;

	   ampCt = FindAmpSerial(0, 0xFFFFFFFF);
	   if (ampCt > 0)
	   {
		   vendorID = supportedVendors[index];
		   break;
	   }
   }

   return ampCt;
}

/***************************************************************************/
/**
  Save the current node ID and bit rate info to non-volatile memory on 
  the selected drive.  When this is called, exactly one drive should be
  in LSS configuration mode.
  @return A pointer to an error object, or NULL on success.
  */
/***************************************************************************/
const Error *LSS::StoreConfig( void )
{
   return Enquire( 23 );
}

/***************************************************************************/
/**
  Get the current CANopen node ID of the specified amplifier.
  @param serial The serial number of the amplifier to query.
  @param nodeID The node ID will be returned here.
  @return A pointer to an error object, or NULL on success.
  */
/***************************************************************************/
const Error *LSS::GetAmpNodeID( uint32 serial, byte &nodeID )
{
   // Select just the specified amplifier
   const Error *err = SelectAmp( serial );
   if( err ) return err;

   return EnquireInfo( 0, 0, 0, 0, &nodeID );
}

/***************************************************************************/
/**
  This internal function is used to enquire one field from the single 
  selected drive using LSS.
  @param cs  The LSS code of the desired field
  @return A pointer to an error object, or NULL on success.
  */
/***************************************************************************/
const Error *LSS::Enquire( int cs )
{
   // Get the node ID
   while( !sem.Get(0) );
   recvCS = cs;
   const Error *err = Xmit( cs );
   if( !err ) err = sem.Get( to );
   recvCS = 0;
   return err;
}

/***************************************************************************/
/**
  Read identification info from the selected device using LSS.
  Only one device should be selected when this is called.
  @param vid     If not NULL, the vendor ID will be returned here
  @param prodID  If not NULL, the product ID will be returned here
  @param revNum  If not NULL, the revision number will be returned here
  @param serial  If not NULL, the serial number will be returned here
  @param nodeID  If not NULL, the node ID will be returned here
  @return A pointer to an error object, or NULL on success.
  */
/***************************************************************************/
const Error *LSS::EnquireInfo( uint32 *vid, uint32 *prodID, uint32 *revNum, uint32 *serial, byte *nodeID )
{
   if( vid )
   {
      const Error *err = Enquire( 90 );
      if( err ) return err;
      *vid = recvData;
   }

   if( prodID )
   {
      const Error *err = Enquire( 91 );
      if( err ) return err;
      *prodID = recvData;
   }

   if( revNum )
   {
      const Error *err = Enquire( 92 );
      if( err ) return err;
      *revNum = recvData;
   }

   if( serial )
   {
      const Error *err = Enquire( 93 );
      if( err ) return err;
      *serial = recvData;
   }

   if( nodeID )
   {
      const Error *err = Enquire( 94 );
      if( err ) return err;
      *nodeID = (byte)recvData;
   }

   return 0;
}

/***************************************************************************/
/**
  Set the CANopen node ID of the specified amplifier.
  @param serial The serial number of the amplifier to update.
  @param nodeID The CANopen node ID to assign to this amplifier.
  @return A pointer to an error object, or NULL on success.
  */
/***************************************************************************/
#include <stdio.h>
const Error *LSS::SetAmpNodeID( uint32 serial, byte nodeID )
{
   if( (nodeID < 1 || nodeID > 127) && (nodeID != 0xFF) )
      return &CanOpenError::BadNodeID;

   // Select just the specified amplifier
   const Error *err = SelectAmp( serial );
   if( err ) return err;

   err = SetNodeID( nodeID );
   if( err ) return err;

   // Deselect the amplifier which causes it to start using
   // the new node ID.
   return SwitchModeGlobal( false );
}

/***************************************************************************/
/**
  Set the CANopen node ID of the currently selected device.
  When this is called, exactly one deivce on the network should 
  be in LSS configuration mode.

  @param nodeID The CANopen node ID to assign to the selected device.
  @return A pointer to an error object, or NULL on success.
  */
/***************************************************************************/
const Error *LSS::SetNodeID( byte nodeID )
{
   if( (nodeID < 1 || nodeID > 127) && (nodeID != 0xFF) )
      return &CanOpenError::BadNodeID;

   // Set the node ID
   while( !sem.Get(0) );
   recvCS = 17;
   const Error *err = Xmit( 17, nodeID );
   if( !err ) err = sem.Get( to );
   recvCS = 0;
   if( err ) return err;

   if( recvData & 0xFF )
      return &CanOpenError::BadNodeID;

   return 0;
}

/***************************************************************************/
/**
   Send an LSS command to program the selected devices CAN bit rate given the
   table number and index within the table.  When this is called, only one device
   should be in LSS config mode.

   Note that the new bit rate doesn't become active immedately, it must be 
   activated first by calling LSS::ActivateBitRate.

   @param tableID  Specifies which table of bit rates to use (0 for standard table)
   @param index    Specifies the index within the table of the desired bit rate.
   @return A pointer to an error object, or NULL on success.
 */
/***************************************************************************/
const Error *LSS::SetBitRate( byte tableID, byte index )
{
   uint32 dat = (((uint32)index)<<8) | tableID;

   // Set the node ID
   while( !sem.Get(0) );
   recvCS = 19;
   const Error *err = Xmit( 19, dat );
   if( !err ) err = sem.Get( to );
   recvCS = 0;
   if( err ) return err;

   if( recvData & 0xFF )
      return &CanError::BadParam;

   // Deselect the amplifier
   return 0;
}

/***************************************************************************/
/**
   Send an LSS command to program the selected devices CAN bit rate given the
   table number and index within the table.  When this is called, only one device
   should be in LSS config mode.

   Note that the new bit rate doesn't become active immedately, it must be 
   activated first by calling LSS::ActivateBitRate.

   @param rate     The new bit rate in bits/sec
   @return A pointer to an error object, or NULL on success.
 */
/***************************************************************************/
const Error *LSS::SetBitRate( uint32 rate )
{
   // Determine which index in the standard bit rate table to use.
   byte index;
   switch( rate )
   {
      case 1000000: index = 0; break;
      case  800000: index = 1; break;
      case  500000: index = 2; break;
      case  250000: index = 3; break;
      case  125000: index = 4; break;
      case   50000: index = 6; break;
      case   20000: index = 7; break;
      case   10000: index = 8; break;
      default: return &CanError::BadParam;
   }
   return SetBitRate( 0, index );
}

/***************************************************************************/
/**
  Activate the new bit rate previously set on all devices using the 
  LSS::SetBitRate function.

  Note that this function doesn't change the bit rate of the local CAN port,
  it simply returns after requesting the new rate on the LSS slave devices.
  It's the responsibility of the calling function to change the local bit 
  rate.

  After calling this function the master should stop transmitting on the CAN
  bus.  After a delay of the specified time, the master should change it's bit
  rate.  After a second delay of the specified time, the master can start 
  transmitting on the bus at the new bit rate.

  @param delay  Delay in milliseconds which the LSS devices will use to ensure
                that they all switch their bit rates at a time when no device is
                transmitting.  This delay will be used twice, once before the 
                change and once after.

   @return A pointer to an error object, or NULL on success.
 */
/***************************************************************************/
const Error *LSS::ActivateBitRate( uint16 delay )
{
   // Put all devices on the network into configuration mode
   const Error *err = SwitchModeGlobal( true );
   if( err ) return err;

   // Send the command to activate bit rate
   while( !sem.Get(0) );
   return Xmit( 21, delay );
}

/***************************************************************************/
/**
  Put the specified amplifier into LSS configure mode.  All other amplifiers
  on the network are switched into LSS operational mode.
  @param serial The serial number of the device to configure
  @return A pointer to an error object, or NULL on success.
  */
/***************************************************************************/
const Error *LSS::SelectAmp( uint32 serial )
{
   const Error *err;

   // Switch all devices on the network out of LSS config mode
   err = SwitchModeGlobal( false );

   // Put the amplifier with the specified serial number into config mode.
   if( !err ) err = Xmit( 64, vendorID );
   if( !err ) err = Xmit( 65, 0 );
   if( !err ) err = Xmit( 66, 0 );
   if( err ) return err;

   // Clear out the semaphore and transfer the serial number message.  This
   // should cause the selected amplifier to respond
   while( !sem.Get(0) );
   recvCS = 68;

   err = Xmit( 67, serial );
   if( !err ) err = sem.Get( to );

   recvCS = 0;

   return err;
}

/***************************************************************************/
/**
  This method is called by the main CAN network listener when a new LSS 
  response frame is received.
  @param frame A reference to the CAN fame that was received
  @return Non-zero if the frame was handled.
  */
/***************************************************************************/
int LSS::NewFrame( CanFrame &frame )
{
   // Ignore any messages when not expected
   if( !recvCS )
      return 1;

   // When an expected message is received, capture
   // the data and post a semaphore.
   else if( frame.data[0] == recvCS )
   {
      recvData = bytes_to_uint32( &frame.data[1] );
      sem.Put();
   }

   return 1;
}

/***************************************************************************/
/**
  Find the serial number of the first amplifier in the passed range.
  @param low The lower limit of the range
  @param high The upper limit of the range
  @return the serial number, or zero if no amp found.
  */
/***************************************************************************/
uint32 LSS::FindAmpSerial( uint32 low, uint32 high )
{
   // Clear out my semaphore
   while( !sem.Get(0) );

   // Send the serial number range
   recvCS = 79;
   if( Xmit( 74, low ) ) return 0;
   if( Xmit( 75, high ) ) return 0;

   // Wait for any amplifiers in this range to respond.
   const Error *err;
   int ct = 0;

   while( 1 )
   {
      err = sem.Get( to );
      if( err )
         break;
      ct++;
   }

   recvCS = 0;
   if( !ct ) return 0;

   if( low == high )
   {
      if( tot < max )
         serial[tot++] = low;
      return 1;
   }

   uint32 mid = low + (high-low)/2;

   return FindAmpSerial( low, mid ) + FindAmpSerial( mid+1, high );
}

/***************************************************************************/
/**
  Transmit a LSS CAN frame 
  @param cs The command specifier for this frame.
  @param data The data passed with the frame.
  @return A pointer to an error object, or NULL on success.
  */
/***************************************************************************/
const Error *LSS::Xmit( byte cs, uint32 data )
{
   CanFrame frame;
   frame.id = 2021;
   frame.type = CAN_FRAME_DATA;
   frame.length = 8;
   frame.data[0] = cs;
   frame.data[1] = ByteCast(data);
   frame.data[2] = ByteCast(data>>8);
   frame.data[3] = ByteCast(data>>16);
   frame.data[4] = ByteCast(data>>24);
   frame.data[5] = 0;
   frame.data[6] = 0;
   frame.data[7] = 0;

   RefObjLocker<CanOpen> co( coRef );
   if( !co ) return &NodeError::NetworkUnavailable;

   return co->Xmit( frame );
}

