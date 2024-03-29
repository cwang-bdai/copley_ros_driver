/********************************************************/
/*                                                      */
/*  Copley Motion Libraries                             */
/*                                                      */
/*  Copyright (c) 2002 Copley Controls Corp.            */
/*                     http://www.copleycontrols.com    */
/*                                                      */
/********************************************************/

/** \file
  Implementation of the Linkage class.
  */

#include "CML.h"

// Note, This disables an annoying VC++ warning
#ifdef _WIN32
#pragma warning( disable: 4355 )
#endif

CML_NAMESPACE_USE();

// Linkage errors
CML_NEW_ERROR( LinkError, BadAmpCount,      "An illegal number of amplifiers was passed to Linkage::Init" );
CML_NEW_ERROR( LinkError, NetworkMismatch,  "The amplifiers passed to Linkage::Init don't share a network" );
CML_NEW_ERROR( LinkError, AlreadyInit,      "The linkage is already initialized" );
CML_NEW_ERROR( LinkError, AmpAlreadyLinked, "The passed amplifier object is already assigned to a linkage" );
CML_NEW_ERROR( LinkError, AxisCount,        "The point dimension doesn't match the number of linkage axes" );
CML_NEW_ERROR( LinkError, AmpTrjOverflow,   "Amplifier trajectory structure overflow." );
CML_NEW_ERROR( LinkError, AmpTrjInUse,      "Amplifier trajectory already in use" );
CML_NEW_ERROR( LinkError, AmpTrjNotRunning, "Amplifier trajectory not presently in use" );
CML_NEW_ERROR( LinkError, NoActiveTrj,      "No linkage trajectory is active" );
CML_NEW_ERROR( LinkError, BadMoveLimit,     "A zero or negative move limit was detected" );
CML_NEW_ERROR( LinkError, UnknownAmpErr,    "An amplifier error occurred" );
CML_NEW_ERROR( LinkError, StartMoveTO,      "Timeout waiting on amplifier to respond to start move command" );
CML_NEW_ERROR( LinkError, NotSupported,     "Support for this function was not enabled in the library" );
CML_NEW_ERROR( LinkError, AmpRemoved,       "An amp object referenced by the linkage is no longer valid" );

/***************************************************************************/
/**
  Default constructor.  Linkage::Init must be called before this linkage 
  object may be used.
  */
/***************************************************************************/
Linkage::Linkage( void ): ctrlPDO( *this )
{
   linkID = 0;
   netRef = 0;
   trjUseCount = 0;
   ampct = 0;
   maxVel = maxAcc = maxDec = maxJrk = 0;
   linkTrjRef = 0;

   for( int i=0; i<CML_MAX_AMPS_PER_LINK; i++ )
   {
      ampRef[i] = 0;
#ifdef CML_LINKAGE_TRJ_BUFFER_SIZE
      ampTrj[i].Init( this );
#endif
   }

   ClearLatchedError();
}

/***************************************************************************/
/**
  Linkage object destructor.
  */
/***************************************************************************/
Linkage::~Linkage()
{
   KillRef();
   for( int i=0; i<ampct; i++ )
   {
      if( ampRef[i] )
      {
         RefObjLocker<Amp> amp( ampRef[i] );
         if( amp )
            amp->SetLinkage(0);
         RefObj::ReleaseRef( ampRef[i] );
      }
   }
}

/***************************************************************************/
/**
  Mark the specified Amp as invalid.  This is called by an Amp object during
  destruction if the amp is still attached to a linkage (which is an error).
  @param a Pointer to the amp
  */
/***************************************************************************/
void Linkage::InvalidateAmp( uint32 a )
{
   for( int i=0; i<ampct; i++ )
   {
      if( ampRef[i] == a )
      {
         RefObj::ReleaseRef( a );
         ampRef[i] = 0;
      }
   }
   return;
}

/***************************************************************************/
/**
  Configure a linkage.  The linkage object will be configured to use the 
  various settings passed in the LinkSettings object.

  When a new Linkage object is created, it will be configured using a 
  default set of settings.  These settings can be modified through this
  call.  Set the documentation of the LinkSettings object for details of 
  the available settings and their default values.

  @param settings The new settings to be used.  A local copy of this object
  will be made by the linkage.

  @return An error object pointer, or NULL on success.
  */
/***************************************************************************/
const Error *Linkage::Configure( LinkSettings &settings )
{
   cfg = settings;
   return 0;
}

/***************************************************************************/
/**
  Initialize a new linkage object.  If the object has already been initialized,
  this will fail with an error.

  All amplifiers attached to a linkage must be initialized, and must share the
  same network object.  Also, amplifiers may only be attached to one
  linkage at a time, so this function will fail if any of the passed amplifier
  objects is already attached to a Linkage.

  The linkage object will maintain pointers to each of the amplifier objects
  passed to this function.  The amplifiers may not be destroyed until after 
  the linkage object is.  

  @param ct The number of amplifiers to be used with this linkage.
  Note that this must be between 1 and CML_MAX_AMPS_PER_LINK.

  @param a An array of amplifiers to be assigned to this linkage.  There
  must be at least ct amplifiers in this array.

  @return An error object pointer, or NULL on success.
  */
/***************************************************************************/
const Error *Linkage::Init( uint16 ct, Amp a[] )
{
   if( ct < 1 || ct > CML_MAX_AMPS_PER_LINK )
      return &LinkError::BadAmpCount;

   Amp *aptr[ CML_MAX_AMPS_PER_LINK ];

   for( int i=0; i<ct; i++ )
      aptr[i] = &a[i];

   return Init( ct, aptr );
}

/***************************************************************************/
/**
  Initialize a new linkage object.  If the object has already been initialized,
  this will fail with an error.

  All amplifiers attached to a linkage must be initialized, and must share the
  same network object.  Also, amplifiers may only be attached to one
  linkage at a time, so this function will fail if any of the passed amplifier
  objects is already attached to a Linkage.

  The linkage object will maintain pointers to each of the amplifier objects
  passed to this function.  The amplifiers may not be destroyed until after 
  the linkage object is.  

  @param ct The number of amplifiers to be used with this linkage.
  Note that this must be between 1 and CML_MAX_AMPS_PER_LINK.

  @param a An array of pointer to amplifier objects to be assigned to this 
  linkage.  There must be at least ct pointers in this array.

  @return An error object pointer, or NULL on success.
  */
/***************************************************************************/
const Error *Linkage::Init( uint16 ct, Amp *a[] )
{
   cml.Debug( "Initializing linkage %d\n", a[0]->GetNodeID() );

   CML_ASSERT( ct > 0 );
   CML_ASSERT( ct <= CML_MAX_AMPS_PER_LINK );
   CML_ASSERT( CML_MAX_AMPS_PER_LINK <= 32 );
   CML_ASSERT( ampRef[0] == 0 );

   if( ct < 1 || ct > CML_MAX_AMPS_PER_LINK )
      return &LinkError::BadAmpCount;

   if( ampRef[0] )
      return &LinkError::AlreadyInit;

   ClearLatchedError();

   // Make sure all amps are initialized, and share
   // the same network.
   netRef = a[0]->GetNetworkRef();
   linkID = a[0]->GetNodeID();
   netType = a[0]->GetNetworkType();

   int i;
   for( i=0; i<ct; i++ )
   {
      const Error *err = 0;

      if (!a[i]->IsInitialized()) {
          err = &CanOpenError::NotInitialized;
      }
      else if (netRef != a[i]->GetNetworkRef()) {
          err = &LinkError::NetworkMismatch;
      }
      else if (a[i]->linkRef != 0) {
          err = &LinkError::AmpAlreadyLinked;
      }
      if (err) {
          return LatchError(err, i);
      }
   }

   // Assign all the amplifiers to this linkage
   ampct = ct;
   for( i=0; i<ct; i++ )
   {
      ampRef[i] = a[i]->GrabRef();
      a[i]->SetLinkage(this);
   }

   // Add my state events to each amplifier
   for( i=0; i<ct; i++ )
   {
      stateEvent[ i ].link = this;
      a[i]->eventMap.Add( &stateEvent[i] );
   }

   UpdateLoadToUserUnitConverters();

   // check for position wrap on each amplifier (updates private 
   // data member variables in amp class)
   UpdatePositionWrap();

   // Start my thread
   start();

   // If these amps are on a CANopen network, then initialize 
   // a PDO used to send control words to each amplifier
   // We use a slightly different technique on EtherCAT networks
   if( netType == NET_TYPE_CANOPEN )
      return ctrlPDO.Init();
   else
      return 0;
}

/***************************************************************************/
/**
  Get a reference to the amplifier object at the specified location in the
  linkage.  Note that if CML_DEBUG_ASSERT is defined, then the standard C 
  assert function will be used to check for an invalid index.

  NOTE: This function is unsafe and has been depreciated.  Use Linkage::GetAmpRef()
  as a safer alternative.

  @param i The index of the amplifier to access.
  @return A reference to the amplifier object.
  */
/***************************************************************************/
Amp &Linkage::GetAmp( uint16 i )
{
   CML_ASSERT( i < ampct );  
   CML_ASSERT( ampRef[i] != 0 );

   Amp *ampPtr = (Amp *)RefObj::LockRef( ampRef[i] );
   CML_ASSERT( ampPtr );

   ampPtr->UnlockRef();
   return *ampPtr;
}

/***************************************************************************/
/**
  Return a CML reference number for the amp object at the specified location 
  in the linkage.  RefObj::LockRef() can then be used to safely obtain a pointer
  to the actual Amp object.

  @param i The index of the amplifier to access.
  @return A reference to the amplifier object.
  */
/***************************************************************************/
uint32 Linkage::GetAmpRef( uint16 i )
{
   CML_ASSERT( i < ampct );  
   return ampRef[i];
}

/***************************************************************************/
/**
  Get the current commanded position of the linkage.  Note that this function
  queries the position of each amplifier sequentially and therefore the returned
  position information will only be accurate if the linkage is at rest when the
  function is called.

  @param p A point that will be filled in with the current Linkage commanded 
  position.
  @return An error object pointer, or NULL on success.
  */
/***************************************************************************/
const Error *Linkage::GetPositionCommand( PointN &p )
{
   int axes = GetAxesCount();
   CML_ASSERT( axes <= CML_MAX_AMPS_PER_LINK );

   if( p.getDim() != axes )
      return &LinkError::AxisCount;

   uunit pos[CML_MAX_AMPS_PER_LINK];

   int i;
   const Error *err;

   for( i=0; i<ampct; i++ )
   {
      RefObjLocker<Amp> amp( ampRef[i] );
      if( !amp )
         err = &LinkError::AmpRemoved;
      else
         err = amp->GetPositionCommand( pos[i] );
      if( err )
      {
         LatchError( err, i );
         return err;
      }
   }

   err = ConvertAmpToAxisPos( pos );
   if( err )
   {
      LatchError( err, -1 );
      return err;
   }

   for( i=0; i<axes; i++ )
      p[i] = pos[i];

   return 0;
}

/***************************************************************************/
/**
  Set limits used for multi-axis point-to-point moves.

  @param vel Maximum velocity
  @param acc Maximum acceleration
  @param dec Maximum deceleration
  @param jrk Maximum jerk
  @return An error object pointer, or NULL on success.
  */
/***************************************************************************/
const Error *Linkage::SetMoveLimits( uunit vel, uunit acc, uunit dec, uunit jrk )
{
   if( vel <= 0 || acc <= 0 || dec <= 0 || jrk <= 0 )
      return 0;

   maxVel = vel;
   maxAcc = acc;
   maxDec = dec;
   maxJrk = jrk;

   return 0;
}

/***************************************************************************/
/**
  Return the move limits currently set for this linkage.
  @param vel Returns maximum velocity
  @param acc Returns maximum acceleration
  @param dec Returns maximum deceleration
  @param jrk Returns maximum jerk
  @return An error object pointer, or NULL on success.
  */
/***************************************************************************/
const Error *Linkage::GetMoveLimits( uunit &vel, uunit &acc, uunit &dec, uunit &jrk )
{
   vel = maxVel;
   acc = maxAcc;
   dec = maxDec;
   jrk = maxJrk;
   return 0;
}

/***************************************************************************/
/**
  Move to a specified position.  This move uses the limits previously set using
  Linkage::SetMoveLimits.

  @param p The point to move to.
  @param start If true (the default), the profile will be started by this call.
               If false, the profile will be uploaded, but not started.  In that case
               the move may be later started by a call to Linkage::StartMove.
  @return An error object pointer, or NULL on success.
  */
/***************************************************************************/
const Error *Linkage::MoveTo( PointN &p, bool start )
{
    linkTrjScurveObj.SetIsRelative(false);
   return MoveTo( p, maxVel, maxAcc, maxDec, maxJrk, start );
}

/***************************************************************************/
/**
  Move to a point in space.  The number of dimensions of the point must equal
  the number of axes controlled by the Linkage (as returned by Linkage::GetAxesCount).

  This method causes the linkage to perform a straight line move in N space 
  from the present position to the specified point.  The move will be limited
  in velocity, acceleration & jerk to the passed values.

  The linkage is assumed to be at rest when this method is called.  If this isn't 
  the case, then an error will result.

  Note that this function causes a trajectory to be calculated and passed to the 
  amplifiers as a series of PVT points.  This calculation requires floating point
  math, so this function is not available if floating point support has not been 
  enabled in CML_Settings.h.

  @param p The point in N space to move to.
  @param vel Maximum velocity
  @param acc Maximum acceleration
  @param dec Maximum deceleration
  @param jrk Maximum jerk
  @param start If true (the default), the profile will be started by this call.
               If false, the profile will be uploaded, but not started.  In that case
               the move may be later started by a call to Linkage::StartMove.

  @return An error object pointer, or NULL on success.
  */
/***************************************************************************/
const Error* Linkage::MoveTo(PointN& p, uunit vel, uunit acc, uunit dec, uunit jrk, bool start)
{
#ifndef CML_ALLOW_FLOATING_POINT
    return &LinkError::NotSupported;
#else
    ClearLatchedError();

   if (p.getDim() != GetAxesCount()) {
      return &LinkError::AxisCount;
   }

   const Error *err;

   Point<CML_MAX_AMPS_PER_LINK> startPos;
   startPos.setDim( GetAxesCount() );

   err = GetPositionCommand( startPos );
   if( err ) return err;

   err = linkTrjScurveObj.Calculate( startPos, p, vel, acc, dec, jrk );

   // If there was an error when calculating the linkage, it is 
   // most likely due to a trajectory reuse attempt when the 
   // linkage was currently in use. Reset the linkage, and try 
   // to calculate again.
   while( err == &ScurveError::InUse ) {
        err = linkTrjScurveObj.Calculate(startPos, p, vel, acc, dec, jrk);
   }

   if (err) { return err; }

   // reset the private boolean "isRelative" variable back to false (default value)
   linkTrjScurveObj.SetIsRelative(false);

   const Error *sendTrajErr = SendTrajectory(linkTrjScurveObj, start);

   return sendTrajErr;
#endif
}

/***************************************************************************/
/**
  Move to a point in space using relative distances. This move uses the limits previously set using
  Linkage::SetMoveLimits.

  @param p     The relative distances to the desired point in N space.
  @param start If true (the default), the profile will be started by this call.
               If false, the profile will be uploaded, but not started.  In that case
               the move may be later started by a call to Linkage::StartMove.
  @return An error object pointer, or NULL on success.
  */
  /***************************************************************************/
const Error* Linkage::MoveToRel(PointN& p, bool start)
{
    linkTrjScurveObj.SetIsRelative(true);
    return MoveTo(p, maxVel, maxAcc, maxDec, maxJrk, start);
}

#ifdef CML_LINKAGE_TRJ_BUFFER_SIZE
/***************************************************************************/
/**
  Upload a multi-axis PVT move trajectory to the linkage and optionally start the move.

  @param trj Reference to the linkage trajectory to be used.  A local
  reference to this trajectory will be stored if the entire profile will 
  not fit in the amplifiers on-board buffer.  This pointer will be kept
  until the entire profile has been uploaded to the linkage.  It is therefore
  important to ensure that the trajectory object passed here will remain
  valid (i.e. not be deallocated) until the linkage has called the 
  LinkTrajectory.Finish() method.

  @param start If true (the default), the profile will be started by this call.
               If false, the profile will be uploaded, but not started.  In that case
               the move may be later started by a call to Linkage::StartMove.

  @return An error object.
  */
/***************************************************************************/
const Error *Linkage::SendTrajectory( LinkTrajectory &trj, bool start )
{
   ClearLatchedError();

   // Clear bit 4 of the control word.  IP mode doesn't have a good handshake with the
   // drive like PP mode does, so clearing bit 4 here ensures that the drive will see
   // the rising edge when we start the move
   SetControlWord( 0x000F );

   // Save a reference to the passed trajectory and increase it's usage 
   // counter.  We increase the usage counter here so that the trajectory
   // won't be freed if the first amp send's it all down.
   linkTrjRef = trj.GrabRef();
   IncTrjUseCount();

   // Reset the amp trajectories to make sure there are no stale 
   // points in their buffers.
   for (int i = 0; i < ampct; i++) {
      ampTrj[i].Reset();
   }

   for( int i=0; i<ampct; i++ ) {
      const Error *err;
      RefObjLocker<Amp> amp( ampRef[i] );
      if (!amp) {
         err = &LinkError::AmpRemoved;
      }
      else {
         err = amp->SendTrajectory( ampTrj[i], false );
      }
      if( err ) {
         DecTrjUseCount();
         return LatchError( err, i );
      }
   }

   // Lower the usage count here to make up for the initial increase.
   // This may cause the trajectory to be freed if it's been completely 
   // downloaded to all axes.
   DecTrjUseCount();

   if (start) {
      
       const Error * startMoveErr = StartMove();
       return startMoveErr;
   }
   else {
       return 0;
   }
}
#endif

/***************************************************************************/
/**
  Start the moves that have already been programmed into all
  axes of this linkage.

  @return An error object pointer, or NULL on success.
  */
/***************************************************************************/
const Error *Linkage::StartMove( void )
{
   ClearLatchedError();

   uint32 allAmps = (1<<ampct) - 1;
   const Error *err = 0;
   EventAny events[CML_MAX_AMPS_PER_LINK];
   EventMap map;
   int i;

   // Create an event to monitor the status of the
   // trajectory abort bits
   EventNone trjAbortEvt( LINKEVENT_ABORT );

   EventNone none( allAmps );
   EventAll all( allAmps );

   // Make sure all amps are in profile mode
   // and are ready to start a new move.
   for( i=0; i<ampct; i++ )
   {
      RefObjLocker<Amp> amp( ampRef[i] );
      if( !amp )
         err = &LinkError::AmpRemoved;
      else
         err = amp->CheckStateForMove();
      if( err )
      {
         err = LatchError( err, i );
         goto cleanup;
      }

      events[i].setChain( map, (1<<i) );
      events[i].setValue( AMPEVENT_SPACK | AMPEVENT_PVT_EMPTY );
      amp->eventMap.Add( &events[i] );
   }

   // Enable the amps, and wait for them to clear
   // the acknowledge bit.
   SetControlWord( 0x000F );

   cml.Debug( "Linkage Waiting for all amps to clear ACK bit, current mask: 0x%08x\n", map.getMask() );
   err = none.Wait( map, cfg.moveAckTimeout );

   // On timeout, find the offending thread
   if( err == &ThreadError::Timeout )
   {
      cml.Debug( "Linkage timeout waiting for clear ACK bits, current mask: 0x%08x\n", map.getMask() );
      for( i=0; i<ampct; i++ )
      {
         RefObjLocker<Amp> amp( ampRef[i] );
         if( !amp )
         {
            err = LatchError( &LinkError::AmpRemoved, i );
            break;
         }

         AMP_EVENT ampEventObj;
         amp->GetEventMask( ampEventObj );
         if( ampEventObj & AMPEVENT_SPACK )
         {
            err = LatchError( &LinkError::StartMoveTO, i );
            break;
         }
      }
   }

   if( err )
   {
      err = LatchError( err, -1 );
      goto cleanup;
   }

   // Now, start a move on all amps, and wait for
   // the acknowledge bit to be set.
   SetControlWord( 0x003F );

   cml.Debug( "Linkage Waiting for all amps to set ACK bit, current mask: 0x%08x\n", map.getMask() );
   err = all.Wait( map, cfg.moveAckTimeout );

   // On timeout, find the offending thread
   if( err == &ThreadError::Timeout )
   {
      cml.Debug( "Linkage timeout waiting for set ACK bits, current mask: 0x%08x\n", map.getMask() );
      for( i=0; i<ampct; i++ )
      {
         RefObjLocker<Amp> amp( ampRef[i] );
         if( !amp )
         {
            err = LatchError( &LinkError::AmpRemoved, i );
            break;
         }

         AMP_EVENT e;
         amp->GetEventMask( e );
         if( !(e & AMPEVENT_SPACK) )
         {
            err = LatchError( &LinkError::StartMoveTO, i );
            break;
         }
      }
   }

   if( err )
   {
      err = LatchError( err, -1 );
      goto cleanup;
   }

   // Wait for drives in the linkage to clear abort bit - 100ms should be plenty of time
   // This ensures that when the semaphore is posted telling the linkage thread
   // to run, any abort bits will be clear and we can wait on a new event
   err = WaitEvent( trjAbortEvt, 100 );
   if (err == &ThreadError::Timeout) {
      cml.Debug( "Timeout waiting for the trj abort bits to clear in Linkage::StartMove\n" );
   }

   // Notify my thread that a move has started
   startSema.Put();

   // Reset the event chaining that I setup.  This ensures that the
   // events won't point to an event map that has been destroyed
   // already.
cleanup:
   for( i=0; i<ampct; i++ )
      events[i].delChain();

   return err;
}

/***************************************************************************/
/**
  Halt the current move.  The exact type of halt can be programmed individually 
  for each axis using the Amp::SetHaltMode function.
  @return An error object pointer, or NULL on success.
  */
/***************************************************************************/
const Error *Linkage::HaltMove( void )
{
   SetControlWord( 0x010F );
   return 0;
}

/***************************************************************************/
/**
  Set the linkage control word.

  @param value The control word value to set
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Linkage::SetControlWord( uint16 value )
{
   cml.Debug( "Link %d control 0x%04x\n", linkID, value );

   // For CANopen, I've got a single PDO that I broadcast to all the 
   // amplifiers in this linkage.
   if( netType == NET_TYPE_CANOPEN )
   {
       // transmit the RxPDO on the CANopen network.
       const Error *err = ctrlPDO.Transmit( value );
       if( err ) return err;

       // update the last control word private data member of each amp object.
       for( int i=0; i<ampct; i++ )
       {
          RefObjLocker<Amp> amp( ampRef[i] );
          if( amp ) amp->lastCtrlWord = value;
       }
   }

   // For EtherCAT, each amp has it's own PDO.
   else
   {
      // Lock a mutex used by the EtherCAT cyclic thread.  This ensures that 
      // all PDO values will be output at the same time
      RefObjLocker<EtherCAT> ecat( netRef );
      {
         MutexLocker ml( ecat->cyclicMutex );

         for( int i=0; i<ampct; i++ )
         {
            RefObjLocker<Amp> amp( ampRef[i] );
            if( amp )
            {
               amp->lastCtrlWord = value;
               amp->xmitCtrlPDO( value );
            }
         }
      }

      const Error *err = ecat->WaitCycleUpdate( 20 * ecat->GetCyclicPeriod() );
      if( err )
         cml.Debug( "Error waiting for EtherCAT cyclic task to run: %s\n", err->toString() );
      return err;
   }

   return 0;
}

/***************************************************************************/
/**
  Wait for a linkage event condition. This function additionally polls the
  drive status while waiting to ensure a lost message won't cause a timeout
  @param e The event to wait on.
  @param timeout The timeout for the wait (milliseconds).  If < 0, then 
  wait forever.
  @param match Returns the matching event condition.
  @param map Event map to wait on
  @return A pointer to an error object, or NULL on success.
  */
/***************************************************************************/
const Error *Linkage::WaitEvent( Event &e, Timeout timeout, LINK_EVENT &match, EventMap &map )
{
   Timeout poll;

   if( timeout < 0 )
      poll = 100;
   else
   {
      if( timeout < 10 )
         poll = timeout;
      else if( timeout < 200 )
         poll = timeout/2+1;
      else
         poll = 100;
      timeout -= poll;
   }

   const Error *err;
   do
   {
      err = e.Wait( map, poll );
      match = (LINK_EVENT)e.getMask();

      if( err != &ThreadError::Timeout )
         return err;

      cml.Debug( "Link wait event polling.  Event mask 0x%08x, map value: 0x%08x\n", e.getValue(), map.getMask() );

      if( poll )
      {
         for( int i=0; i<ampct; i++ )
         {
            RefObjLocker<Amp> amp( ampRef[i] );
            if( amp ) amp->RequestStatUpdt( (uint32)poll );
         }
      }

      // Find remaining timeout after this wait
      if( timeout >= 0 )
      {
         if( timeout < poll )
            poll = timeout;

         timeout -= poll;
      }
   } while( poll ); 

   return err;
}

/***************************************************************************/
/**
  Wait for a linkage event condition. This function can be used to wait
  on any generic event associated with the linkage.
  @param e The event to wait on.
  @param timeout The timeout for the wait (milliseconds).  If < 0, then 
  wait forever.
  @param match Returns the matching event condition.
  @return A pointer to an error object, or NULL on success.
  */
/***************************************************************************/
const Error *Linkage::WaitEvent( Event &e, Timeout timeout, LINK_EVENT &match )
{
   return WaitEvent( e, timeout, match, eventMap );
}

/***************************************************************************/
/**
  Wait for a linkage event condition. This function can be used to wait
  on any generic event associated with the linkage.
  @param e The event to wait on.
  @param timeout The timeout for the wait (milliseconds).  If < 0, then 
  wait forever (default).
  @return A pointer to an error object, or NULL on success.
  */
/***************************************************************************/
const Error *Linkage::WaitEvent( Event &e, Timeout timeout )
{
   LINK_EVENT match;
   return WaitEvent( e, timeout, match );
}

/***************************************************************************/
/**
  Wait for the currently running move to finish, or for an error to occur.

  @param timeout The maximum time to wait (milliseconds).  Default is -1 (forever).
  @return A pointer to an error object, or NULL on success.
  */
/***************************************************************************/
const Error *Linkage::WaitMoveDone( Timeout timeout )
{
   cml.Debug( "Link %d waiting on move\n", linkID );

   uint32 value = LINKEVENT_MOVEDONE | LINKEVENT_NODEGUARD | LINKEVENT_FAULT |
                  LINKEVENT_ERROR | LINKEVENT_DISABLED | LINKEVENT_QUICKSTOP | 
                  LINKEVENT_ABORT | LINKEVENT_TRSTP;

   EventAny e( value );

   LINK_EVENT match;
   const Error *err = WaitEvent( e, timeout, match );

   if (err) {
      cml.Warn( "Link %d WaitMoveDone returned error %s\n", linkID, err->toString() );
   }

   else
   {
      match = (LINK_EVENT)(match & value);
      if( match == LINKEVENT_MOVEDONE )
         return 0;

      // There should be a latched error
      int ndx;
      err = GetLatchedError( ndx );

      // If not, take a best guess
      if( !err ) 
         err = GetError( match );

      cml.Debug( "Linkage::WaitMoveDone returned: %s\n", err->toString() );
   }

   return err;
}

/***************************************************************************/
/**
  Return an error code for a failed move.  The passed mask identifies
  which amplifier in the linkage generated the error.
  @param mask The event mask that caused the error
  @return A pointer to an error object
  */
/***************************************************************************/
const Error *Linkage::GetError( uint32 mask )
{
   const Error *err, *someErr=0;
   int i, someAmp=-1;


   // Try to find an amplifier with an event mask equal 
   // to the passed mask.
   for( i=0; i<ampct; i++ )
   {
      RefObjLocker<Amp> amp( ampRef[i] );

      if( !amp )
         err = &LinkError::AmpRemoved;
      else
         err = amp->GetErrorStatus();
      if( !err ) continue;

      someErr = err;
      someAmp = i;

      AMP_EVENT ae;
      amp->GetEventMask( ae );

      if( mask == (uint32)ae )
      {
         LatchError( err, i );
         return err;
      }
   }

   // If no exact match was found, just return an error 
   // reported by one of the amplifiers.
   if( someAmp >= 0 )
   {
      LatchError( someErr, someAmp );
      return someErr;
   }

   // If all else fails, return a generic error 
   LatchError( &LinkError::UnknownAmpErr, -1 );
   return &LinkError::UnknownAmpErr;
}

#ifdef CML_LINKAGE_TRJ_BUFFER_SIZE
/***************************************************************************/
/**
  Get the next PVT segment.  This function is called by an amplifier trajectory
  object when it requires a new trajectory point and doesn't have one cached.
  The linkage trajectory is queried for it's next point, and after this point
  is converted from axis space to amplifier space, the point is distributed to
  all amplifier trajectory objects.

  @return A pointer to an error object, or NULL on success.
  */
/***************************************************************************/
const Error *Linkage::RequestNextTrjPoint( void )
{
   const Error *err;

   if (!linkTrjRef) {
      return &LinkError::NoActiveTrj;
   }

   MutexLocker ml( mtx );

   uunit pos[ CML_MAX_AMPS_PER_LINK ];
   uunit vel[ CML_MAX_AMPS_PER_LINK ];
   uint8 time{ 0 };

   RefObjLocker<LinkTrajectory> trj( linkTrjRef );
   if( !trj )
   {
      cml.Error( "LinkTrajectory deleted while still in use!\n" );
      return &LinkError::NoActiveTrj;
   }

   bool useVel = trj->UseVelocityInfo();
   err = trj->NextSegment( pos, vel, time );
   if( err ) return err;

   // Convert from the frame of each axis to the frame used by the drive
   if( useVel )
      err = ConvertAxisToAmp( pos, vel );
   else
      err = ConvertAxisToAmpPos( pos );

   if( err ) return err;

   for( int i=0; i<ampct; i++ )
      ampTrj[i].AddPoint( pos[i], vel[i], time, useVel );

   return 0;
}

/***************************************************************************/
/**
  Increment a local counter indicating that the linkage trajectory is in use
  by one more axis.
  @return A pointer to an error object, or NULL On success
  */
/***************************************************************************/
const Error *Linkage::IncTrjUseCount( void )
{
   if( linkTrjRef == 0 )
      return &LinkError::NoActiveTrj;

   RefObjLocker<LinkTrajectory> trj( linkTrjRef );
   if( !trj )
      return &LinkError::NoActiveTrj;

   if( trjUseCount++ == 0 )
      return trj->StartNew();
   else
      return 0;
}

/***************************************************************************/
/**
  Decrement a local counter indicating the number of axes currently using the
  linkage trajectory.
  @return A pointer to an error object, or NULL On success
  */
/***************************************************************************/
const Error *Linkage::DecTrjUseCount( void )
{
    if (linkTrjRef == 0) {
        return &LinkError::NoActiveTrj;
    }

    RefObjLocker<LinkTrajectory> trj( linkTrjRef );
    if (!trj) {
        return &LinkError::NoActiveTrj;
    }

    if( --trjUseCount == 0 )
    {
       trj->Finish();
       RefObj::ReleaseRef( linkTrjRef );
       linkTrjRef = 0;
    }

    return 0;
}
#endif

/***************************************************************************/
/**
  Update the status event map used by this linkage.  
  */
/***************************************************************************/

#define ERROR_EVENTS       (LINKEVENT_NODEGUARD | LINKEVENT_FAULT | LINKEVENT_ERROR | \
                            LINKEVENT_QUICKSTOP | LINKEVENT_ABORT | LINKEVENT_DISABLED | \
                            LINKEVENT_TRSTP)

void Linkage::UpdateStatus( void )
{
   MutexLocker ml( mtx );

   uint32 orMask = 0;
   uint32 andMask = 0xffffffff;

   uint32 errors = ERROR_EVENTS;

   if( cfg.haltOnPosWarn ) 
      errors |= LINKEVENT_POSWARN;

   if( cfg.haltOnVelWin )
      errors |= LINKEVENT_VELWIN;

   for( int i=0; i<ampct; i++ )
   {
      RefObjLocker<Amp> amp( ampRef[i] );

      if( !amp )
      {
         cml.Error( "Link %d, amp index %d removed\n", linkID, i );
         continue;
      }

      AMP_EVENT e;
      amp->GetEventMask( e );
      cml.Debug( "Link %d, Amp %d status 0x%08x\n", linkID, amp->GetNodeID(), (uint32)e );

      orMask |= (uint32)e;
      andMask &= (uint32)e;

      // If an error condition is being reported, latch it
      if( e & errors )
      {
         // Check for any real errors
         const Error *err = amp->GetErrorStatus( true );

         // If none, it's probably a tracking warning or velocity window
         if( !err )
         {
            if( e & LINKEVENT_POSWARN )
               err = &AmpError::TrackWarn;
            else if( e & LINKEVENT_VELWIN )
               err = &AmpError::VelWin;
            else
               err = &AmpError::Unknown;
         }

         LatchError( err, i );
         cml.Warn( "Link %d error latched for amp %d: %s\n", linkID, i, err->toString() );
      }
   }
   orMask &= ( ERROR_EVENTS | LINKEVENT_POSWARN | 
               LINKEVENT_POSWIN | LINKEVENT_VELWIN | 
               LINKEVENT_POSLIM | LINKEVENT_NEGLIM | 
               LINKEVENT_SOFTLIM_POS | LINKEVENT_SOFTLIM_NEG );

   andMask &= ( LINKEVENT_MOVEDONE | LINKEVENT_TRJDONE );

   orMask |= andMask;

   cml.Debug( "Link %d status: 0x%08x\n", linkID, orMask );
   eventMap.setMask( orMask );
}

/***************************************************************************/
/**
  Update the load to user unit converters in the LinkTrjScurve object for each 
  amplifier in the linkage. It is important to update them because they are 
  used in the LinkTrjScurve::NextSegment to transform pout and vout from load 
  to user units.
  */
/***************************************************************************/
void Linkage::UpdateLoadToUserUnitConverters( void ){
#ifdef CML_ENABLE_USER_UNITS
    uunit u2lPosArr[CML_MAX_AMPS_PER_LINK];
    uunit u2lVelArr[CML_MAX_AMPS_PER_LINK];

    // get the latest user-units from each amplifier in the linkage
    for (int i = 0; i < ampct; i++) {
        if (ampRef[i]) {
            RefObjLocker<Amp> amp(ampRef[i]);
            if (amp) {
                u2lPosArr[i] = amp->PosUser2Load(1);   // 1 counts units
                u2lVelArr[i] = amp->VelUser2Load(0.1); // 0.1 counts/sec units
            }
        }
    }    
    
    linkTrjScurveObj.UpdateUserToLoadUnitConverters(u2lPosArr, u2lVelArr);

#endif
}

/***************************************************************************/
/**
  Update the position wrap value for each axis in the linkage. The position
  wrap will be in firmware units. The position wrap value of the encoder 
  that is closing the position loop (not in passive mode) will be sent to 
  the linkTrjScurveObj.
*/
/***************************************************************************/
const Error* Linkage::UpdatePositionWrap(void) {
#ifdef CML_ENABLE_USER_UNITS
    const Error* err = 0;
    uunit positionWrapArrUserUnits[CML_MAX_AMPS_PER_LINK];

    // update the position wrap for each axis in the linkage
    for (int i = 0; i < ampct; i++) {
        if (ampRef[i]) {
            RefObjLocker<Amp> amp(ampRef[i]);
            if (amp) {
                positionWrapArrUserUnits[i] = 0;
                err = amp->GetPositionWrap(positionWrapArrUserUnits[i]);
            }
        }
    }

    linkTrjScurveObj.SetPositionWrapArrUserUnits(positionWrapArrUserUnits);
    return err;
#endif
}

/***************************************************************************/
/**
  Linkage thread.  This thread monitors the linkage during run time.
  */
/***************************************************************************/
void Linkage::run( void )
{
   const Error *err;
   EventAny  allDone( LINKEVENT_MOVEDONE );
   EventAny  doneEvent;
   int errAmp;

   uint32 doneValue(  LINKEVENT_MOVEDONE | LINKEVENT_NODEGUARD | LINKEVENT_FAULT |
                      LINKEVENT_ERROR | LINKEVENT_DISABLED | LINKEVENT_QUICKSTOP | 
                      LINKEVENT_ABORT | LINKEVENT_TRSTP );
   LINK_EVENT match;


   while( 1 )
   {
      // Wait for a move to start on this linkage.
      err = startSema.Get();

      // This should never fail, but if it does just delay and try again.
      if( err )
      {
         sleep( 100 );
         continue;
      }

      // Now, wait for the move to finish or an error to occur.
      cml.Debug( "Link thread waiting for move to finish.\n" );

      // Set or clear the position warning bit depending on whether
      // the linkage is configured to watch it.
      if( cfg.haltOnPosWarn )
         doneValue |= LINKEVENT_POSWARN;
      else
         doneValue &= ~LINKEVENT_POSWARN;

      // Same thing for velocity window
      if( cfg.haltOnVelWin )
         doneValue |= LINKEVENT_VELWIN;
      else
         doneValue &= ~LINKEVENT_VELWIN;

      // Wait for any of my selected events
      doneEvent.setValue( doneValue );

      err = WaitEvent( doneEvent, -1, match );
      match = (LINK_EVENT)(match & doneValue);

      if( err )
         cml.Debug( "Link %d error waiting on move done: %s\n",  linkID, err->toString() );

      else if( match == LINKEVENT_MOVEDONE )
      {
         cml.Debug( "Link thread done OK\n" );
         continue;
      }

      else if( (err = GetLatchedError( errAmp )) == 0 )
      {
         cml.Debug( "Link %d stopped move with unexpected event: 0x%08x\n", 
                    linkID, match );
      }

      else
         cml.Debug( "Link %d error from amp %d while waiting on move: %s\n",  
                    linkID, errAmp, err->toString() );

      // On error, halt the linkage
      HaltMove();

      // Finish the PVT trajectory on each amplifier in the linkage.
      for (int i = 0; i < ampct; i++) {
          if (ampRef[i]) {
              RefObjLocker<Amp> amp(ampRef[i]);
              if (amp) {
                  amp->FinishPvtTrj();
              }
          }
      }
      
      // wait here (blocking statement) until LINKEVENT_MOVEDONE bit is set.
      allDone.Wait( eventMap, -1 );
   }
}

/***************************************************************************/
/**
  When the status of an amplifier connected to this linkage is updated, this
  function is called.  It simply causes the linkage status to be updated also.
  @return false
  */
/***************************************************************************/
bool Linkage::StateEvent::isTrue( uint32 mask )
{
   link->UpdateStatus();
   return false;
}

#ifdef CML_LINKAGE_TRJ_BUFFER_SIZE
/***************************************************************************/
/**
  Initialize the amplifier trajectory structure.  This is called from the 
  Linkage object constructor.
  @param lptr Points to the linkage that owns this object.
  */
/***************************************************************************/
void Linkage::AmpTrj::Init( Linkage *lptr )
{
   linkPtr = lptr;
   head = tail = 0;
   inUse = false;
}

/***************************************************************************/
/**
Reset the local buffer used to store points.
*/
/***************************************************************************/
void Linkage::AmpTrj::Reset( void )
{
   head = tail = 0;
}

/***************************************************************************/
/**

*/
/***************************************************************************/
const Error *Linkage::AmpTrj::StartNew( void )
{
    if (inUse) {
        return &LinkError::AmpTrjInUse;
    }
    else {
        inUse = true;
        return linkPtr->IncTrjUseCount();
    }
}

/***************************************************************************/
/**

*/
/***************************************************************************/
void Linkage::AmpTrj::Finish( void )
{
   inUse = false;
   linkPtr->DecTrjUseCount();
}

/***************************************************************************/
/**

*/
/***************************************************************************/
const Error *Linkage::AmpTrj::AddPoint( uunit pos, uunit vel, uint8 time, bool useVel )
{
   int newHead = (head+1) % CML_LINKAGE_TRJ_BUFFER_SIZE;

   if( newHead == tail )
      return &LinkError::AmpTrjOverflow;

   ampTrjPositionArray[head] = pos;
   ampTrjVelocityArray[head] = vel;
   ampTrjTimeArray[head] = time;
   useVelArray[head] = useVel;

   head = newHead;

   return 0;
}

/***************************************************************************/
/**
   Get info about velocity usage from the linkage trajectory.
  */
/***************************************************************************/
bool Linkage::AmpTrj::UseVelocityInfo( void )
{
    if (head == tail) {
        linkPtr->RequestNextTrjPoint();
    }

   return useVelArray[tail];
}

/***************************************************************************/
/**
  Get the buffer size.  This just uses the buffer size info from the 
  owned Linkage trajectory
  */
/***************************************************************************/
int Linkage::AmpTrj::MaximumBufferPointsToUse( void )
{
   if( !linkPtr->linkTrjRef )
      return Trajectory::MaximumBufferPointsToUse();

   RefObjLocker<LinkTrajectory> trj( linkPtr->linkTrjRef );
   if( !trj )
      return Trajectory::MaximumBufferPointsToUse();

   return trj->MaximumBufferPointsToUse();
}

/***************************************************************************/
/**
  Get the next segment for the PVT move.
  */
/***************************************************************************/
const Error *Linkage::AmpTrj::NextSegment( uunit &pos, uunit &vel, uint8 &time )
{
   if( !inUse ) return &LinkError::AmpTrjNotRunning;

   const Error *err;

   if( head == tail )
   {
      err = linkPtr->RequestNextTrjPoint();
      if( err ) return err;
   }

   pos  = ampTrjPositionArray[tail];
   vel  = ampTrjVelocityArray[tail];
   time = ampTrjTimeArray[tail];

   tail = (tail+1) % CML_LINKAGE_TRJ_BUFFER_SIZE;

   return 0;
}
#endif

/***************************************************************************/
/**
  Latch an error if one isn't already being held.
  @param err Points to the error
  @param ndx Index of the amp that caused it, or -1 if not known.
  @return The new latched error.
  */
/***************************************************************************/
const Error *Linkage::LatchError( const Error *err, int ndx )
{
   cml.Debug( "Linkage error on amp index %d, %s\n", ndx, err->toString() );
   if( !latchedErr )
   {
      latchedErr = err;
      latchedErrAmp = ndx;
   }
   else
      cml.Debug( "Previous latched error for linkage index %d: %s\n", ndx, latchedErr->toString() );
   return latchedErr;
}

/***************************************************************************/
/**
  Initialize the receive PDO used to control words to each amplifier
  held by a linkage.  The COB ID used for this PDO is the standard
  ID used for RPDO 1 of the first axis.

  @return An error object pointer on failure, NULL on success
  */
/***************************************************************************/
const Error *RPDO_LinkCtrl::Init( void )
{
   const Error *err = 0;

   if( link.GetNetworkType() == NET_TYPE_CANOPEN )
   {
      RefObjLocker<Amp> amp( link.GetAmpRef( 0 ) );
      if( !amp ) return &LinkError::AmpRemoved;

      int32 cobID = 0x300 + amp->GetNodeID();
      err = RPDO::Init( cobID );
   }

   if( !err ) err = ctrl.Init( OBJID_CONTROL );
   if( !err ) err = AddVar( ctrl );
   if( !err ) err = SetType( 255 );
   if( err ) return err;

   for( int i=0; i<link.GetAmpCount(); i++ )
   {
      RefObjLocker<Amp> amp( link.GetAmpRef( i ) );
      if( !amp ) return &LinkError::AmpRemoved;

      // maps RxPDO that has COB-ID 0x301 to each amplifier in the linkage.
      err = amp->PdoSet( 1, *this );
      if( err ) return err;
   }

   return 0;
}

/***************************************************************************/
/**
  Transmit a control word using this PDO.
  @param c Value of the control word to send.
  @return A pointer to an error object, or NULL on success.
  */
/***************************************************************************/
const Error *RPDO_LinkCtrl::Transmit( uint16 c )
{
   ctrl.Write( c );

   RefObjLocker<Network> net( link.GetNetworkRef() );
   if( !net ) return &NodeError::NetworkUnavailable;

   return RPDO::Transmit( *net );
}

/***************************************************************************/
/**
  Default constructor.  All settings are set to their default values at
  construction time.
  */
/***************************************************************************/
LinkSettings::LinkSettings()
{
   moveAckTimeout = 200;
   haltOnPosWarn = false;
   haltOnVelWin = false;
}

