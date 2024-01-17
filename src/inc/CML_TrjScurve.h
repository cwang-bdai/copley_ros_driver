/********************************************************/
/*                                                      */
/*  Copley Motion Libraries                             */
/*                                                      */
/*  Copyright (c) 2002 Copley Controls Corp.            */
/*                     http://www.copleycontrols.com    */
/*                                                      */
/********************************************************/

/** \file

This file defines the TrjScurve class.  This class is used to 
calculate asymmetric S-curve trajectory profiles for use by 
the Amp or Linkage objects.

*/

#ifndef _DEF_INC_TRJSCURVE
#define _DEF_INC_TRJSCURVE

#include "CML_Settings.h"
#include "CML_Trajectory.h"

CML_NAMESPACE_START()

/***************************************************************************/
/**
This class represents error conditions that can occur in the TrjScurve class.
*/
/***************************************************************************/
class ScurveError: public Error
{
public:
   static const ScurveError BadParam;            ///< Illegal input parameter
   static const ScurveError NoCalc;              ///< Trjaectory has not been calculated
   static const ScurveError InUse;               ///< Trajectory is currently in use
   static const ScurveError NotInUse;            ///< Trajectory has not been started
   static const ScurveError ChangeInPosTooGreat; ///< A change in time of 1ms produces a change in position greater than 0x7FFFFF (won't fit in PDO)
   static const ScurveError ChangeInVelTooGreat; ///< A change in time of 1ms produces a change in velocity greater than 0x7FFFFF (won't fit in PDO)

protected:
   /// Standard protected constructor
   ScurveError( uint16 id, const char *desc ): Error( id, desc ){}
};

/***************************************************************************/
/**
Asymmetric S-curve profile generator.  

A symmetric S-curve profile uses the same constraint for acceleration & 
deceleration.  Asymmetric profiles use different acceleration & deceleration
values.  Copley amplifiers are able to calculate symmetric profiles internally,
however if asymmetric s-curve profiles are required then they must be calculate
external to the amplifier, and passed to it using PVT profile mode.

This class extends the generic Trajectory class, and provides the code necessary
to calculate an asymmetric s-curve profile.  Since it extends the Trajectory
object, it may be passed to the Amp::SendTrajectory function.

Internally, the s-curve profile is stored as an absolute move from some starting
position.  This allows the same trajectory object to be reused for multiple moves
of the same distance from different starting positions.  The starting position
may be either passed to the TrjScurve::Calculate function, or set using 
TrjScurve::SetStartPos.
*/
/***************************************************************************/
class TrjScurve: public Trajectory 
{
   bool inUse;
   bool initialized;
   double P, V, A, D, J;
   double tj, tk, ta, td, tv;

   // user unit convertors. Give them default values.
   double u2lPos = 1;
   double u2lVel = 1;
   double l2uPos = 1;
   double l2uVel = 1;
   double scale = 1;
   
   bool usePositionWrap = false;
   int positionWrap = 0; // default wrap position is 0
   uunit posWrapUserUnits = 0;
   
   bool advanceMove = true;

   uunit startPos;

   int moveSeg;
   double p,v,a,j;
   double timeRemain;
   double max24BitDataSize = 0x007FFFFF;

   int32 PosUser2Load(uunit pos);
   uunit PosLoad2User(int32 pos);
   int32 VelUser2Load(uunit vel);
   uunit VelLoad2User(int32 vel);

   int AdvanceSegment( double &tf );

   /// Private copy constructor (not supported)
   TrjScurve( const TrjScurve& );

   /// Private assignment operator (not supported)
   TrjScurve& operator=( const TrjScurve& );

public:
   TrjScurve();
   ~TrjScurve(){ KillRef(); }

   const Error *SetPositionWrap( double wrapPosition );
   void  SetUserUnitsAndScale( double u2lPos, double u2lVel, double scale );
   void  SetAdvanceMove(bool value);
   void  SetStartPos( uunit s );
   uunit GetStartPos( void );
   const Error *Calculate( uunit start, uunit end, uunit vel, uunit acc, uunit dec, uunit jrk );
   const Error *Calculate( uunit dist, uunit vel, uunit acc, uunit dec, uunit jrk );

   const Error *StartNew( void );
   void Finish( void );
   const Error *NextSegment( uunit &pos, uunit &vel, uint8 &time );
};

/***************************************************************************/
/**
Multi-axis s-curve profile.  This extends the single axis TrjScurve object
for use in multi-axis linkage moves.
*/
/***************************************************************************/
class LinkTrjScurve: public LinkTrajectory
{
   TrjScurve trjScurveObj;
   Point<CML_MAX_AMPS_PER_LINK> startingPosLnkTrjScurve;
   double scale[CML_MAX_AMPS_PER_LINK];
   double u2lPosArr[CML_MAX_AMPS_PER_LINK];
   double l2uPosArr[CML_MAX_AMPS_PER_LINK];
   double u2lVelArr[CML_MAX_AMPS_PER_LINK];
   double l2uVelArr[CML_MAX_AMPS_PER_LINK];

   bool isRelative = false;

   bool usePositionWrapArr[CML_MAX_AMPS_PER_LINK];
   int positionWrapLoadUnitsArr[CML_MAX_AMPS_PER_LINK]; // default wrap position is 0
   uunit posWrapUserUnitsArr[CML_MAX_AMPS_PER_LINK];

   int32 PosUser2Load(uunit pos);
   int32 PosUser2Load(uunit pos, int axis);

   /// Private copy constructor (not supported)
   LinkTrjScurve( const LinkTrjScurve& );

   /// Private assignment operator (not supported)
   LinkTrjScurve& operator=( const LinkTrjScurve& );

public:
   LinkTrjScurve();
   ~LinkTrjScurve(){ KillRef(); }

   const Error *Calculate( PointN &start, PointN &end, uunit vel, uunit acc, uunit dec, uunit jrk );
   int GetDim( void ){ return startingPosLnkTrjScurve.getDim(); }

   const Error *StartNew( void );
   void Finish( void );
   const Error *NextSegment( uunit pos[], uunit vel[], uint8 &time );
   const Error *SetPositionWrapArrUserUnits(uunit positionWrapArrIn[]);
   void UpdateUserToLoadUnitConverters(uunit l2uPosArrIn[], uunit l2uVelArrIn[]);
   void SetIsRelative(bool isRelative);
};

CML_NAMESPACE_END()

#endif

