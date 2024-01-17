/********************************************************/
/*                                                      */
/*  Copley Motion Libraries                             */
/*                                                      */
/*  Copyright (c) 2002 Copley Controls Corp.            */
/*                     http://www.copleycontrols.com    */
/*                                                      */
/********************************************************/

#include "CML_Settings.h"
#ifdef CML_ALLOW_FLOATING_POINT

#include <math.h>
#include "CML.h"

CML_NAMESPACE_USE();

// macro used for rounding floats
#define Round(x)  ((x>=0) ? (x+0.5) : (x-0.5))

CML_NEW_ERROR( ScurveError, BadParam,  "An illegal input parameter was passed" );
CML_NEW_ERROR( ScurveError, NoCalc,    "Trajectory has not been calculated" );
CML_NEW_ERROR( ScurveError, InUse,     "Trajectory is currently in use" );
CML_NEW_ERROR( ScurveError, NotInUse,  "Trajectory has not been started" );
CML_NEW_ERROR( ScurveError, ChangeInPosTooGreat, "A change in time of 1ms produces a change in position greater than 0x7FFFFF (won't fit in PDO)" );
CML_NEW_ERROR( ScurveError, ChangeInVelTooGreat, "A change in time of 1ms produces a change in velocity greater than 0x7FFFFF (won't fit in PDO)" );

/***************************************************************************/
/**
S-curve trajectory default constructor.  This simply sets the profile to 
zero length with a starting position of zero.
*/
/***************************************************************************/
TrjScurve::TrjScurve( void )
{
   startPos = 0;
   inUse = false;
   initialized = false;
}

/***************************************************************************/
/**
Set the trajectory starting position.  S-curve profiles are internally stored
as absolute moves of some length.  This allows them to be used multiple times 
with different starting positions.  

This function may be used to update the starting position of the trajectory.

@param s The new starting position
*/
/***************************************************************************/
void TrjScurve::SetStartPos( uunit s )
{
   startPos = s;
   return;
}

/***************************************************************************/
/**
Return the current starting position of the trajectory.  The starting position
will either be the value set using TrjScurve::SetStartPos, or the value set 
using TrjScurve::Calculate.  If neither has been called since construction, 
then the starting position will be zero.

@return The trajectory starting position.
*/
/***************************************************************************/
uunit TrjScurve::GetStartPos( void )
{
   return startPos;
}

/***************************************************************************/
/**
Calculate a new S-curve profile and also set it's starting position.

@param start The profile starting position.
@param end The profile ending position.
@param vel The maximum allowable velocity for the move.
@param acc The maximum allowable acceleration for the move.
@param dec The maximum allowable deceleration for the move.
@param jrk The maximum jerk (rate of change of acceleration) for the move.

@return A pointer to an error object, or NULL on success.

Note: If encoder wrap is enabled, the ending position "end" will represent
      a relative position command, not absolute.
*/
/***************************************************************************/
const Error *TrjScurve::Calculate( uunit start, uunit end, uunit vel, uunit acc, uunit dec, uunit jrk )
{
    uunit dist = 0; 

    // if we use encoder wrap, the end position is now considered a relative position command
    if (usePositionWrap) {

        dist = end;
    }
    else {
        dist = end - start;
    }

   const Error *err = Calculate( dist, vel, acc, dec, jrk );
   if( err ) return err;
   SetStartPos( start );
   return 0;
}

/***************************************************************************/
/**
Calculate a new S-curve profile.  The resulting profile may then be sent 
to an Amp object using the Amp::SendTrajectory method.

Note, all profile parameters are passed in 'user units'.  See the 
documentation for Amp::SetCountsPerUnit for details.

Note also that this function calculates the profile as an absolute move from
the starting position that is set using TrjScurve::SetStartPos.  The same
profile may be used multiple times with different starting positions without
calling the TrjScurve::Calculate function again.

@param dist The distance to move.
@param maxVel The maximum allowable velocity for the move.
@param maxAcc The maximum allowable acceleration for the move.
@param maxDec The maximum allowable deceleration for the move.
@param maxJrk The maximum jerk (rate of change of acceleration) for the move.

@return A pointer to an error object, or NULL on success.

The distance will always be met exactly.  This may be either positive or
negative.
  
The velocity, acceleration, deceleration and jerk values are constraints 
and won't be exceeded.  These must all be positive numbers greater than zero.
*/
/***************************************************************************/
const Error *TrjScurve::Calculate( uunit dist, uunit maxVel, uunit maxAcc, uunit maxDec, uunit maxJrk )
{
   /**************************************************
    * Do some sanity checks on the input limits.
    **************************************************/
    if (maxVel <= 0 || maxAcc <= 0 || maxDec <= 0 || maxJrk <= 0) {
       return &ScurveError::BadParam;
    }

    if( inUse ) return &ScurveError::InUse;

   /**************************************************
    * Convert to floating point units if necessary
    **************************************************/
#ifdef CML_ENABLE_USER_UNITS
   P = dist;
   V = maxVel;
   A = maxAcc;
   D = maxDec;
   J = maxJrk;
#else
   P = dist;
   V = maxVel * 0.1;
   A = maxAcc * 10.0;
   D = maxDec * 10.0;
   J = maxJrk * 100.0;
#endif

   /**************************************************
    * Assume positive moves for simplicity.  We'll fix
    * this in the end if necessary.
    **************************************************/
   // declare and initialize local variable
   bool negMove = false;
   
   // if P < 0, set negMove to true (move in negative direction).
   if (P < 0) {
       negMove = true;
   }

   // if we are moving in the negative direction, multiply the 
   // commanded position by -1 to make it positive.
   if (negMove) {
       P *= -1.0;
   }

   // handle the case where we are already at the commanded position.
   if( P==0 ) {
      tj = tk = ta = td = tv = 0;
      initialized = true;
      return 0;
   }

   /**************************************************
    * Make sure maxAccel <= maxDecel.  This reduces the number of 
    * tests I need to do later.  I'll fix this at the
    * end of the calculation.
    **************************************************/
   bool swapAD = (A > D);

   if( swapAD )
   {
      double tmp;
      tmp = A; A = D; D = tmp;
   }

   /**************************************************
    * I'll lower jerk to ensure that my jerk segments
    * are at least 1 millisecond long.
    **************************************************/
   if (J > A * 1e3) { 
       J = A * 1e3; 
   }
   if (J > V * 1e6) { 
       J = V * 1e6; 
   }
   if (J > P * 5e8) { 
       J = P * 5e8; 
   }

   /**************************************************
    * These are the key variables I'll need to find.
    *   tj = time to increase/decrease acceleration
    *   ta = time to run at constant accel
    *   tv = time to run at constant velocity
    *   td = time to run at constant decel
    *   tk = time to increase/decrease deceleration
    **************************************************/

   /**************************************************
    * See if a simple jerk limited move will handle 
    * this.  In this case, the move will be 4 segments
    * each of the same time.
    **************************************************/
   
   // the time at which acceleration is to be adjusted = cuberoot(counts/(counts/second^3)) = seconds.
   tj = pow( P/(2*J), 1.0/3.0 );

   // see if the time fits under the current max accel and max velocity limits.
   if( (J*tj < A) && (J*tj*tj < V) )
   {
      // we will not need to travel at the max velocity, accel, or decel.
      ta = td = tv = 0;

      // Adjust accel and decel time to the next higher millisecond.
      tk = tj = 0.001 * ceil( tj*1000 );

      // Adjust jerk to the next higher millisecond.
      J  = P / (2 * tj*tj*tj);
   }

   /**************************************************
    * We know we'll hit either the accel or velocity 
    * limit.  See if the accel limit is too high.
    * If so, this must be a Jerk & Velocity move.
    **************************************************/
   else if( J*V < A*A )
   {
      ta = td = 0;
      tj = sqrt( V/J );

      // Adjust the times so they are integer multiples
      // of milliseconds.  I'll adjust J & V to compensate
      tk = tj = 0.001 * ceil( tj*1000 );
      tv = P/V - 2*tj;

      tv = 0.001 * ceil( tv*1000 );

      V = P / (tv + 2*tj);
      J = V / (tj*tj);
   }

   else 
   {
      /**************************************************
       * At this point we know we will hit the accel limit.
       * We may or may not hit the velocity & decel limits.
       * I'll start by assuming that I'll hit the velocity
       * limit.
       **************************************************/
      double vj, vk;

      // calculate the time when we will adjust the acceleration.
      tj = A/J;

      // area under the acceleration triangle.
      vj = A*tj / 2.0;

      // calculate the time at which to run at a constant acceleration.
      ta = (V - 2*vj) / A;

      // if the 
      if( J*V < D*D )
      {
         td = 0.0;
         tk = sqrt(V/J);
         vk = V/2;
      }
      else
      {
         tk = D/J;
         td = (V-J*tk*tk) / D;
         vk = D*tk / 2.0;
      }

      // Find the distance moved getting up to 
      // and down from V
      double pa = tj*vj*2 + ta*vj + A*ta*tj + ta*ta*A/2.0;
      double pd = tk*vk*2 + td*vk + D*td*tk + td*td*D/2.0;

      // If this distance is less then the total move,
      // then I've found my solution.  Otherwise, the
      // velocity limit isn't reached.
      if( pa+pd <= P ){
         tv = (P-pa-pd) / V;
      }
      else
      {
         /**************************************************
          * At this point, we know we will hit the accel 
          * limit, but not the velocity limit.  The only 
          * question now is whether the decel limit will 
          * be reached.
          *
          * I'll try no decel limit first.
          **************************************************/
         tv = 0.0;
         tk = (sqrt( sqrt(2*P*A)*4*J +A*A ) - A) / (2*J);

         if( J*tk <= D )
         {
            ta = (J*tk*tk - J*tj*tj) / A;
            td = 0.0;
         }

         else
         {
            tk = D/J;

            double a = J*A*(D+A);
            double b = 3*A*A*D -2*A*D*D +2*A*A*A +3*A*D*D;
            double c = (A*A + D*D + 2*A*D) * A*A/J - 2*P*J*D;

            ta = (-b + sqrt(b*b -4*a*c)) / (2*a);
            td = (J*tj*tj + A*ta - J*tk*tk)/D;
         }
      }
   }

   /**************************************************
    * If I previously swapped A & D, fix that now.
    **************************************************/
   if( swapAD )
   {
      double tmp;
      tmp = ta; ta = td; td = tmp;
      tmp = tj; tj = tk; tk = tmp;
      tmp =  A;  A =  D;  D = tmp;
   }

   /**************************************************
    * Adjust for negative moves as necessary
    **************************************************/
   if( negMove )
   {
      P  *= -1.0;
      J  *= -1.0;
   }

   initialized = true;
   return 0;
}

/***************************************************************************/
/**
Reset this object so it may be passed to an amplifier.  This will return an
error if the trajectory has not yet been calculated, or if it is currently 
being sent to another amp.

@return A pointer to an error object, or NULL on success
*/
/***************************************************************************/
const Error *TrjScurve::StartNew( void )
{
   if( !initialized ) return &ScurveError::NoCalc;
   if( inUse ) return &ScurveError::InUse;

   // the trajectory is currently in use.
   inUse = true;

   p = startPos;
   v = a = 0;
   j = J;
   timeRemain = tj;
   moveSeg = 0;

   return 0;
}

/***************************************************************************/
/**
Notify the trajectory object that it is no longer in use.
*/
/***************************************************************************/
void TrjScurve::Finish( void )
{
   inUse = false;
}

/***************************************************************************/
/**
Get the next PVT segment for this s-curve profile.
*/
/***************************************************************************/
const Error *TrjScurve::NextSegment( uunit &pout, uunit &vout, uint8 &tout )
{
   if( !inUse ) return &ScurveError::NotInUse;

   /**************************************************
    * We have previously calculate the position & 
    * velocity for this segment.  I'll set the output
    * values now.
    **************************************************/
#ifdef CML_ENABLE_USER_UNITS
   pout = p;
   vout = v;
#else
   if( p >= 0 ) pout = (uunit)(p + 0.5);
   else         pout = (uunit)(p - 0.5);

   if( v >= 0 ) vout = (uunit)(10.0 * v + 0.5);
   else         vout = (uunit)(10.0 * v - 0.5);
#endif

   if( moveSeg == 7 )
   {
      tout = 0;
      return 0;
   }

   bool endOfSegment = false;
   double timeLocal = 0;

   if( timeRemain > 0.510 )
   {
      timeLocal = 0.255;
      tout = 255;
   }

   else if( timeRemain > 0.254 )
   {
      tout = (uint8)floor(500*timeRemain); // converting seconds to ms and multiplying by 0.5
      timeLocal = 0.001 * tout;
   }

   // the end of the segment has been reached.
   else
   {
      endOfSegment = true;
      
      tout = (uint8)floor(timeRemain*1000 + 0.01);
      timeLocal = timeRemain;
   }

   double deltaVelocity = 0;
   
   // compute the change in velocity between the last commanded velocity and the new commanded velocity
   deltaVelocity = (a* timeLocal + j * timeLocal * timeLocal / 2);
   deltaVelocity = VelUser2Load(deltaVelocity) * scale;  // need to convert from user to load units. Need a scale in case of a linkage.

   // if the change in velocity is too big to fit into a PDO, solve for the time value that will fit
   // 0x007FFFFF into it (maximum velocity value). The time is guaranteed to be less than 255 for 
   // this case, since whatever we used previously yielded something with an absolute value greater 
   // than 0x007FFFFF.
   if (deltaVelocity > max24BitDataSize || -deltaVelocity > max24BitDataSize) {
       
       endOfSegment = false;

       double changeInTime = timeLocal / 2;

       // first, cut the time value in half
       timeLocal = timeLocal - changeInTime;

       // perform a binary search for the time value. 
       // time is uint8 (eight bits in size).
       // 2^8 = 256. The search will have a max of 8 iterations.
       do {
           // compute the change in velocity between the last commanded velocity and the new commanded velocity
           deltaVelocity = a * timeLocal + j * timeLocal * timeLocal / 2;
           deltaVelocity = VelUser2Load(deltaVelocity) * scale;
           changeInTime = changeInTime / 2;

           // adjust the timeLocal using half its value
           if (deltaVelocity > max24BitDataSize || -deltaVelocity > max24BitDataSize) {
               timeLocal = timeLocal - changeInTime;
           }
           else {
               timeLocal = timeLocal + changeInTime;
           }

           // if changeInTime drops below 1ms, stop adjusting timeLocal. 
       } while (changeInTime > 0.001);

       // if the last calculation put us slightly over the limit, let's adjust it.
       deltaVelocity = a * timeLocal + j * timeLocal * timeLocal / 2;
       deltaVelocity = VelUser2Load(deltaVelocity) * scale;
       if (deltaVelocity > max24BitDataSize || -deltaVelocity > max24BitDataSize) {
           timeLocal = timeLocal - changeInTime;
       }

       // handle the case where time is less than 1ms (not valid). Test if 1ms falls into acceptable range.
       if (timeLocal < 0.001) {
           double deltaVel1ms = a * 0.001 + j * 0.001 * 0.001 / 2;
           deltaVel1ms = VelUser2Load(deltaVel1ms) * scale;
           if (deltaVel1ms > max24BitDataSize || -deltaVel1ms > max24BitDataSize) {
               return &ScurveError::ChangeInVelTooGreat; // there is no solution for the given trajectory parameters (change in velocity is too big even for 1ms change in time)
           }
           else {
               tout = 1;
               timeLocal = 0.001;
           }
       }

       tout = (uint8)floor(timeLocal * 1000); // convert to uint8 by flooring the variable
       timeLocal = (double)tout / 1000; // copy the floored value back to timeLocal for further calculations
   }

   double deltaPosition = 0;

   // compute the change in position between the last commanded position and the new commanded position
   deltaPosition = v * timeLocal + a * timeLocal * timeLocal / 2 + j * timeLocal * timeLocal * timeLocal / 6;
   deltaPosition = PosUser2Load(deltaPosition) * scale;

   // if the change in position is too big to fit into a PDO, solve for the time value that will fit
   // 0x007FFFFF into it (maximum position value). The time is guaranteed to be less than 255 for 
   // this case, since whatever we used previously yielded something with an absolute value greater 
   // than 0x007FFFFF.
   if (deltaPosition > max24BitDataSize || -deltaPosition > max24BitDataSize) {

       endOfSegment = false;

       double changeInTime = timeLocal / 2;

       // first, cut the time value in half
       timeLocal = timeLocal - changeInTime;

       // perform a binary search for the time value. 
       // time is uint8 (eight bits in size).
       // 2^8 = 256. The search will have a max of 8 iterations.
       do {

           // compute the change in position between the last commanded position and the new commanded position
           deltaPosition = v * timeLocal + a * timeLocal * timeLocal / 2 + j * timeLocal * timeLocal * timeLocal / 6;
           deltaPosition = PosUser2Load(deltaPosition) * scale;

           changeInTime = changeInTime / 2;

           // adjust the timeLocal using half its value
           if (deltaPosition > max24BitDataSize || -deltaPosition > max24BitDataSize) {
               timeLocal = timeLocal - changeInTime;
           }
           else {
               timeLocal = timeLocal + changeInTime;
           }

        // if changeInTime drops below 1ms, stop adjusting timeLocal. 
       } while (changeInTime > 0.001);

       // if the last calculation put us slightly over the limit, let's adjust it.
       deltaPosition = v * timeLocal + a * timeLocal * timeLocal / 2 + j * timeLocal * timeLocal * timeLocal / 6;
       deltaPosition = PosUser2Load(deltaPosition) * scale;

       if (deltaPosition > max24BitDataSize || -deltaPosition > max24BitDataSize) {
           timeLocal = timeLocal - changeInTime;
       }

       // handle the case where time is less than 1ms (not valid). Test if 1ms falls into acceptable range.
       if (timeLocal < 0.001) {
           double deltaPos1ms = v * 0.001 + a * 0.001 * 0.001 / 2 + j * 0.001 * 0.001 * 0.001 / 6;
           deltaPos1ms = PosUser2Load(deltaPos1ms) * scale;

           if (deltaPos1ms > max24BitDataSize || -deltaPos1ms > max24BitDataSize) {
               return &ScurveError::ChangeInPosTooGreat; // there is no solution for the given trajectory parameters (change in position is too big even for 1ms change in time)
           }
           else {
               tout = 1;
               timeLocal = 0.001;
           }
       }

       tout = (uint8)floor(timeLocal * 1000); // convert to uint8 by flooring the variable
       timeLocal = (double)tout / 1000; // copy the floored value back to timeLocal for further calculations
   }

   if (advanceMove) {

       p += v * timeLocal + a * timeLocal * timeLocal / 2 + j * timeLocal * timeLocal * timeLocal / 6;
       v += a * timeLocal + j * timeLocal * timeLocal / 2;
       a += j * timeLocal;
       
       // decrease the time remaining in the segment
       timeRemain -= timeLocal;
       if (timeRemain < 0) {
           timeRemain = 0;
       }
   }

   // We're done if that wasn't the end of a segment.
   if( !endOfSegment )
      return 0;

   // There's a good chance that the previous segment time 
   // wasn't an even number of milliseconds.  Find the amount
   // of time I went over the millisecond mark.
   double timeMilliseconds;
   double flt = modf( 1000*timeLocal, &timeMilliseconds );
   double timeFloat = 0;

   // Increment my output time if I didn't end on an even millisecond.
   if( flt > 0.001 )
   {
      timeFloat = 0.001 * (1.0 - flt);

      tout = round(timeMilliseconds) + 1;
   }

   if (advanceMove) {

       // Now, keep advancing to the next segment until I hit the end of
       // the millisecond, or come to the end of the move.
       while( AdvanceSegment(timeFloat) ){}
   }

   return 0;
}

/***************************************************************************/
/**
Move to the next s-curve segment with a non-zero time.  I also adjust my
local position, vel, etc values using the passed time.

@param advanceTime The time value used to advance pos, vel, acc in the new segment.
@return zero if finished with move or extra time
*/
/***************************************************************************/
int TrjScurve::AdvanceSegment( double &advanceTime )
{
   // Find the next segment with a non-zero time
   switch( moveSeg++ )
   {
      case 0: 
          timeRemain = ta; 
          j =  0; 
          if (timeRemain > 0.000001) { 
              break; 
          }
          moveSeg++;
              // Fall through
      case 1: 
          timeRemain = tj; 
          j = -J; 
          break;
      case 2: 
          timeRemain = tv; 
          j =  0; 
          if (timeRemain > 0.000001) { 
              break; 
          } 
          moveSeg++;
              // Fall through
      case 3: 
          timeRemain = tk; 
          j = -J; 
          break;
      case 4: 
          timeRemain = td; 
          j =  0; 
          if (timeRemain > 0.000001) { 
              break; 
          } 
          moveSeg++;
              // Fall through
      case 5: 
          timeRemain = tk; 
          j =  J; 
          break;
      default: 
          return 0;
   }

   // Advance the p,v,a values by the amount passed, or by
   // the total time in this segment if it's less.
   int done = (timeRemain >= advanceTime);
   double timeUsed = done ? advanceTime : timeRemain;

   p += v*timeUsed + a*timeUsed*timeUsed/2 + j*timeUsed*timeUsed*timeUsed/6;
   v += a*timeUsed + j*timeUsed*timeUsed/2;
   a += j*timeUsed;

   // Reduce the total time left in this segment by the value used.
   timeRemain -= timeUsed;
   advanceTime -= timeUsed;
   return !done;
}

/***************************************************************************/
/**
Update the user-unit convertors and scale so that the NextSegment method can
check if the change in position is greater than 0x7FFFFF.
*/
/***************************************************************************/
void TrjScurve::SetUserUnitsAndScale(double u2lPosIn, double u2lVelIn, double scaleIn) {
    u2lPos = u2lPosIn;
    u2lVel = u2lVelIn;
    scale = scaleIn;
    l2uPos = 1.0 / u2lPos;
}

/***************************************************************************/
/**
Update the private boolean data member variables "wrapPosition." 
*/
/***************************************************************************/
const Error *TrjScurve::SetPositionWrap( double wrapPositionIn ) {
    
    const Error* err = 0;
    if (wrapPositionIn < 0) {
        return &ScurveError::BadParam;
    }

    positionWrap = (int)wrapPositionIn;
    posWrapUserUnits = PosLoad2User(positionWrap);
    
    if (positionWrap == 0) {
        usePositionWrap = false;
    }
    else {
        usePositionWrap = true;
    }

    return err;
}

/***************************************************************************/
/**
Convert a position from user position units to internal amplifier units.

@param pos The position in user units
@return The position in encoder counts
*/
/***************************************************************************/
int32 TrjScurve::PosUser2Load(uunit pos)
{
#ifdef CML_ENABLE_USER_UNITS
    pos *= u2lPos;
    return (int32)Round(pos);
#else
    return pos;
#endif
}

/***************************************************************************/
/**
Convert a position from user units to internal amplifier units.

@param pos The position in encoder counts
@return The position in user units
*/
/***************************************************************************/
uunit TrjScurve::PosLoad2User(int32 pos)
{
#ifdef CML_ENABLE_USER_UNITS
    return pos * l2uPos;
#else
    return pos;
#endif
}

/***************************************************************************/
/**
Convert a velocity from user velocity units to internal amplifier units.

@param pos The velocity in user units
@return The velocity in encoder counts
*/
/***************************************************************************/
int32 TrjScurve::VelUser2Load(uunit vel)
{
#ifdef CML_ENABLE_USER_UNITS
    vel *= u2lVel;
    return (int32)Round(vel);
#else
    return vel;
#endif
}

/***************************************************************************/
/**
Convert a velocity from internal amplifier units to user units

@param vel The velocity in 0.1 encoder counts / second
@return The velocity in user units
*/
/***************************************************************************/
uunit TrjScurve::VelLoad2User(int32 vel)
{
#ifdef CML_ENABLE_USER_UNITS
    return vel * l2uVel;
#else
    return vel;
#endif
}

/***************************************************************************/
/**
Update the private boolean data member variable "advanceMove" used by the 
NextSegment method. If it's true, NextSegment will decrease the time remaining
in the scurve move. If it's false, it will not.
*/
/***************************************************************************/
void TrjScurve::SetAdvanceMove(bool value) {
    advanceMove = value;
}


/***************************************************************************/
/**
Default constructor for multi-axis s-curve trajectory.
*/
/***************************************************************************/
LinkTrjScurve::LinkTrjScurve( void ){}

/***************************************************************************/
/**
Calculate a multi-axis s-curve trajectory.  This function calculates the 
straight line move between the two passed positions.
@param s The starting position
@param e The ending position
@param vel The max velocity
@param acc The max acceleration
@param dec The max deceleration
@param jrk The max jerk (rate of change of velocity)
@return A pointer to an error object, or NULL on success.
*/
/***************************************************************************/
const Error *LinkTrjScurve::Calculate( PointN &startingPosArg, PointN &endingPosArg, uunit maxVelArgument, 
                                       uunit maxAccelArgument, uunit maxDecelArgument, uunit maxJerkArgument )
{
   // get the dimension of the move.
   int dimension = startingPosArg.getDim();

   // Ensure that the dimensions of the move fall within the legal limits.
   CML_ASSERT( dimension <= CML_MAX_AMPS_PER_LINK );

   double startingPosOriginal[CML_MAX_AMPS_PER_LINK];

   // if we are using position wrap on certain axes in the linkage, 
   // make the move distance relative by setting the starting 
   // position for those axes to 0.
   for (int i = 0; i < dimension; i++) {
       startingPosOriginal[i] = startingPosArg[i];
       if (usePositionWrapArr[i] || isRelative) {
           startingPosArg[i] = 0;
       }
   }

   // calculate the total distance of the move.
   uunit distanceTotal = startingPosArg.distance( endingPosArg );

   // calculate the trajectory parameters for each segment of the PVT move.
   const Error *err = trjScurveObj.Calculate( distanceTotal, maxVelArgument, maxAccelArgument, maxDecelArgument, maxJerkArgument );
   if( err ) return err;

   // find the inverted distance. If the total distance == 0, set the inverted distance to 0.0.
   // Else, set the inverted distance equal to 1 / distanceTotal.
   double distanceInverted = (distanceTotal!=0) ? 1.0/distanceTotal : 0.0;

   // set the dimension of the starting position.
   startingPosLnkTrjScurve.setDim( dimension );

   // adjust the starting position and scale for all axes.
   for( int i=0; i<dimension; i++ )
   {
      // update the starting position
      startingPosLnkTrjScurve[i] = startingPosOriginal[i];

      // update the scale
      scale[i] = (endingPosArg[i] - startingPosArg[i]) * distanceInverted;

      // set the starting position back to what it was originally
      startingPosArg[i] = startingPosOriginal[i];
   }

   return 0;
}

/***************************************************************************/
/**
Start a new move using this trajectory.  The trajectory must have already
been calculated when this function is called.
@return A pointer to an error object, or NULL on success.
*/
/***************************************************************************/
const Error *LinkTrjScurve::StartNew( void )
{
   return trjScurveObj.StartNew();
}

/***************************************************************************/
/**
Retrieve the next segment of this trajectory.  The positions & velocities for
all axes are returned in the passed arrays.

@param pos An array which will be filled with position information.  
@param vel An array which will be filled with velocity information.
@param time A reference to a variable where the time (milliseconds) will be
       returned.
@return A pointer to an error object, or NULL on success.
*/
/***************************************************************************/
const Error *LinkTrjScurve::NextSegment( uunit pos[], uunit vel[], uint8 &time )
{
    const Error* err = 0;
    uunit nextCommandedPos = 0;
    uunit nextMaxVelocity = 0;
    int index = 0;

    int axisCt = startingPosLnkTrjScurve.getDim();

    // use the user units and scale of the axis that requires the lowest time to 
    // increment the trjScurveObj
    if (axisCt > 1) {
        uint8* timeArr = new uint8[axisCt];

        trjScurveObj.SetAdvanceMove(false);
        for (int i = 0; i < axisCt; i++) {
            trjScurveObj.SetUserUnitsAndScale( u2lPosArr[i], u2lVelArr[i], scale[i] );
            err = trjScurveObj.NextSegment( nextCommandedPos, nextMaxVelocity, timeArr[i] );
            if (err) return err;
        }

        //Intialize the value of min
        int min = 260;

        // Iterate the array
        for (int i = 0; i < axisCt; i++)
        {
            if (timeArr[i] < min)
            {
                min = timeArr[i];
                index = i;
            }
        }
        delete[] timeArr;
        trjScurveObj.SetAdvanceMove(true);
    }
    
   trjScurveObj.SetUserUnitsAndScale(u2lPosArr[index], u2lVelArr[index], scale[index]);

   // get the next commanded position and maximum velocity values.
   err = trjScurveObj.NextSegment( nextCommandedPos, nextMaxVelocity, time );
   if( err ) return err;

   for( int i=0; i<axisCt; i++ )
   {
       pos[i] = (uunit)(startingPosLnkTrjScurve[i] + nextCommandedPos * scale[i]);
       vel[i] = (uunit)(nextMaxVelocity * scale[i]);
   }
   return 0;
}

/***************************************************************************/
/**
Finish this trajectory. 
*/
/***************************************************************************/
void LinkTrjScurve::Finish( void )
{
   trjScurveObj.Finish();
}

/***************************************************************************/
/**
Update the position wrap values for each axis in the linkage. The values in
the positionWrapArrIn array should be the values stored in the position wrap
object (firmware units). 
*/
/***************************************************************************/
const Error *LinkTrjScurve::SetPositionWrapArrUserUnits(uunit positionWrapArrIn[])
{
#ifdef CML_ENABLE_USER_UNITS

    const Error* err = 0;

    for (int i = 0; i < startingPosLnkTrjScurve.getDim(); i++) {
    
        if (positionWrapArrIn[i] < 0) {
            return &ScurveError::BadParam;
        }

        posWrapUserUnitsArr[i] = positionWrapArrIn[i];
        positionWrapLoadUnitsArr[i] = PosUser2Load(posWrapUserUnitsArr[i], i);

        if (positionWrapLoadUnitsArr[i] == 0) {
            usePositionWrapArr[i] = false;
        }
        else {
            usePositionWrapArr[i] = true;
        }
    }
    return err;
#endif
}

/***************************************************************************/
/**
Convert a position from user units to internal amplifier units.

@param pos The position in user units
@return The position in encoder counts
*/
/***************************************************************************/
int32 LinkTrjScurve::PosUser2Load(uunit pos, int axis)
{
#ifdef CML_ENABLE_USER_UNITS
    pos *= u2lPosArr[axis];
    return (int32)Round(pos);
#else
    return pos;
#endif
}

/***************************************************************************/
/**
Update the load to user unit converters of the amplifiers in the linkage trajectory. 
The user units need to be applied to the pout and vout variables in the 
LinkTrjScurve::NextSegment method, transforming them from load to user units.
*/
/***************************************************************************/
void LinkTrjScurve::UpdateUserToLoadUnitConverters(uunit u2lPosArrIn[], uunit u2lVelArrIn[])
{
#ifdef CML_ENABLE_USER_UNITS
    for (int i = 0; i < startingPosLnkTrjScurve.getDim(); i++) {
        u2lPosArr[i] = u2lPosArrIn[i];
        u2lVelArr[i] = u2lVelArrIn[i];
        l2uPosArr[i] = 1.0 / u2lPosArr[i];
        l2uVelArr[i] = 1.0 / u2lVelArr[i];
    }
#endif
}

/***************************************************************************/
/**
Update the private data member variable isRelative.
*/
/***************************************************************************/
void LinkTrjScurve::SetIsRelative(bool isRelIn) {
    isRelative = isRelIn;
}

#endif
