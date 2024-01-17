/*  Copley Motion Libraries                             */
/*                                                      */
/*  Copyright (c) 2021 Copley Controls Corp.            */
/*                     http://www.copleycontrols.com    */
/*                                                      */
/********************************************************/

/** \file

This header file defines the classes that define the
PvtConstAccelTrj class. The class is derived from the Linkage
Trajectory class and is used to calculate a set of
velocities that produce constant acceleration from a given
set of positions and time constants.

*/

#ifndef _DEF_INC_PVTCONSTACCELTRJ
#define _DEF_INC_PVTCONSTACCELTRJ

#include <vector>
#include <list>
#include "CML.h"

using std::list;
using std::vector;

CML_NAMESPACE_START()

/*
This class represents node errors.
There is one static member for each defined node error.
*/
class PvtConstAccelTrjError : public Error
{
public:
	/// The user input an empty multidimensional position vector.
	static const PvtConstAccelTrjError PositionVectorInvalidSize;

	/// The user input an empty multidimensional time vector.
	static const PvtConstAccelTrjError InvalidDimension;

protected:
	/// Standard protected constructor
	PvtConstAccelTrjError(uint16 id, const char* desc) : Error(id, desc) {}
};

class PvtConstAccelTrj : public LinkTrajectory {
public:

	// default constructor	
	PvtConstAccelTrj();		           

	// initialization method. Sets the number of dimensions in the trajectory.
	const Error* Init(int inputNumberOfDimensions);
		
	// add a PVT point to the trajectory
	const Error* addPvtPoint(vector<double>* inputPositions, uint8* inputTime);
	
	// return a pointer to the multidimensional velocity vector.
	vector<list<double> >* getVelocitiesPntr();

	// return a pointer to the multidimensional position vector.
	vector<list<double> >* getPositionsPntr();

	// return a pointer to the vector storing the time data.
	list<uint8>* getTimePntr();

	// virtual destructor
	virtual ~PvtConstAccelTrj(); 

	// retrieve the dimension of the linkage move (number of axis)
	virtual int GetDim(void);

	// Retrieve the number of points to maintain in the PVT buffer. 
	// For real-time applications, set this value to a low number.
	// It must be at least 2.
	virtual int MaximumBufferPointsToUse(void) { return maxBufferPoints; }

	// clear the points in the multidimensional PVT vector
	void clearPvtPoints();

	// return number of PVT points
	int getNumberOfPvtPoints();

	// in the event of an error in which the PVT move did not successfully finish
	// and the user has deletePointsAfterExecution == false, call this method
	// to reset the iterators to the linked lists storing the PVT points.
	void resetTrajectory();

	// delete PVT points after they've been executed
	bool deletePointsAfterExecution = true;

	// end the move gracefully by setting the next time segment to 0 if the PVT buffer only
	// has one point remaining. 
	bool endMoveIfEmptyBuffer = true;

	// the number of points to maintain in the PVT buffer. Must be at least 2.
	// Set to a low value for real-time applications.
	int maxBufferPoints = 10000;

private:

	// ------------ PRIVATE DATA MEMBER FUNCTIONS ---------------------- //

	// returns a list of velocities producing constant acceleration
	void calculateVelocities();

	// defines the next segment to load to the trajectory generator
	virtual const Error* NextSegment(uunit pos[], uunit vel[], uint8& time);

	// ------------ PRIVATE DATA MEMBER VARIABLES ---------------------- //

	// The time constant between user defined PVT points (milliseconds).
	// Set to zero to end the move.
	list<uint8> timesList;

	// positions multidimensional vector
	vector<list<double> > posMultiAxisVector;

	// velocities multidimensional vector
	vector<list<double> > velMultiAxisVector;

	// number of axis in move
	int dimension;

	// last commanded PVT point (only used if deletePointsAfterExecution == false)
	vector<list<double>::iterator> lastCommandedPositionVector;
	vector<list<double>::iterator> lastCommandedVelocityVector;
	list<uint8>::iterator lastCommandedTime;
};

CML_NAMESPACE_END()

#endif

