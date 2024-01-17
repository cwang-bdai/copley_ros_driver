/********************************************************/
/*                                                      */
/*  Copley Motion Libraries                             */
/*                                                      */
/*  Copyright (c) 2021 Copley Controls Corp.            */
/*                     http://www.copleycontrols.com    */
/*                                                      */
/********************************************************/

/** \file
This file holds code to implement the PvtConstAccelTrj class.
*/
#include "CML.h"
#include "CML_PvtConstAccelTrj.h"
#include <iostream>

CML_NAMESPACE_START()

/**************************************************
* PvtConstAccelError Error objects
**************************************************/
CML_NEW_ERROR(PvtConstAccelTrjError, PositionVectorInvalidSize,   "The size of the position vector used in PvtConstAccelTrj::addPvtPoint is invalid. Please use a position vector with size equal to the number of dimensions in the PvtConstAccelTrj object. If the object has not been initialized, please use the Init method to set the number of dimensions in the trajectory.");
CML_NEW_ERROR(PvtConstAccelTrjError, InvalidDimension,			  "The input dimension used in PvtConstAccelTrj::Init is invalid. Please enter a dimension that is greater than 0 and less than 33.");

// default constructor
PvtConstAccelTrj::PvtConstAccelTrj()
	:dimension{ 0 }
{}

// set the number of dimensions in the PVT move.
const Error* PvtConstAccelTrj::Init(int inputDimension) {

	const Error* err{ NULL }; // pointer to Error object

	// if the user entered an invalid number of axes, return an error.
	if ((inputDimension < 1) || (inputDimension > CML_MAX_AMPS_PER_LINK)) {
		err = &PvtConstAccelTrjError::InvalidDimension;
		return(err);
	}

	// assign the user's value to the dimension (private data member)
	dimension = inputDimension;

	// initialize the multiaxis vectors
	list<double> temp;
	for (int i = 0; i < dimension; i++) {
		posMultiAxisVector.push_back(temp);
		velMultiAxisVector.push_back(temp);
	}

	// initialize the last PVT point pointers
	list<double>::iterator tempIterator;
	for (int i = 0; i < dimension; i++) {
		lastCommandedPositionVector.push_back(tempIterator);
		lastCommandedVelocityVector.push_back(tempIterator);
	}

	return 0;
}

// add a point to the trajectory
const Error* PvtConstAccelTrj::addPvtPoint(vector<double>* positionVector, uint8* timeValue) {
	
	const Error* err{ NULL }; // pointer to Error object

	// if the size of the position vector does not match the dimension of the trajectory, 
	// return an error code here.
	if ((int)positionVector->size() != dimension) {
		// return error code here
		// return a helpful message to the user asking them to use the Init method.
		err = &PvtConstAccelTrjError::PositionVectorInvalidSize;
		return(err);
	}

	// push the positions onto the position vector
	for (size_t i = 0; i < positionVector->size(); i++) {
		posMultiAxisVector[i].push_back((*positionVector)[i]);

		// push 0.0 value onto the velocity vector
		velMultiAxisVector[i].push_back(0.0);
	}

	// push the time value onto the time vector
	timesList.push_back(*timeValue);

	// calculate the velocities
	calculateVelocities();

	// if adding the first PVT point, assign the iterators to the first element in the linked lists storing the PVT data
	if (getNumberOfPvtPoints() == 1) {
		resetTrajectory();
	}

	return 0;
}

// Deconstructor
PvtConstAccelTrj::~PvtConstAccelTrj() {
	velMultiAxisVector.clear(); // remove all elements from vector
	posMultiAxisVector.clear();
	timesList.clear();
	lastCommandedPositionVector.clear();
	lastCommandedVelocityVector.clear();
}

// retrieve the dimension of the Pvt trajectory
int PvtConstAccelTrj::GetDim(void) {
	return dimension;
}

// return a pointer to the velocities vector
vector<list<double>>* PvtConstAccelTrj::getVelocitiesPntr() {
	vector<list<double>>* pntr{ &velMultiAxisVector };
	return(pntr);
}

// return a pointer to the positions vector
vector<list<double>>* PvtConstAccelTrj::getPositionsPntr() {
	vector<list<double>>* pntr{ &posMultiAxisVector };
	return(pntr);
}

// return a pointer to the time vector
list<uint8>* PvtConstAccelTrj::getTimePntr() {
	list<uint8>* pntr{ &timesList };
	return(pntr);
}

// Define the next PVT segment to send to the CML buffer.
const Error* PvtConstAccelTrj::NextSegment(uunit pos[], uunit vel[], uint8& time){

	// load correct position and velocity data to each axis
	for (int i = 0; i < dimension; i++) {

		// load the last commanded positions and velocities
		pos[i] = *lastCommandedPositionVector[i];
		vel[i] = *lastCommandedVelocityVector[i];
	}
	
	// check if it is the last PVT point
	if (++lastCommandedTime != timesList.end()) {
		time = *--lastCommandedTime;

		// check if the points need to be deleted from the buffers
		if (deletePointsAfterExecution) {
			for (int k = 0; k < dimension; k++) {
				posMultiAxisVector[k].erase(lastCommandedPositionVector[k]++);
				velMultiAxisVector[k].erase(lastCommandedVelocityVector[k]++);
			}
			timesList.erase(lastCommandedTime++);
		}
		else {
			for (int k = 0; k < dimension; k++) {
				lastCommandedPositionVector[k]++;
				lastCommandedVelocityVector[k]++;
			}
			lastCommandedTime++;
		}
	}
	// Last PVT point!
	else {
		if (endMoveIfEmptyBuffer) {
			time = 0;
			if (!deletePointsAfterExecution) {
				resetTrajectory(); // reset the iterators to the beginning of the linked lists storing the PVT data
			}
			// delete the last PVT point
			else {
				for (int k = 0; k < dimension; k++) {
					posMultiAxisVector[k].erase(lastCommandedPositionVector[k]);
					velMultiAxisVector[k].erase(lastCommandedVelocityVector[k]);
				}
				timesList.erase(--lastCommandedTime);
			}
		}
		// Don't end the move.
		else {
			time = *--lastCommandedTime;
		}
	}

	return 0;
}

// Calculate the velocities using the constant acceleration algorithm.
void PvtConstAccelTrj::calculateVelocities() {

	// if there are only 3 points, return. 
	// There must be at least 3 points to perform some sort of calculation.
	if (timesList.size() < 3) {
		return;
	}
	
	// initialize local variable.
	double timeNow{ 0.0 };
	double timeNext{ 0.0 };
	double timePrev{ 0.0 };
	double lastPos{ 0.0 };
	double secLastPos{ 0.0 };
	double thirdLastPos{ 0.0 };
	double fourthLastPos{ 0.0 };
	double lastVel{ 0.0 };
	double thirdToLastVel{ 0.0 };
	double fourthToLastVel{ 0.0 };

	list<uint8>::iterator iterUint8;
	list<double>::iterator iterDouble;

	for (int i = 0; i < dimension; i++) {

		// load address of correct axis into axis pointer
		// posPntr = &(posMultiAxisVector[i]);

		// perform rough guess velocities calculation
		// get 2nd to last time and assign it to timeNow.
		iterUint8 = timesList.end();
		iterUint8--;
		iterUint8--;

		// second to last time
		timeNow = *iterUint8 / 1000.0;

		// assign position
		iterDouble = posMultiAxisVector[i].end();
		iterDouble--;
		
		// last position
		lastPos = *iterDouble;

		// third to last position
		iterDouble--;
		iterDouble--;
		thirdLastPos = *iterDouble;
		
		// address second to last velocity
		iterDouble = velMultiAxisVector[i].end();
		iterDouble--;
		iterDouble--;

		// overwrite the second to last velocity
		*iterDouble = (lastPos - thirdLastPos) / (2 * timeNow);

		// need at least 4 PVT point to perform the rest of the calculation
		if (timesList.size() < 4) {
			// do nothing
		}

		// handle case when there are 4 PVT points
		else if (timesList.size() == 4) {
			
			// refine velocity calculation			
			// record the second to last time value.
			timeNext = timeNow;

			// record the current time (third to last time)
			iterUint8--;
			timeNow = *iterUint8 / 1000.0;

			// record the previous time (fourth to last time)
			iterUint8--;
			timePrev = *iterUint8 / 1000.0;

			// last pos
			iterDouble = posMultiAxisVector[i].end();
			iterDouble--;
			lastPos = *iterDouble;

			// 2nd to last pos
			iterDouble--;
			secLastPos = *iterDouble;

			// 3rd to last pos
			iterDouble--;
			thirdLastPos = *iterDouble;

			// 4th to last pos
			iterDouble--;
			fourthLastPos = *iterDouble;

			iterDouble = velMultiAxisVector[i].end();

			// last vel
			iterDouble--;
			lastVel = *iterDouble;
			
			// 4th to last vel
			iterDouble--;
			iterDouble--;
			iterDouble--;
			fourthToLastVel = *iterDouble;

			// overwriting third to last vel
			iterDouble++;
			*iterDouble = (1 / ((4 * (timePrev + timeNow) * (timeNow + timeNext)) - (timePrev * timeNext))) * ((((6 * timePrev * (secLastPos - thirdLastPos) * (timeNow + timeNext)) - (3 * timePrev * timeNext * (secLastPos - thirdLastPos))) / timeNow) + ((6 * timeNow * (thirdLastPos - fourthLastPos) * (timeNow + timeNext)) / timePrev) - (2 * fourthToLastVel * timeNow * (timeNow + timeNext)) - ((3 * timePrev * timeNow * (lastPos - secLastPos)) / timeNext) + (lastVel * timePrev * timeNow));
			
			// adjust second the second to last velocity
			
			// third to last time is previous
			timePrev = timeNow;

			// second to last time is now
			timeNow = timeNext;

			// last time is next
			iterUint8 = timesList.end();
			iterUint8--;
			timeNext = *iterUint8 / 1000.0;
			
			// update copy of 3rd to last vel.
			thirdToLastVel = *iterDouble;

			// adjust second to last velocity
			iterDouble++;
			*iterDouble = (1 / (2 * (timePrev + timeNow))) * (((3 * timePrev * (lastPos - secLastPos)) / timeNow) + ((3 * timeNow * (secLastPos - thirdLastPos)) / timePrev) - (thirdToLastVel * timeNow) - (lastVel * timePrev));
		}

		// handle the case when there are 5 PVT points
		else if (timesList.size() == 5) {

			// refine velocity calculation by performing calculation three times.
			for (size_t j = 0; j < 3; j++) {
				
				////  1st iteration     //////////////////

				// copy the third, fourth, and fifth to last time values
				iterUint8 = timesList.end();
				iterUint8--;
				iterUint8--;
				iterUint8--;
				timeNext = *iterUint8 / 1000.0;
				iterUint8--;
				timeNow = *iterUint8 / 1000.0;
				iterUint8--;
				timePrev = *iterUint8 / 1000.0;

				// get the value of the 2nd to last position
				iterDouble = posMultiAxisVector[i].end();
				iterDouble--;
				iterDouble--;
				lastPos = *iterDouble;

				// get the value of the 3rd to last position
				iterDouble--;
				secLastPos = *iterDouble;

				// get the value of the fourth to last position
				iterDouble--;
				thirdLastPos = *iterDouble;

				// get the value of the fifth to last position
				iterDouble--;
				fourthLastPos = *iterDouble;

				// fifth to last vel, 2nd to last vel

				// get the 2nd to last velocity
				iterDouble = velMultiAxisVector[i].end();
				iterDouble--;
				iterDouble--;
				lastVel = *iterDouble;
				
				// get the fifth to last velocity
				iterDouble--;
				iterDouble--;
				iterDouble--;
				fourthToLastVel = *iterDouble;

				// set the fourth to last velocity
				iterDouble++;
				*iterDouble = (1 / ((4 * (timePrev + timeNow) * (timeNow + timeNext)) - (timePrev * timeNext))) * ((((6 * timePrev * (secLastPos - thirdLastPos) * (timeNow + timeNext)) - (3 * timePrev * timeNext * (secLastPos - thirdLastPos))) / timeNow) + ((6 * timeNow * (thirdLastPos - fourthLastPos) * (timeNow + timeNext)) / timePrev) - (2 * fourthToLastVel * timeNow * (timeNow + timeNext)) - ((3 * timePrev * timeNow * (lastPos - secLastPos)) / timeNext) + (lastVel * timePrev * timeNow));

				////// 2nd Iteration ////////////////

				// copy the 2nd, 3rd, and 4th to last time values
				iterUint8 = timesList.end();
				iterUint8--;
				iterUint8--;
				timeNext = *iterUint8 / 1000.0;
				iterUint8--;
				timeNow = *iterUint8 / 1000.0;
				iterUint8--;
				timePrev = *iterUint8 / 1000.0;

				// get the value of the last position
				iterDouble = posMultiAxisVector[i].end();
				iterDouble--;
				lastPos = *iterDouble;

				// get the value of the 2nd to last position
				iterDouble--;
				secLastPos = *iterDouble;

				// get the value of the 3rd to last position
				iterDouble--;
				thirdLastPos = *iterDouble;

				// get the value of the 4th to last position
				iterDouble--;
				fourthLastPos = *iterDouble;

				// fourth to last vel, last vel

				// get the value of the last velocity
				iterDouble = velMultiAxisVector[i].end();
				iterDouble--;
				lastVel = *iterDouble;

				// get the 4th to last velocity
				iterDouble--;
				iterDouble--;
				iterDouble--;
				fourthToLastVel = *iterDouble;

				// set the 3rd to last velocity
				iterDouble++;
				*iterDouble = (1 / ((4 * (timePrev + timeNow) * (timeNow + timeNext)) - (timePrev * timeNext))) * ((((6 * timePrev * (secLastPos - thirdLastPos) * (timeNow + timeNext)) - (3 * timePrev * timeNext * (secLastPos - thirdLastPos))) / timeNow) + ((6 * timeNow * (thirdLastPos - fourthLastPos) * (timeNow + timeNext)) / timePrev) - (2 * fourthToLastVel * timeNow * (timeNow + timeNext)) - ((3 * timePrev * timeNow * (lastPos - secLastPos)) / timeNext) + (lastVel * timePrev * timeNow));

				// get the last time (next)
				iterUint8 = timesList.end();
				iterUint8--;
				timeNext = *iterUint8 / 1000.0;
				
				// get the 2nd last time (now)
				iterUint8--;
				timeNow = *iterUint8 / 1000.0;

				// get the 3rd to last time (prev)
				iterUint8--;
				timePrev = *iterUint8 / 1000.0;

				// last pos, 2nd last pos, 3rd last pos, 3rd last vel, last vel
				iterDouble = velMultiAxisVector[i].end();
				iterDouble--;
				lastVel = *iterDouble;

				// get the 3rd to last velocity
				iterDouble--;
				iterDouble--;
				thirdToLastVel = *iterDouble;

				// adjust the 2nd to last velocity
				iterDouble++;
				*iterDouble = (1 / (2 * (timePrev + timeNow))) * (((3 * timePrev * (lastPos - secLastPos)) / timeNow) + ((3 * timeNow * (secLastPos - thirdLastPos)) / timePrev) - (thirdToLastVel * timeNow) - (lastVel * timePrev));
			}
		}

		else {
			// refine velocity calculation by performing calculation three times.
			for (size_t j = 0; j < 3; j++) {

				////  1st iteration     //////////////////

				// copy the fourth, fifth, and sixth to last time values
				iterUint8 = timesList.end();
				iterUint8--;
				iterUint8--;
				iterUint8--;
				iterUint8--;
				timeNext = *iterUint8 / 1000.0;
				iterUint8--;
				timeNow = *iterUint8 / 1000.0;
				iterUint8--;
				timePrev = *iterUint8 / 1000.0;

				// get the value of the 3rd to last position
				iterDouble = posMultiAxisVector[i].end();
				iterDouble--;
				iterDouble--;
				iterDouble--;
				lastPos = *iterDouble;

				// get the value of the 4th to last position
				iterDouble--;
				secLastPos = *iterDouble;

				// get the value of the 5th to last position
				iterDouble--;
				thirdLastPos = *iterDouble;

				// get the value of the 6th to last position
				iterDouble--;
				fourthLastPos = *iterDouble;

				// 6th to last vel, 3rd to last vel

				// get the 3rd to last velocity
				iterDouble = velMultiAxisVector[i].end();
				iterDouble--;
				iterDouble--;
				iterDouble--;
				lastVel = *iterDouble;

				// get the 6th to last velocity
				iterDouble--;
				iterDouble--;
				iterDouble--;
				fourthToLastVel = *iterDouble;

				// set the 5th to last velocity
				iterDouble++;
				*iterDouble = (1 / ((4 * (timePrev + timeNow) * (timeNow + timeNext)) - (timePrev * timeNext))) * ((((6 * timePrev * (secLastPos - thirdLastPos) * (timeNow + timeNext)) - (3 * timePrev * timeNext * (secLastPos - thirdLastPos))) / timeNow) + ((6 * timeNow * (thirdLastPos - fourthLastPos) * (timeNow + timeNext)) / timePrev) - (2 * fourthToLastVel * timeNow * (timeNow + timeNext)) - ((3 * timePrev * timeNow * (lastPos - secLastPos)) / timeNext) + (lastVel * timePrev * timeNow));

				////// 2nd Iteration ////////////////
				
				// copy the third, fourth, and fifth to last time values
				iterUint8 = timesList.end();
				iterUint8--;
				iterUint8--;
				iterUint8--;
				timeNext = *iterUint8 / 1000.0;
				iterUint8--;
				timeNow = *iterUint8 / 1000.0;
				iterUint8--;
				timePrev = *iterUint8 / 1000.0;

				// get the value of the 2nd to last position
				iterDouble = posMultiAxisVector[i].end();
				iterDouble--;
				iterDouble--;
				lastPos = *iterDouble;

				// get the value of the 3rd to last position
				iterDouble--;
				secLastPos = *iterDouble;

				// get the value of the fourth to last position
				iterDouble--;
				thirdLastPos = *iterDouble;

				// get the value of the fifth to last position
				iterDouble--;
				fourthLastPos = *iterDouble;

				// fifth to last vel, 2nd to last vel

				// get the 2nd to last velocity
				iterDouble = velMultiAxisVector[i].end();
				iterDouble--;
				iterDouble--;
				lastVel = *iterDouble;

				// get the fifth to last velocity
				iterDouble--;
				iterDouble--;
				iterDouble--;
				fourthToLastVel = *iterDouble;

				// set the fourth to last velocity
				iterDouble++;
				*iterDouble = (1 / ((4 * (timePrev + timeNow) * (timeNow + timeNext)) - (timePrev * timeNext))) * ((((6 * timePrev * (secLastPos - thirdLastPos) * (timeNow + timeNext)) - (3 * timePrev * timeNext * (secLastPos - thirdLastPos))) / timeNow) + ((6 * timeNow * (thirdLastPos - fourthLastPos) * (timeNow + timeNext)) / timePrev) - (2 * fourthToLastVel * timeNow * (timeNow + timeNext)) - ((3 * timePrev * timeNow * (lastPos - secLastPos)) / timeNext) + (lastVel * timePrev * timeNow));

				////// 3rd Iteration ////////////////

				// copy the 2nd, 3rd, and 4th to last time values
				iterUint8 = timesList.end();
				iterUint8--;
				iterUint8--;
				timeNext = *iterUint8 / 1000.0;
				iterUint8--;
				timeNow = *iterUint8 / 1000.0;
				iterUint8--;
				timePrev = *iterUint8 / 1000.0;

				// get the value of the last position
				iterDouble = posMultiAxisVector[i].end();
				iterDouble--;
				lastPos = *iterDouble;

				// get the value of the 2nd to last position
				iterDouble--;
				secLastPos = *iterDouble;

				// get the value of the 3rd to last position
				iterDouble--;
				thirdLastPos = *iterDouble;

				// get the value of the 4th to last position
				iterDouble--;
				fourthLastPos = *iterDouble;

				// fourth to last vel, last vel

				// get the value of the last velocity
				iterDouble = velMultiAxisVector[i].end();
				iterDouble--;
				lastVel = *iterDouble;

				// get the 4th to last velocity
				iterDouble--;
				iterDouble--;
				iterDouble--;
				fourthToLastVel = *iterDouble;

				// set the 3rd to last velocity
				iterDouble++;
				*iterDouble = (1 / ((4 * (timePrev + timeNow) * (timeNow + timeNext)) - (timePrev * timeNext))) * ((((6 * timePrev * (secLastPos - thirdLastPos) * (timeNow + timeNext)) - (3 * timePrev * timeNext * (secLastPos - thirdLastPos))) / timeNow) + ((6 * timeNow * (thirdLastPos - fourthLastPos) * (timeNow + timeNext)) / timePrev) - (2 * fourthToLastVel * timeNow * (timeNow + timeNext)) - ((3 * timePrev * timeNow * (lastPos - secLastPos)) / timeNext) + (lastVel * timePrev * timeNow));

				// get the last time (next)
				iterUint8 = timesList.end();
				iterUint8--;
				timeNext = *iterUint8 / 1000.0;

				// get the 2nd last time (now)
				iterUint8--;
				timeNow = *iterUint8 / 1000.0;

				// get the 3rd to last time (prev)
				iterUint8--;
				timePrev = *iterUint8 / 1000.0;

				// last pos, 2nd last pos, 3rd last pos, 3rd last vel, last vel
				iterDouble = velMultiAxisVector[i].end();
				iterDouble--;
				lastVel = *iterDouble;

				// get the 3rd to last velocity
				iterDouble--;
				iterDouble--;
				thirdToLastVel = *iterDouble;

				// adjust the 2nd to last velocity
				iterDouble++;
				*iterDouble = (1 / (2 * (timePrev + timeNow))) * (((3 * timePrev * (lastPos - secLastPos)) / timeNow) + ((3 * timeNow * (secLastPos - thirdLastPos)) / timePrev) - (thirdToLastVel * timeNow) - (lastVel * timePrev));
			}
		}
	}
}

// Clear the points in the multidimensional vectors.
void PvtConstAccelTrj::clearPvtPoints() {

	// clear the position data
	posMultiAxisVector.clear();
	lastCommandedPositionVector.clear();

	// clear the velocity data
	velMultiAxisVector.clear();
	lastCommandedVelocityVector.clear();

	// clear the time data
	timesList.clear();

	// if the user has already initialized the object with a valid number of axes,
	// initialize the object again.
	if ((dimension > 0) && (dimension <= CML_MAX_AMPS_PER_LINK)) {
		Init(dimension);
	}
}

// in the event of an error in which the PVT move did not successfully finish
// and the user has deletePointsAfterExecution == false, call this method
// to reset the iterators to the linked lists storing the PVT points.
void PvtConstAccelTrj::resetTrajectory() {

	for (int i = 0; i < dimension; i++) {
		lastCommandedPositionVector[i] = posMultiAxisVector[i].begin();
		lastCommandedVelocityVector[i] = velMultiAxisVector[i].begin();
	}

	lastCommandedTime = timesList.begin();
}

// return the number of PVT points in the profile.
int PvtConstAccelTrj::getNumberOfPvtPoints() {
	return(timesList.size());
}

CML_NAMESPACE_END()