/********************************************************/
/*                                                      */
/*  Copley Motion Libraries                             */
/*                                                      */
/*  Copyright (c) 2022 Copley Controls Corp.            */
/*                     http://www.copleycontrols.com    */
/*                                                      */
/********************************************************/

/** \file
This file holds code to implement the PvtTrj class.
*/
#include "CML.h"
#include "CML_PvtTrj.h"
#include <iostream>

CML_NAMESPACE_START()

/**************************************************
* PvtTrjError Error objects
**************************************************/
CML_NEW_ERROR(PvtTrjError, PositionVectorInvalidSize,   "The size of the position vector used in PvtTrj::addPvtPoint is invalid. Please use a position vector with size equal to the number of dimensions in the PvtTrj object. If the object has not been initialized, please use the Init method to set the number of dimensions in the trajectory.");
CML_NEW_ERROR(PvtTrjError, VelocityVectorInvalidSize,   "The size of the velocity vector used in PvtTrj::addPvtPoint is invalid. Please use a velocity vector with size equal to the number of dimensions in the PvtTrj object. If the object has not been initialized, please use the Init method to set the number of dimensions in the trajectory.");
CML_NEW_ERROR(PvtTrjError, InvalidDimension,			"The input dimension used in PvtTrj::Init is invalid. Please enter a dimension that is greater than 0 and less than 33.");

// default constructor
PvtTrj::PvtTrj()
	:dimension{ 0 }
{}

// set the number of dimensions in the PVT move.
const Error* PvtTrj::Init(int inputDimension) {

	const Error* err{ NULL }; // pointer to Error object

	// if the user entered an invalid number of axes, return an error.
	if ((inputDimension < 1) || (inputDimension > CML_MAX_AMPS_PER_LINK)) {
		err = &PvtTrjError::InvalidDimension;
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
const Error* PvtTrj::addPvtPoint(vector<double>* positionVector, vector<double>* velocityVector, uint8* timeValue) {
	
	const Error* err{ NULL }; // pointer to Error object

	// if the size of the position vector does not match the dimension of the trajectory, 
	// return an error code here.
	if ((int)positionVector->size() != dimension) {
		// return error code here
		// return a helpful message to the user asking them to use the Init method.
		err = &PvtTrjError::PositionVectorInvalidSize;
		return(err);
	}

	// if the size of the velocity vector does not match the dimension of the trajectory, 
	// return an error code here.
	if ((int)velocityVector->size() != dimension) {
		// return error code here
		// return a helpful message to the user asking them to use the Init method.
		err = &PvtTrjError::VelocityVectorInvalidSize;
		return(err);
	}

	// push the positions and velocities onto the position vector
	for (size_t i = 0; i < positionVector->size(); i++) {
		posMultiAxisVector[i].push_back((*positionVector)[i]);
		velMultiAxisVector[i].push_back((*velocityVector)[i]);
	}

	// push the time value onto the time vector
	timesList.push_back(*timeValue);

	// if adding the first PVT point, assign the iterators to the first element in the linked lists storing the PVT data
	if (getNumberOfPvtPoints() == 1) {
		resetTrajectory();
	}

	return 0;
}

// Deconstructor
PvtTrj::~PvtTrj() {
	velMultiAxisVector.clear(); // remove all elements from vector
	posMultiAxisVector.clear();
	timesList.clear();
	lastCommandedPositionVector.clear();
	lastCommandedVelocityVector.clear();
}

// retrieve the dimension of the Pvt trajectory
int PvtTrj::GetDim(void) {
	return dimension;
}

// return a pointer to the velocities vector
vector<list<double>>* PvtTrj::getVelocitiesPntr() {
	vector<list<double>>* pntr{ &velMultiAxisVector };
	return(pntr);
}

// return a pointer to the positions vector
vector<list<double>>* PvtTrj::getPositionsPntr() {
	vector<list<double>>* pntr{ &posMultiAxisVector };
	return(pntr);
}

// return a pointer to the time vector
list<uint8>* PvtTrj::getTimePntr() {
	list<uint8>* pntr{ &timesList };
	return(pntr);
}

// Define the next PVT segment to send to the CML buffer.
const Error* PvtTrj::NextSegment(uunit pos[], uunit vel[], uint8& time){

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

// Clear the points in the multidimensional vectors.
void PvtTrj::clearPvtPoints() {

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
void PvtTrj::resetTrajectory() {

	for (int i = 0; i < dimension; i++) {
		lastCommandedPositionVector[i] = posMultiAxisVector[i].begin();
		lastCommandedVelocityVector[i] = velMultiAxisVector[i].begin();
	}

	lastCommandedTime = timesList.begin();
}

// return the number of PVT points in the profile.
int PvtTrj::getNumberOfPvtPoints() {
	return(timesList.size());
}

CML_NAMESPACE_END()