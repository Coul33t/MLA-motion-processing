/**
MotionOperation.h

@author Quentin Couland
@version 0.1

*/

#ifndef __MLA_MOTIONOPERATION_H__
#define __MLA_MOTIONOPERATION_H__

#include "MLAMotion\MLAMotion.h"
#include <numeric> // std::accumulate

// Define an EPSILON value to compare to.
// This is because floating point operations
// are not accurate.
#define EPSILON 0.00001

namespace motionoperation{

	Joint* interpolateJoint(Joint*, Joint*, double);

	Frame* interpolateFrame(Frame*, Frame*, double);

	std::map<std::string, double> jointsLinearSpeed(Frame*, Frame*, double);

	std::map<std::string, double> jointsAngularSpeed(Frame*, Frame*, double);

	std::map<std::string, double> MeanLinearSpeed(std::vector<Frame*>, double);

	std::vector<std::map<std::string, double>> MeanLinearSpeedInterval(Motion*, unsigned int);

	std::vector<std::map<std::string, double>> MeanLinearSpeedIntervalFrame(Motion*, unsigned int);

	Frame* getFrameFromTime(Motion*, double, double);

	void getGlobalCoordinates(Joint*, std::map<std::string, glm::dvec3>&, std::map<std::string, glm::dmat4>&);

	void motionFiltering(Motion*);

	double vectorLength(glm::dvec3, glm::dvec3);

	double roundToDecimal(double, unsigned int);
};

#endif //__MLA_MOTIONOPERATION_H__
