/**
MotionOperation.h

@author Quentin Couland
@version 0.1

*/

#ifndef __MLA_MOTIONOPERATION_H__
#define __MLA_MOTIONOPERATION_H__

#include "MLAMotion/MLAMotion.h"
#include "MLAUtility/MLAUtility.h"
#include <numeric> // std::accumulate
#include <limits>

// Define an EPSILON value to compare to.
// This is because floating point operations
// are not accurate.
#define EPSILON 0.00001

namespace mla {
	namespace motionoperation{

		Joint* interpolateJoint(Joint*, Joint*, double);
		Frame* interpolateFrame(Frame*, Frame*, double);

		std::map<std::string, double> jointsLinearSpeed(Frame*, Frame*, double);
		std::map<std::string, double> jointsAngularSpeed(Frame*, Frame*, double);

		std::vector<std::map<std::string, double>> MeanLinearSpeed(Motion*, unsigned int);
		Frame* getFrameFromTime(Motion*, double, double);

		void getGlobalCoordinates(Joint*, std::map<std::string, glm::dvec3>&, std::map<std::string, glm::dmat4>&);

		double getLocalMinimumFromMaximum(std::vector<double>, double, int);

		void motionFiltering(Motion*);

	};
};
#endif //__MLA_MOTIONOPERATION_H__
