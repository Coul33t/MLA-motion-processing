/**
MotionOperation.h

@author Quentin Couland
@version 0.1

*/

#ifndef __MLA_MOTIONOPERATION_H__
#define __MLA_MOTIONOPERATION_H__

#include "MLAMotion\MLAMotion.h"

class MotionOperation {

public:
	/** Interpolate a joint from 2 joints and a mix factor.
			
			@param Joint: the first joint
			@param Joint: the second joint
			@param double: the mix factor

			@return Joint: the interpolated joint
	*/
	Joint* interpolateJoint(Joint*, Joint*, double);

	/** Interpolate a frame from 2 frame and a mix factor.

			@param Frame: the first frame
			@param Frame: the second frame
			@param double: the mix factor

			@return Frame: the interpolated frame
	*/
	Frame* interpolateFrame(Frame*, Frame*, double);

	/** Compute the speed of the joints between 2 frames.

			@param Frame: the first frame
			@param Frame: the second frame
			@param double: the interframe time

			@return map<string, double>: the speed of joints
	*/
	std::map<std::string, double> jointsSpeed(Frame*, Frame*, double);

	/** Recursively transforms initial joints (and its childs) local coordinates to global coordinates.

			@param Joint: the initial joint
			@param map<string, dvec3>: the global coordinates map
			@param map<string, dmat4>: the matrix used to compute global coordinates
	*/
	void getGlobalCoordinates(Joint*, std::map<std::string, glm::dvec3>&, std::map<std::string, glm::dmat4>&);

	/** Filter a motion, by eliminating position variations.

			@param Motion: the motion to be filtered
	*/
	void motionFiltering(Motion*);
};

#endif //__MLA_MOTIONOPERATION_H__
