/**
* \file MlaMotionOperation.h
* \brief This header file contains all the required declarations for the motion operations.
* \author Quentin.C
* \version 0.1
* \date 12 February 2019
*/

#ifndef __MLA_MOTIONOPERATION_H__ 
#define __MLA_MOTIONOPERATION_H__

#include "MLAMotion/MLAMotion.h"
#include "MLAUtility/MLAUtility.h"
#include "MLAFilters/MLASavgol.h"
#include "MLAMotionOperation/MLASpeedData.h"
#include "MLAMotionOperation/MLAAccData.h"
#include <numeric> // std::accumulate
#include <cmath> // std::round

#define EPSILON 0.0001

struct SegmentationInformation {
	int left_cut;
	int right_cut;
	int savgol_window_size;
	int savgol_polynom_order;
	int final_frame_number;
	double final_interframe_time;
	std::pair<int, int> throw_idx;
};

namespace Mla {
	namespace MotionOperation {

		Joint* interpolateJoint(Joint*, Joint*, double);
		Frame* interpolateFrame(Frame*, Frame*, double);

		void jointsLinearSpeed(std::map<std::string, double>&, Frame*, Frame*, double);
		void jointsLinearSpeed(std::map<std::string, glm::dvec3>&, Frame*, Frame*, double, bool = false);
		void jointsLinearSpeedAxis(std::map<std::string, double>&, Frame*, Frame*, double, const std::string&, bool = false);
		void jointsAngularSpeed(std::map<std::string, double>&, Frame*, Frame*, double);

		void jointsLinearAcc(std::map<std::string, double>&, Frame*, Frame*, Frame*, double);
		void jointsLinearAcc(std::map<std::string, glm::dvec3>&, Frame*, Frame*, Frame*, double, bool = false);
		void jointLinearAccAxis(std::map<std::string, double>&, Frame*, Frame*, Frame*, double, const std::string&, bool = false);

		void jointsJerk(std::map<std::string, glm::dvec3>&, Frame*, Frame*, Frame*, Frame*, double);
		void jointsJerkNorm(std::map<std::string, double>&, Frame*, Frame*, Frame*, Frame*, double);
		void jointsJerkAxis(std::map<std::string, double>&, Frame*, Frame*, Frame*, Frame*, double, const std::string&, bool = false);

		void jointsBoundingBox(std::map<std::string, std::vector<double>>&, Frame*, std::vector<std::string>&);

		void MeanLinearSpeed(std::vector<std::map<std::string, double>>&, Motion*, unsigned int);
		void MeanLinearSpeedAxis(std::vector<std::map<std::string, double>>&, Motion*, unsigned int, const std::string&, bool);
		Frame* getFrameFromTime(Motion*, double, double);

		void getGlobalCoordinates(Frame*, Frame*, Joint*, glm::dmat4);

		void getGlobalMaximum(std::vector<double>&, std::pair<double, unsigned int>&);
		int getLocalMinimumFromMaximum(std::vector<double>&, int);
		int getLocalMaximum(std::vector<double>&, int);
		int getLocalMinimum(std::vector<double>&, int);

		void FindIndexSeparation(std::vector<double>&, unsigned int, unsigned int, std::vector<std::pair<int, int>>&);
		void FindThrowIndex(std::vector<double>&, std::pair<int, int>&);

		void MotionSegmentation(Motion*, SegmentationInformation&, std::vector<Motion*>&, const std::string&);
		void MotionThrowSegmentation(Motion*, SegmentationInformation&, Motion*, const std::string&);

		void getBegMaxEndIndexes(Motion*, SegmentationInformation&, const std::string&, std::vector<int>&);
		void BegMaxEndAcceleration(Motion*, SegmentationInformation&, const std::string&, std::vector<std::map<std::string, glm::dvec3>>&);
		void BegMaxEndSpeed(Motion*, SegmentationInformation&, const std::string&, std::vector<std::map<std::string, glm::dvec3>>&);
		void BegMaxEndSpeedThrow(Motion*, SegmentationInformation&, const std::string&, std::vector<std::map<std::string, glm::dvec3>>&, bool);
		void ThrowDuration(Motion*, SegmentationInformation&, const std::string&);

		void motionSpeedComputing(Motion*, SpeedData&);
		void motionAccelerationComputing(Motion*, AccData&);
		
		void ComputeSpeedData(std::vector<Motion*>&, std::vector<SpeedData>&);
		void ComputeAccData(std::vector<Motion*>&, std::vector<AccData>&);
		void computeJerk(Motion*, std::vector<std::map<std::string, double>>&, std::string&, bool);
		void computeBoundingBoxes(Motion*, std::vector<std::map<std::string, std::vector<double>>>&, std::vector<std::string>&);
		void computeFinalBoudingBox(Motion*, std::vector<std::map<std::string, std::vector<double>>>&, std::vector<std::string>&);

		void ComputeSavgol(SpeedData&, SegmentationInformation&);
		void ComputeSavgol(AccData&, SegmentationInformation&);

		void motionRebuilding(Motion*, Motion*, unsigned int);

		void motionFiltering(Motion*);

	};
};
#endif //__MLA_MOTIONOPERATION_H__
