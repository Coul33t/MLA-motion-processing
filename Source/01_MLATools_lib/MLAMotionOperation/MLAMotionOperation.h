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

#define EPSILON 0.0001 // Floating point precision

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

		std::shared_ptr<Joint> interpolateJoint(std::shared_ptr<Joint>, std::shared_ptr<Joint>, double);
		std::shared_ptr<Frame> interpolateFrame(std::shared_ptr<Frame>, std::shared_ptr<Frame>, double);

		void jointsPositions(std::map<std::string, glm::dvec3>&, std::shared_ptr<Frame>, const std::string&);
		void jointsPositions(std::map<std::string, glm::dvec3>&, std::shared_ptr<Frame>, std::vector<std::string>&);
		void jointsPositionsAxis(std::map<std::string, double>&, std::shared_ptr<Frame>, const std::string&, 
			const std::string&);

		void jointsLinearSpeed(std::map<std::string, double>&, std::shared_ptr<Frame>, std::shared_ptr<Frame>, double);
		void jointsLinearSpeed(std::map<std::string, glm::dvec3>&, std::shared_ptr<Frame>, std::shared_ptr<Frame>, 
			double, bool = false);
		void jointsLinearSpeedAxis(std::map<std::string, double>&, std::shared_ptr<Frame>, std::shared_ptr<Frame>, 
			double, const std::string&, bool = false);
		void jointsAngularSpeed(std::map<std::string, double>&, std::shared_ptr<Frame>, std::shared_ptr<Frame>, double);

		void jointsLinearAcc(std::map<std::string, double>&, std::shared_ptr<Frame>, std::shared_ptr<Frame>, 
			std::shared_ptr<Frame>, double);
		void jointsLinearAcc(std::map<std::string, glm::dvec3>&, std::shared_ptr<Frame>, std::shared_ptr<Frame>, 
			std::shared_ptr<Frame>, double, bool = false);
		void jointLinearAccAxis(std::map<std::string, double>&, std::shared_ptr<Frame>, std::shared_ptr<Frame>,
			std::shared_ptr<Frame>, double, const std::string&, bool = false);

		void jointsJerk(std::map<std::string, glm::dvec3>&, std::shared_ptr<Frame>, std::shared_ptr<Frame>, 
			std::shared_ptr<Frame>, std::shared_ptr<Frame>, double);
		void jointsJerkNorm(std::map<std::string, double>&, std::shared_ptr<Frame>, std::shared_ptr<Frame>,
			std::shared_ptr<Frame>, std::shared_ptr<Frame>, double);
		void jointsJerkAxis(std::map<std::string, double>&, std::shared_ptr<Frame>, std::shared_ptr<Frame>,
			std::shared_ptr<Frame>, std::shared_ptr<Frame>, double, const std::string&, bool = false);

		void jointsBoundingBox(std::map<std::string, std::vector<double>>&, std::shared_ptr<Frame>, std::vector<std::string>&,
			bool = true);
		void jointsBoundingBoxReframed(std::vector<double>&, std::shared_ptr<Frame>, std::vector<std::string>&, 
			unsigned int, unsigned int, bool = true);

		void MeanLinearSpeed(std::vector<std::map<std::string, double>>&, std::shared_ptr<Motion>, unsigned int);
		void MeanLinearSpeedAxis(std::vector<std::map<std::string, double>>&, std::shared_ptr<Motion>, unsigned int, 
			const std::string&, bool);

		void jointsDistance(std::map<std::string, double>&, std::shared_ptr<Frame>, std::vector<std::string>&,
			const std::vector<std::pair<std::string, std::string>>&, bool = true);
		void jointsDistanceAxis(std::map<std::string, std::vector<double>>&, std::shared_ptr<Frame>,
			std::vector<std::string>&, const std::vector<std::pair<std::string, std::string>>&, unsigned int, bool = true);
		
		std::shared_ptr<Frame> getFrameFromTime(std::shared_ptr<Motion>, double, double);

		void getGlobalCoordinates(std::shared_ptr<Frame>, std::shared_ptr<Frame>, std::shared_ptr<Joint>, glm::dmat4);

		void getGlobalMaximum(std::vector<double>&, std::pair<double, unsigned int>&);
		int getLocalMinimumFromMaximum(std::vector<double>&, int);
		int getLocalMaximum(std::vector<double>&, int);
		int getLocalMinimum(std::vector<double>&, int);

		void FindIndexSeparation(std::vector<double>&, unsigned int, unsigned int, std::vector<std::pair<int, int>>&);
		void FindThrowIndex(std::vector<double>&, std::pair<int, int>&);

		void MotionSegmentation(std::shared_ptr<Motion>, SegmentationInformation&, std::vector<std::shared_ptr<Motion>>&,
			const std::string&);
		void MotionThrowSegmentation(std::shared_ptr<Motion>, SegmentationInformation&, std::shared_ptr<Motion>,
			const std::string&);

		void getBegMaxEndIndexes(std::shared_ptr<Motion>, SegmentationInformation&, const std::string&, std::vector<int>&);
		void BegMaxEndAcceleration(std::shared_ptr<Motion>, SegmentationInformation&, const std::string&,
			std::vector<std::map<std::string, glm::dvec3>>&);
		void BegMaxEndSpeed(std::shared_ptr<Motion>, SegmentationInformation&, const std::string&,
			std::vector<std::map<std::string, glm::dvec3>>&);
		void BegMaxEndSpeedThrow(std::shared_ptr<Motion>, SegmentationInformation&, const std::string&,
			std::vector<std::map<std::string, glm::dvec3>>&, bool);
		void ThrowDuration(std::shared_ptr<Motion>, SegmentationInformation&, const std::string&);

		void motionSpeedComputing(std::shared_ptr<Motion>, SpeedData&);
		void motionAccelerationComputing(std::shared_ptr<Motion>, AccData&);
		
		void ComputeSpeedData(std::vector<std::shared_ptr<Motion>>&, std::vector<SpeedData>&);
		void ComputeAccData(std::vector<std::shared_ptr<Motion>>&, std::vector<AccData>&);
		void computeJerk(std::shared_ptr<Motion>, std::vector<std::map<std::string, double>>&, std::string&, bool);
		void computeBoundingBoxes(std::shared_ptr<Motion>, std::vector<std::map<std::string, std::vector<double>>>&,
			std::vector<std::string>&);
		void computeFinalBoudingBox(std::shared_ptr<Motion>, std::vector<std::map<std::string, std::vector<double>>>&,
			std::vector<std::string>&, bool = true);
		void computeBoundingBoxWidth(std::shared_ptr<Motion>, std::map<std::string, double>&, std::vector<std::string>&);


		void computeDistancesBeforeThrow(std::shared_ptr<Motion>, SegmentationInformation&, const std::string&,
			std::vector<std::map<std::string, double>>&, const std::vector<std::pair<std::string, std::string>>&,
			std::vector<std::string>&, bool = true);
		void computeDistancesAxisBeforeThrow(std::shared_ptr<Motion>, SegmentationInformation&, const std::string&,
			std::vector<std::map<std::string, std::vector<double>>>&, const std::vector<std::pair<std::string, std::string>>&,
			std::vector<std::string>&, bool = true);
		void computePositionBeforeThrow(std::shared_ptr<Motion>, SegmentationInformation&, const std::string&,
			std::vector<std::map<std::string, glm::dvec3>>&, const std::string&);
		void computePositionBeforeThrow(std::shared_ptr<Motion>, SegmentationInformation&, const std::string&,
			std::vector<std::map<std::string, glm::dvec3>>&, std::vector<std::string>&);

		void ComputeSavgol(SpeedData&, SegmentationInformation&);
		void ComputeSavgol(AccData&, SegmentationInformation&);

		void motionRebuilding(std::shared_ptr<Motion>, std::shared_ptr<Motion>, unsigned int);

		void motionFiltering(std::shared_ptr<Motion>);

	};
};
#endif //__MLA_MOTIONOPERATION_H__
