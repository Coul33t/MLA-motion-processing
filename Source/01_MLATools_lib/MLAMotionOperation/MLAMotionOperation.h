/**
MotionOperation.h

@author Quentin Couland
@version 0.1

*/

#ifndef __MLA_MOTIONOPERATION_H__ 
#define __MLA_MOTIONOPERATION_H__

#include "MLAMotion/MLAMotion.h"
#include "MLAUtility/MLAUtility.h"
#include "MLAFilters/MLASavgol.h"
#include "MLAMotionOperation/MLASpeedData.h"
#include <numeric> // std::accumulate

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
		void jointsLinearSpeed(std::map<std::string, glm::dvec3>&, Frame*, Frame*, double);
		void jointsLinearSpeedAxis(std::map<std::string, double>&, Frame*, Frame*, double, const std::string&, bool);
		void jointsAngularSpeed(std::map<std::string, double>&, Frame*, Frame*, double);

		void MeanLinearSpeed(std::vector<std::map<std::string, double>>&, Motion*, unsigned int);
		void MeanLinearSpeedAxis(std::vector<std::map<std::string, double>>&, Motion*, unsigned int, const std::string&, bool);
		Frame* getFrameFromTime(Motion*, double, double);
		Frame* getFrameFromIndex(Motion*, unsigned int, double);

		void getGlobalCoordinates(Frame*, Frame*, Joint*, glm::dmat4);

		int getLocalMinimumFromMaximum(std::vector<double>&, int);
		int getLocalMaximum(std::vector<double>&, int);
		int getLocalMinimum(std::vector<double>&, int);

		void FindIndexSeparation(std::vector<double>&, unsigned int, unsigned int, std::vector<std::pair<int, int>>&);

		void MotionSegmentation(Motion*, SegmentationInformation&, std::vector<Motion*>&, const std::string&);

		void getBegEndIndexes(Motion*, SegmentationInformation&, const std::string&, std::vector<int>&);
		void BegMaxEndAcceleration(Motion*, SegmentationInformation&, const std::string&, std::vector<std::map<std::string, glm::dvec3>>&);
		void BegMaxEndSpeed(Motion*, SegmentationInformation&, const std::string&, std::vector<std::map<std::string, glm::dvec3>>&);
		void ThrowDuration(Motion*, SegmentationInformation&, const std::string&);

		void motionSpeedComputing(Motion*, SpeedData&);

		void ComputeSpeedData(std::vector<Motion*>&, std::vector<SpeedData>&);
		void motionAccelerationComputing(SpeedData&, std::vector<std::map<std::string, glm::dvec3>>&, bool);
		void ComputeSavgol(SpeedData&, SegmentationInformation&);

		void motionRebuilding(Motion*, Motion*, unsigned int);

		void motionFiltering(Motion*);

	};
};
#endif //__MLA_MOTIONOPERATION_H__
