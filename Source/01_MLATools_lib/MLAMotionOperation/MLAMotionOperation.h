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
#include <limits>

// Define an EPSILON value to compare to.
// This is because floating point operations
// are not accurate.
#define EPSILON 0.00001

struct SegmentationInformation {
	int left_cut;
	int right_cut;
	int savgol_window_size;
	int savgol_polynom_order;
	int final_frame_number;
	double final_interframe_time;
};

namespace Mla {
	namespace MotionOperation {

		Joint* interpolateJoint(Joint*, Joint*, double);
		Frame* interpolateFrame(Frame*, Frame*, double);

		void jointsLinearSpeed(std::map<std::string, double>&, Frame*, Frame*, double);
		void jointsLinearSpeedAxis(std::map<std::string, double>&, Frame*, Frame*, double, const std::string&, bool);
		void jointsAngularSpeed(std::map<std::string, double>&, Frame*, Frame*, double);

		void MeanLinearSpeed(std::vector<std::map<std::string, double>>&, Motion*, unsigned int);
		void MeanLinearSpeedAxis(std::vector<std::map<std::string, double>>&, Motion*, unsigned int, const std::string&, bool);
		Frame* getFrameFromTime(Motion*, double, double);

		void getGlobalCoordinates(Frame*, Frame*, Joint*, glm::dmat4);

		int getLocalMinimumFromMaximum(std::vector<double>&, int);
		int getLocalMaximum(std::vector<double>&, int);
		int getLocalMinimum(std::vector<double>&, int);

		void FindIndexSeparation(std::vector<double>&, unsigned int, unsigned int, std::vector<std::pair<int, int>>&);

		void MotionSegmentation(Motion*, SegmentationInformation&, std::vector<Motion*>&, const std::string&);

		//TODO: obviously, rename it
		void JESAISPASQUOI(Motion*, SegmentationInformation&, const std::string&, std::vector<std::map<std::string, double>>&);

		void motionSpeedComputing(Motion*, SpeedData&);
		void motionSpeedAxisComputing(Motion*, SpeedData&, const std::string&, bool);

		void ComputeSpeedData(std::vector<Motion*>&, std::vector<SpeedData>&);
		void ComputeSpeedAxis(std::vector<Motion*>&, std::vector<SpeedData>&, const std::string&, bool);

		void motionRebuilding(Motion*, Motion*, unsigned int);

		void motionFiltering(Motion*);

	};
};
#endif //__MLA_MOTIONOPERATION_H__
