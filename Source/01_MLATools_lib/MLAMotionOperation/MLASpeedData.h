/**
Class for the speed data.
MLASpeedData.h

@author Quentin Couland
@version 0.1

*/

#ifndef __MLA_SPEED_DATA_H__
#define __MLA_SPEED_DATA_H__

#include <iostream>
#include <map>
#include <vector>
#include <string>

#include "MLAMotion/MLAMotion.h"
#include "MLAMotionOperation/MLAMotionOperation.h"

class SpeedData {

	private:
		struct MLABodySpeed {
			// Time at which the values are computed
			double m_time;
			// Set of speed values for each joint at time m_time
			std::map<std::string, double> m_speedSet;
		};

	public:
		SpeedData(const unsigned int interval_number, const double interframe_time, const unsigned int nb_frame);
		SpeedData(const unsigned int interval_number, const double interframe_time, const unsigned int nb_frame, Motion* motion);
		~SpeedData();

		bool getJointSpeedAndTime(const std::string& joint_name, const unsigned int index, std::pair<double, double>& speed_time) const;
		bool getJointSpeedAndTimeVector(const std::string& joint_name, std::vector<std::pair<double, double>>& speed_time_vector) const;

		bool getJointSpeedVector(const std::string& joint_name, std::vector<double>& speed_vector) const;
		bool getSpeedSetVector(std::vector<std::map<std::string, double>>& speed_set) const;
		
		void motionSpeedComputing(Motion* motion);

		const unsigned int getNbInterval() const;
		const double getIntervalTime() const;
		double getDuration() const;

		void addFrameSpeed(const std::map<std::string, double>& body_speed, double time);

	private:
		// Set of speed and corresponding time, for each joint for the whole motion
		std::vector<MLABodySpeed> m_speed_data; 
		
		// Number of interval desired (??? The motion should dictate the number of interval)
		unsigned int m_interval_number;

		// Interval time
		double m_interval_time;


};

#endif //__MLA_SPEED_DATA_H__