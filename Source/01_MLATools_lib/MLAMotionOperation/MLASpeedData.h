/**
Class for the speed data.
MLASpeedData.h

@author Ludovic Hamon & Quentin Couland
@version 0.1

*/

#ifndef __MLA_SPEED_DATA_H__
#define __MLA_SPEED_DATA_H__

#include <iostream>
#include <map>
#include <vector>
#include <string>

#include "MLAMotion/MLAMotion.h"

class SpeedData {

	private:
		struct BodySpeed {
			// Time at which the values are computed
			double m_time;
			// Set of speed values (x, y, z) for each joint at time m_time
			std::map<std::string, glm::dvec3> m_speed_set;
		};

	public:
		SpeedData(const unsigned int interval_number, const double interframe_time, const unsigned int nb_frame);
		~SpeedData();

		void getAllValues(std::vector<std::map<std::string, glm::dvec3>>&, bool normalise = false) const;
		void getAllValues(std::vector<std::map<std::string, double>>&, std::string, bool normalise = false) const;
		void getMeanSpeedValues(std::vector<double>&, std::string) const;
		void getMeanSpeedValues(std::vector<std::map<std::string, double>>&) const;


		bool isEmpty();

		glm::dvec3 getJointSpeed(const std::string& joint_name, const unsigned int index) const;
		bool getJointSpeedAndTime(const std::string& joint_name, const unsigned int index, std::pair<glm::dvec3, double>& speed_time) const;
		bool getJointSpeedAndTimeVector(const std::string& joint_name, std::vector<std::pair<glm::dvec3, double>>& speed_time_vector) const;

		bool getJointSpeedVector(const std::string& joint_name, std::vector<glm::dvec3>& speed_vector) const;
		bool getSpeedSetVector(std::vector<std::map<std::string, glm::dvec3>>& speed_set) const;

		const unsigned int getNbInterval() const;
		const double getIntervalTime() const;
		double getDuration() const;

		void addFrameSpeed(const std::map<std::string, glm::dvec3>& body_speed, double time);

	private:
		// Set of speed and corresponding time, for each joint for the whole motion
		std::vector<BodySpeed> m_speed_data;
		
		// Number of interval desired (??? The motion should dictate the number of interval)
		unsigned int m_interval_number;

		// Interval time
		double m_interval_time;


};

#endif //__MLA_SPEED_DATA_H__