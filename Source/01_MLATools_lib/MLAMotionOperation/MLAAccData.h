/**
Class for the acceleration data.
MLAAccData.h

@author Quentin Couland
@version 0.1

*/

#ifndef __MLA_ACC_DATA_H__
#define __MLA_ACC_DATA_H__

#include <iostream>
#include <map>
#include <vector>
#include <string>

#include "MLAMotion/MLAMotion.h"

class AccData {

private:
	struct BodyAcc {
		// Time at which the values are computed
		double m_time;
		// Set of speed values (x, y, z) for each joint at time m_time
		std::map<std::string, glm::dvec3> m_acc_set;
		std::map<std::string, double> m_norm;
	};

public:
	AccData(const unsigned int interval_number, const double interframe_time, const unsigned int nb_frame);
	~AccData();

	void getAllValues(std::vector<std::map<std::string, glm::dvec3>>&, bool normalise = false) const;
	void getAllValues(std::vector<glm::dvec3>&, const std::string&) const;
	void getAllValues(std::vector<std::map<std::string, double>>&, const std::string&, bool normalise = false) const;
	void getNorm(std::vector<double>&, const std::string&) const;
	void getNorm(std::vector<std::map<std::string, double>>&) const;


	bool isEmpty();

	glm::dvec3 getJointAcc(const std::string& joint_name, const unsigned int index) const;
	bool getJointAccAndTime(const std::string& joint_name, const unsigned int index, std::pair<glm::dvec3, double>& speed_time) const;
	bool getJointAccAndTimeVector(const std::string& joint_name, std::vector<std::pair<glm::dvec3, double>>& speed_time_vector) const;

	bool getJointAccVector(const std::string& joint_name, std::vector<glm::dvec3>& speed_vector) const;
	bool getAccSetVector(std::vector<std::map<std::string, glm::dvec3>>& speed_set) const;

	const unsigned int getNbInterval() const;
	const double getIntervalTime() const;
	double getDuration() const;
	const unsigned int getBodyAccCount() const;
	const std::vector<std::string> getJointNames() const;

	void addFrameAcc(const std::map<std::string, glm::dvec3>& body_speed, double time);

	void setAccSet(std::vector<double>&, std::string, unsigned int);
	void setNormSet(std::vector<double>&, std::string);

private:
	// Set of speed and corresponding time, for each joint for the whole motion
	std::vector<BodyAcc> m_acc_data;

	// Number of interval desired (??? The motion should dictate the number of interval)
	unsigned int m_interval_number;

	// Interval time
	double m_interval_time;


};

#endif //__MLA_ACC_DATA_H__