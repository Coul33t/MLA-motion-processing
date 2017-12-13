#include "MLASpeedData.h"

SpeedData::SpeedData(const unsigned int interval_number, const double interframe_time, const unsigned int nb_frame)
	: m_interval_time((nb_frame * interframe_time) / static_cast<double>(interval_number)), m_interval_number(interval_number) {

}

SpeedData::~SpeedData() {

}


/** Returns a couple of speed and time, at a designated index.

	@param jointName the name of the joint
	@param index the index of the frame
	@param speedTime the returned values of speed and time
	
	@return bool whenever the values for the index and the joint were found or not
*/
bool SpeedData::getJointSpeedAndTime(const std::string& joint_name, const unsigned int index, std::pair<double, double>& speed_time) const {

	if (index < m_speed_data.size()) {
		std::map<std::string, double>::const_iterator iter = m_speed_data[index].m_speed_set.find(joint_name);

		if (iter != m_speed_data[index].m_speed_set.end()) {
			speed_time.first = iter->second;
			speed_time.second = m_speed_data[index].m_time;
			return true;
		}
	}

	return false;
}

/** Returns a vector of couples of speed and time.

@param jointName the name of the joint
@param index the index of the frame
@param a pair of speed and time

@return bool whenever the joint has been found and/or if it's vector is empty
*/
bool SpeedData::getJointSpeedAndTimeVector(const std::string& joint_name, std::vector<std::pair<double, double>>& speed_time_vector) const {
	
	speed_time_vector.clear();

	// IF the vector is empty OR the joint is not found in the map
	if (m_speed_data.empty() || m_speed_data[0].m_speed_set.count(joint_name) == 0)
		return false;
	
	for (unsigned int i = 0; i < m_speed_data.size(); ++i) {
		speed_time_vector.push_back(std::pair<double, double>(m_speed_data[i].m_speed_set.at(joint_name), m_speed_data[i].m_time));
	}

	return true;
}

/** Returns a vector of speeds for one joint (whole motion).

@param jointName the name of the joint
@param the velocity vector 

@return bool whenever the joint has been found and/or if it's vector is empty
*/
bool SpeedData::getJointSpeedVector(const std::string& joint_name, std::vector<double>& speed_vector) const {
	speed_vector.clear();

	// IF the vector is empty OR the joint is not found in the map
	if (m_speed_data.empty() || m_speed_data[0].m_speed_set.count(joint_name) == 0)
		return false;

	for (unsigned int i = 0; i < m_speed_data.size(); ++i) {
		speed_vector.push_back(m_speed_data[i].m_speed_set.at(joint_name));
	}

	return true;
}

/** Returns a vector of speeds for all the joints (whole motion).

@param the velocity vector

@return bool returns if the speed vector is empty
*/
bool SpeedData::getSpeedSetVector(std::vector<std::map<std::string, double>>& speed_set) const {
	speed_set.clear();
	
	if (m_speed_data.empty())
		return false;

	for (unsigned int i = 0; i < m_speed_data.size(); ++i) {
		speed_set.push_back(m_speed_data[i].m_speed_set);
	}

	return true;
}


const unsigned int SpeedData::getNbInterval() const { 
	return m_interval_number;
}

const double SpeedData::getIntervalTime() const { 
	return m_interval_time;
}

// ??? Duration is a property of the motion, not the computed speed
double SpeedData::getDuration() const {
	return (double)m_interval_number * m_interval_time;
}

void SpeedData::addFrameSpeed(const std::map<std::string, double>& body_speed, double time) {
	BodySpeed newbodySpeed;
	newbodySpeed.m_speed_set = body_speed;
	newbodySpeed.m_time = time;
	m_speed_data.push_back(newbodySpeed);
}
