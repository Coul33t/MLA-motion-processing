#include "MLASpeedData.h"

SpeedData::SpeedData(const double& intervalNumber, const double& timeStepInterFrame, const double& nbframe)
	: m_intervalTime((nbframe * timeStepInterFrame) / static_cast<double>(intervalNumber)) {

}

SpeedData::~SpeedData() {

}


/** Returns a couple of speed and time.

	@param jointName the name of the joint
	@param index the index of the frame
	@param speedTime the returned values of speed and time
	
	@return bool whenever the values for the index and the joint were found or not
*/
bool SpeedData::getJointSpeedAndTime(const std::string& jointName, const unsigned int index, std::pair<double, double>& speedTime) const {

	if (index < m_speedData.size()) {
		std::map<std::string, double>::const_iterator iter = m_speedData[index].m_speedSet.find(jointName);

		if (iter != m_speedData[index].m_speedSet.end()) {
			speedTime.first = iter->second;
			speedTime.second = m_speedData[index].m_time;
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
bool SpeedData::getJointSpeedAndTimeVector(const std::string& jointName, std::vector<std::pair<double, double>>& speedAndTimeVector) const {
	
	speedAndTimeVector.clear();

	// IF the vector is empty OR the joint is not found in the map
	if (m_speedData.empty() || m_speedData[0].m_speedSet.count(jointName) == 0)
		return false;
	
	for (unsigned int i = 0; i < m_speedData.size(); ++i) {
		speedAndTimeVector.push_back(std::pair<double, double>(m_speedData[i].m_speedSet.at(jointName), m_speedData[i].m_time));
	}

	return true;
}


unsigned int SpeedData::getNbInterval() const { 
	return m_speedData.size(); 
}

const double SpeedData::getIntervalTime() const { 
	return m_intervalTime; 
}

double SpeedData::getDuration() const { 
	return (double)(m_speedData.size()) * m_intervalTime;
}


void SpeedData::addFrameSpeed(const std::map<std::string, double>& bodySpeed) {
	MLABodySpeed newbodySpeed;
	newbodySpeed.m_speedSet = bodySpeed;
	newbodySpeed.m_time = getNbInterval() * m_intervalTime;
	m_speedData.push_back(newbodySpeed);
}

