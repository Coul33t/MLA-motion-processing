#include "MLASpeedData.h"

//constructeurs
MLASpeedData::MLASpeedData(const double& intervalNumber, const double& timeStepInterFrame, const double& nbframe)
	: m_intervalTime((nbframe *timeStepInterFrame) / static_cast<double>(intervalNumber)) {

}

MLASpeedData::~MLASpeedData() {

}

//accesseurs
bool MLASpeedData::getJointSpeedAndTime(const std::string& jointName, const unsigned int& index, const double* speedValue, const double* timeValue) const {
	
	if (index < m_speedData.size()) {
		std::map<std::string, double>::const_iterator iter = m_speedData[index].m_speedSet.find(jointName);

		if (iter != m_speedData[index].m_speedSet.end()) {
			speedValue = &iter->second;
			timeValue = &m_speedData[index].m_time;
			return true;
		}
	}

	return false;
}

//accesseurs
bool MLASpeedData::getJointSpeedAndTimeVector(const std::string& jointName, std::vector<std::pair<double, double>>& speedAndTimeVector) const {
	
	speedAndTimeVector.clear();

	// IF the vector is empty OR the joint is not found in the map
	if (m_speedData.empty() || m_speedData[0].m_speedSet.find(jointName) == m_speedData[0].m_speedSet.end())
		return false;
	
	for (unsigned int i = 0; i < m_speedData.size(); ++i) {
		speedAndTimeVector.push_back(std::pair<double, double>(m_speedData[i].m_speedSet.at(jointName), m_speedData[i].m_time));
	}

	return true;
}


unsigned int MLASpeedData::getNbInterval() const { 
	return m_speedData.size(); 
}

const double& MLASpeedData::getIntervalTime() const { 
	return m_intervalTime; 
}

double MLASpeedData::getDuration() const { 
	return (double)(getNbInterval()) * m_intervalTime; 
}

//modificateurs
void MLASpeedData::addFrameSpeed(const std::map<std::string, double>& bodySpeed) {
	MLABodySpeed newbodySpeed;
	newbodySpeed.m_speedSet = bodySpeed;
	newbodySpeed.m_time = getNbInterval() * m_intervalTime;
	m_speedData.push_back(newbodySpeed);
}

