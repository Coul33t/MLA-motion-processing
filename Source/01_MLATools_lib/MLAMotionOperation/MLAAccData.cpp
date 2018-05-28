#include "MLAAccData.h"

AccData::AccData(const unsigned int interval_number, const double interframe_time, const unsigned int nb_frame)
	: m_interval_time((nb_frame * interframe_time) / static_cast<double>(interval_number)), m_interval_number(interval_number) {

}

AccData::~AccData() {

}


bool AccData::isEmpty() {
	return m_acc_data.empty();
}

/** Returns all the values of the Speed (as a vector of map containing a 3d vector for each joint)

@param values The returned vector containing maps of joints and their corresponding 3d vector of speed
@param normalise Whenever the vector should be normalised or not (default: false)
*/
void AccData::getAllValues(std::vector<std::map<std::string, glm::dvec3>>& values, bool normalise) const {
	values.clear();

	// For each speed map
	for (auto it = m_acc_data.begin(); it != m_acc_data.end(); ++it) {
		// Not normalised
		if (!normalise)
			values.push_back(it->m_acc_set);

		// Normalised
		else {
			// We declare a temp map, which will contain
			// the normalised values
			std::map<std::string, glm::dvec3> normalised_map = std::map<std::string, glm::dvec3>();

			// For each key-value in the speed set, we normalise it
			// and put it in the temp map
			for (auto& kv : it->m_acc_set)
				normalised_map[kv.first] = glm::normalize(kv.second);

			values.push_back(normalised_map);
		}

	}
}

/** Returns all the values of the Speed for one joint (as a vector of 3d vector)

@param values The returned vector containing the 3d vectors of the joints
@param normalise Whenever the vector should be normalised or not (default: false)
*/
void AccData::getAllValues(std::vector<glm::dvec3>& values, const std::string& joint) const {
	values.clear();

	// For each speed map
	for (auto it = m_acc_data.begin(); it != m_acc_data.end(); ++it)
		values.push_back(it->m_acc_set.at(joint));
}

/** Returns all the values of the Speed along an axis (as a vector of map containing a value for each joint)

@param values The returned vector containing maps of joints and their corresponding value of speed
@param normalise Whenever the vector should be normalised or not (default: false)

*/
void AccData::getAllValues(std::vector<std::map<std::string, double>>& values, const std::string& axis, bool normalise) const {

	values.clear();
	// A tmp map to put one axis of the vector
	std::map<std::string, double> tmp_map;

	// For each speed map
	for (auto it = m_acc_data.begin(); it != m_acc_data.end(); ++it) {
		tmp_map.clear();

		for (auto& kv : it->m_acc_set) {
			// Not normalised
			if (!normalise) {
				if (axis == "x")
					tmp_map[kv.first] = kv.second[0];
				else if (axis == "y")
					tmp_map[kv.first] = kv.second[1];
				else if (axis == "z")
					tmp_map[kv.first] = kv.second[2];
			}

			// Normalised
			else {
				auto vec = glm::normalize(kv.second);

				// Since we normalise, if ||v|| = 0, any vec component = NaN
				if (isnan(vec.x))
					tmp_map[kv.first] = 0;
				else if (axis == "x")
					tmp_map[kv.first] = vec[0];
				else if (axis == "y")
					tmp_map[kv.first] = vec[1];
				else if (axis == "z")
					tmp_map[kv.first] = vec[2];

			}

		}

		values.push_back(tmp_map);
	}
}

void AccData::getNorm(std::vector<double>& norm, const std::string& joint) const {
	norm.clear();

	// for each frame
	// Here, auto sets a const interator to the map.
	// Since C++11, const map can't use [] to access elements
	// (see https://stackoverflow.com/questions/1474894/why-isnt-the-operator-const-for-stl-maps)
	for (auto it = m_acc_data.begin(); it != m_acc_data.end(); ++it) {
		norm.push_back(it->m_norm.at(joint));
	}
}

void AccData::getNorm(std::vector<std::map<std::string, double>>& norm) const {
	norm.clear();

	// for each frame
	// Here, auto sets a const interator to the map.
	// Since C++11, const map can't use [] to access elements
	// (see https://stackoverflow.com/questions/1474894/why-isnt-the-operator-const-for-stl-maps)
	for (auto it = m_acc_data.begin(); it != m_acc_data.end(); ++it) {
		norm.push_back(it->m_norm);
	}
}

/** TODO: doc

*/
glm::dvec3 AccData::getJointAcc(const std::string& joint_name, const unsigned int index) const {

	if (index < m_acc_data.size()) {
		std::map<std::string, glm::dvec3>::const_iterator iter = m_acc_data[index].m_acc_set.find(joint_name);

		if (iter != m_acc_data[index].m_acc_set.end()) {
			return iter->second;
		}
	}

	return glm::dvec3(0, 0, 0);
}

/** Returns a couple of acceleration and time, at a designated index.

@param jointName the name of the joint
@param index the index of the frame
@param speedTime the returned values of speed and time

@return bool whenever the values for the index and the joint were found or not
*/
bool AccData::getJointAccAndTime(const std::string& joint_name, const unsigned int index, std::pair<glm::dvec3, double>& acc_time) const {

	if (index < m_acc_data.size()) {
		std::map<std::string, glm::dvec3>::const_iterator iter = m_acc_data[index].m_acc_set.find(joint_name);

		if (iter != m_acc_data[index].m_acc_set.end()) {
			acc_time.first = iter->second;
			acc_time.second = m_acc_data[index].m_time;
			return true;
		}
	}

	return false;
}

/** Returns a vector of couples of acceleration and time.

@param jointName the name of the joint
@param index the index of the frame
@param a pair of acceleration and time

@return bool whenever the joint has been found and/or if it's vector is empty
*/
bool AccData::getJointAccAndTimeVector(const std::string& joint_name, std::vector<std::pair<glm::dvec3, double>>& acc_time_vector) const {

	acc_time_vector.clear();

	// IF the vector is empty OR the joint is not found in the map
	if (m_acc_data.empty() || m_acc_data[0].m_acc_set.count(joint_name) == 0)
		return false;

	for (unsigned int i = 0; i < m_acc_data.size(); ++i) {
		acc_time_vector.push_back(std::pair<glm::dvec3, double>(m_acc_data[i].m_acc_set.at(joint_name), m_acc_data[i].m_time));
	}

	return true;
}

/** Returns a vector of accelerations for one joint (whole motion).

@param jointName the name of the joint
@param the velocity vector

@return bool whenever the joint has been found and/or if it's vector is empty
*/
bool AccData::getJointAccVector(const std::string& joint_name, std::vector<glm::dvec3>& speed_vector) const {
	speed_vector.clear();

	// IF the vector is empty OR the joint is not found in the map
	if (m_acc_data.empty() || m_acc_data[0].m_acc_set.count(joint_name) == 0)
		return false;

	for (unsigned int i = 0; i < m_acc_data.size(); ++i) {
		speed_vector.push_back(m_acc_data[i].m_acc_set.at(joint_name));
	}

	return true;
}

/** Returns a vector of accelerations for all the joints (whole motion).

@param the velocity vector

@return bool returns if the speed vector is empty
*/
bool AccData::getAccSetVector(std::vector<std::map<std::string, glm::dvec3>>& speed_set) const {
	speed_set.clear();

	if (m_acc_data.empty())
		return false;

	for (unsigned int i = 0; i < m_acc_data.size(); ++i) {
		speed_set.push_back(m_acc_data[i].m_acc_set);
	}

	return true;
}


const unsigned int AccData::getNbInterval() const {
	return m_interval_number;
}

const double AccData::getIntervalTime() const {
	return m_interval_time;
}

// ??? Duration is a property of the motion, not the computed speed
double AccData::getDuration() const {
	return (double)m_interval_number * m_interval_time;
}

const unsigned int AccData::getBodyAccCount() const {
	return m_acc_data.size();
}

const std::vector<std::string> AccData::getJointNames() const {
	std::vector<std::string> names;

	for (auto& kv : m_acc_data[0].m_acc_set)
		names.push_back(kv.first);

	return names;
}

void AccData::addFrameAcc(const std::map<std::string, glm::dvec3>& body_acc, double time) {
	BodyAcc newbodyAcc;
	newbodyAcc.m_acc_set = body_acc;
	newbodyAcc.m_time = time;

	std::map<std::string, double> body_norm;
	for (auto& kv : body_acc) {
		body_norm[kv.first] = glm::length(kv.second);
	}

	newbodyAcc.m_norm = body_norm;

	m_acc_data.push_back(newbodyAcc);
}

void AccData::setAccSet(std::vector<double>& data, std::string joint, unsigned int axis) {
	if (m_acc_data[0].m_acc_set.find(joint) != m_acc_data[0].m_acc_set.end()) {
		for (unsigned int i = 0; i < data.size(); ++i) {
			m_acc_data[i].m_acc_set[joint][axis] = data[i];
		}
	}
}

void AccData::setNormSet(std::vector<double>& data, std::string joint) {
	if (m_acc_data[0].m_norm.find(joint) != m_acc_data[0].m_norm.end()) {
		for (unsigned int i = 0; i < data.size(); ++i) {
			m_acc_data[i].m_norm[joint] = data[i];
		}
	}
}