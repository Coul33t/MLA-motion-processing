#include "MLAMotion.h"

Motion::Motion() {

}

Motion::~Motion() {
	if(!m_frames.empty())
		for(unsigned int i=0 ; i<m_frames.size() ; i++)
			free(m_frames.at(i));
}

Motion::Motion(const Motion& motion) {
	m_frameTime = motion.m_frameTime;
	m_frames = motion.m_frames;
}

Motion& Motion::operator=(const Motion& motion) {
	m_frameTime = motion.m_frameTime;
	m_frames = motion.m_frames;
	return *this;
}

void Motion::setFrameTime(const double& frameTime) {
	m_frameTime = frameTime;
}

void Motion::setFrames(const std::vector<Frame*> frames) {
	m_frames = frames;
}

const double& Motion::getFrameTime() const {
	return m_frameTime;
}

const std::vector<Frame*>& Motion::getFrames() const {
	return m_frames;
}

void Motion::addFrame(Frame* frame) {
	m_frames.push_back(frame);
}

Frame* Motion::getFrame(unsigned int idx) {
	if(idx < m_frames.size())
		return m_frames.at(idx);
	return 0;
}


void Motion::interpolateJoint(Joint* j1, Joint* j2, Joint* j3, double mixFactor) {
	glm::dvec3 pos1 = j1->getPositions();
	glm::dvec3 pos2 = j2->getPositions();

	j3->setPositions(glm::dvec3(pos2.x + (pos1.x - pos2.x)*mixFactor,
		pos2[1] + (pos1.y - pos2.y)*mixFactor,
		pos2[2] + (pos1.z - pos2.z)*mixFactor));

	j3->setOrientations(glm::slerp(j1->getOrientations(), j2->getOrientations(), mixFactor));
}

Frame* Motion::interpolateFrame(Frame* f1, Frame* f2, double mixFactor) {
	
	Frame* newFrame = new Frame();

	for (unsigned int i=0 ; i<f1->getJoints().size() ; i++) {
		Joint* newJoint = new Joint();
		newJoint->setJointName(f1->getJoint(i)->getJointName());
		interpolateJoint(f1->getJoint(i), f2->getJoint(i), newJoint, mixFactor);
		
		if (f1->getJoint(i)->getParent() == nullptr)
			newFrame->addRoot(newJoint);

		else {
			newJoint->setParent(newFrame->getJoint(f1->getJoint(i)->getParent()->getJointName()));
			newJoint->getParent()->addChild(newJoint);		
		}
			

		newFrame->insertJoint(newJoint);
	}

	return newFrame;
}

//TODO: move to tools
void Motion::jointsSpeed(Frame* f1, Frame* f2) {
	glm::dmat4 matF1 = glm::mat4_cast(f1->getJoint(0)->getOrientations());
	glm::dmat4 matF2 = glm::mat4_cast(f2->getJoint(0)->getOrientations());

	std::map<std::string, glm::dvec3> globalCoord1;
	std::map<std::string, glm::dvec3> globalCoord2;

	// Only used in the recursive function
	// (hey I made a working recursive function, yay !)
	std::map<std::string, glm::dmat4> globalMat1;
	std::map<std::string, glm::dmat4> globalMat2;

	getGlobalCoordinates(f1->getJoint(0), globalCoord1, globalMat1);
	getGlobalCoordinates(f2->getJoint(0), globalCoord2, globalMat2);

	std::map<std::string, double> speedVector;

	for (std::map<std::string, glm::dvec3> ::iterator it = globalCoord1.begin(); it != globalCoord1.end(); ++it) {
		glm::dvec3 v1 = globalCoord1.find(it->first)->second;
		glm::dvec3 v2 = globalCoord2.find(it->first)->second;

		double v = sqrt(pow(v2.x - v1.x, 2) + pow(v2.y - v1.y, 2) + pow(v2.z - v1.z, 2)) / m_frameTime;
		speedVector.insert(std::pair<std::string, double>(it->first, v));
		std::cout << "Speed of " << it->first << " : " << v << std::endl;
	}
}

//TODO: move to tools
void Motion::getGlobalCoordinates(Joint* currentJoint, std::map<std::string, glm::dvec3>& globalCoord, std::map<std::string, glm::dmat4>& globalMat) {
	// If it's the root
	if(!currentJoint->getParent()) {
		glm::dmat4 mat = glm::dmat4(1.0);
		mat = glm::translate(mat, currentJoint->getPositions());
		mat *= glm::mat4_cast(currentJoint->getOrientations());

		globalMat.insert(std::pair<std::string, glm::dmat4>(currentJoint->getJointName(), mat));

		/*glm::dmat4 parentMat = glm::mat4_cast(currentJoint->getOrientations());
		parentMat *= glm::dvec4(currentJoint->getPositions().x,
			currentJoint->getPositions().y,
			currentJoint->getPositions().z,
			1);*/

		globalCoord.insert(std::pair<std::string, glm::dvec3>(currentJoint->getJointName(), glm::dvec3(mat[3][0], mat[3][1], mat[3][2])));
	}

	else {
		glm::dmat4 mat = globalMat.find(currentJoint->getParent()->getJointName())->second;
		mat = glm::translate(mat, currentJoint->getPositions());
		mat *= glm::mat4_cast(currentJoint->getOrientations());
		/*parentMat *= glm::dvec4(currentJoint->getParent()->getPositions().x,
								currentJoint->getParent()->getPositions().y,
								currentJoint->getParent()->getPositions().z,
								1);*/


		glm::dvec3 global = glm::dvec3(mat[3][0], mat[3][1], mat[3][2]);
		
		globalMat.insert(std::pair<std::string, glm::dmat4>(currentJoint->getJointName(), mat));
		globalCoord.insert(std::pair<std::string, glm::dvec3>(currentJoint->getJointName(), glm::dvec3(mat[3][0], mat[3][1], mat[3][2])));

	}

	for (unsigned int i=0 ; i < currentJoint->getChilds().size() ; i++) {
		getGlobalCoordinates(currentJoint->getChilds().at(i), globalCoord, globalMat);
	}
}

//TODO: move to tools
void Motion::motionFiltering() {
	// j = 1 because the first frame is the reference
	for(unsigned int i=1 ; i < this->getFrames().size() ; i++) {
		// j = 1 because the root actually has position information
		for (unsigned int j = 1; j < this->getFrame(i)->getJoints().size(); j++) {
			this->getFrame(i)->getJoint(j)->setPositions(glm::dvec3(this->getFrame(0)->getJoint(j)->getPositions()));
		}
	}
}