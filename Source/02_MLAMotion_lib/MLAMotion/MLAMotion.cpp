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

