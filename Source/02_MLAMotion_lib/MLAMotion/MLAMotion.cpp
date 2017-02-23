#include "MLAMotion.h"

Motion::Motion() {

}

Motion::~Motion() {
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