#include "MLAMotion.h"

Motion::Motion() {

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

void Motion::setFrameTime(const float& frameTime) {
	m_frameTime = frameTime;
}

void Motion::setFrames(const std::vector<Frame> frames) {
	m_frames = frames;
}

const float& Motion::getFrameTime() const {
	return m_frameTime;
}

const std::vector<Frame>& Motion::getFrames() const {
	return m_frames;
}