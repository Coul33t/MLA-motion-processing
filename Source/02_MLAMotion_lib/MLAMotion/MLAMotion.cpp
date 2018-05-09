#include "MLAMotion.h"

Motion::Motion() {

}

Motion::~Motion() {
	delete m_offset_frame;

	if(!m_frames.empty())
		for(unsigned int i = 0; i < m_frames.size(); i++)
			delete (m_frames[i]);
}

Motion::Motion(const Motion& motion) {
	m_frame_time = motion.m_frame_time;

	for (auto it = motion.m_frames.begin(); it != motion.m_frames.end(); it++)
		m_frames.push_back((*it)->duplicateFrame());

	m_offset_frame = motion.m_offset_frame->duplicateFrame();
	m_motion_name = motion.m_motion_name;
}

Motion& Motion::operator=(const Motion& motion) {
	m_frame_time = motion.m_frame_time;
	
	for (auto it = motion.m_frames.begin(); it != motion.m_frames.end(); it++)
		m_frames.push_back((*it)->duplicateFrame());

	m_offset_frame = motion.m_offset_frame->duplicateFrame();
	m_motion_name = motion.m_motion_name;
	return *this;
}

void Motion::setName(const std::string& name) {
	m_motion_name = name;
}

void Motion::setFrameTime(const double frame_time) {
	m_frame_time = frame_time;
}

void Motion::setOffsetFrame(Frame* frame) {
	m_offset_frame = frame;
}

void Motion::setFrames(const std::vector<Frame*>& frames_vector) {
	m_frames = frames_vector;
}

const std::string& Motion::getName() const {
	return m_motion_name;
}

const double& Motion::getFrameTime() const {
	return m_frame_time;
}

Frame* Motion::getOffsetFrame() const {
	return m_offset_frame;
}

const std::vector<Frame*>& Motion::getFrames() const {
	return m_frames;
}

const std::vector<Frame*> Motion::getFrames(unsigned int beg, unsigned int end) {
	return std::vector<Frame*>(m_frames.begin() + beg, m_frames.begin() + end);
}

void Motion::addFrame(Frame* frame) {
	m_frames.push_back(frame);
}

Frame* Motion::getFrame(unsigned int idx) const {
	if(idx < m_frames.size())
		return m_frames[idx];
	return 0;
}

MotionInformation Motion::getMotionInformation() {
	MotionInformation motion_info;
	motion_info.motion_name = m_motion_name;
	motion_info.frame_number = m_frames.size();
	motion_info.frame_time = m_frame_time;
	motion_info.root_name = m_frames[0]->getRoot()->getName();
	return motion_info;
}