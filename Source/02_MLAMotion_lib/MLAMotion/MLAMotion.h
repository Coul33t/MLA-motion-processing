/**
Class for the Motion. A motion is composed of a interframe time, and a vector of frames.
Character.h

@author Quentin Couland
@version 0.1

*/

#ifndef __MLA_MOTION_H__
#define __MLA_MOTION_H__

#include "MLAFrame.h"

struct MotionInformation {
	std::string motion_name = "NO_NAME_SET";
	double frame_time = -1;
	int frame_number = -1;
	std::string root_name = "NO_ROOT_SET";
};

class Motion {

public:

	Motion();
	~Motion();
	Motion(const Motion& motion);
	//Motion(const std::shared_ptr<Motion> motion);

	Motion& operator=(const Motion& motion);
	//std::shared_ptr<Motion> operator=(const std::shared_ptr<Motion> motion);

	void setName(const std::string& name);
	void setFrameTime(const double frame_time);
	void setFrames(const std::vector<std::shared_ptr<Frame>>& frames_vector);
	void setOffsetFrame(std::shared_ptr<Frame> frame);

	const std::string& getName() const;
	const double& getFrameTime() const;
	std::shared_ptr<Frame> getOffsetFrame() const;
	const std::vector<std::shared_ptr<Frame>>& getFrames() const;
	const std::vector<std::shared_ptr<Frame>> getFrames(unsigned int beg, unsigned int end);
	std::shared_ptr<Frame> getFrame(unsigned int idx) const;

	MotionInformation getMotionInformation();

	void addFrame(std::shared_ptr<Frame> frame);

private:
	std::string m_motion_name;
	double m_frame_time;
	std::shared_ptr<Frame> m_offset_frame;
	std::vector<std::shared_ptr<Frame>> m_frames;

};

#endif //__MLA_MOTION_H__
