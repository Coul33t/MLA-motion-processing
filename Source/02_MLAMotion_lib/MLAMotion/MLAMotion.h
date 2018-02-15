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
	std::string motion_name;
	double frame_time;
	int frame_number;
	std::string root_name;

};

class Motion {

public:

	Motion();
	~Motion();
	Motion(const Motion& motion);

	Motion& operator=(const Motion& motion);

	void setName(const std::string& name);
	void setFrameTime(const double frame_time);
	void setFrames(const std::vector<Frame*>& frames_vector);
	void setOffsetFrame(Frame* frame);

	const std::string& getName() const;
	const double& getFrameTime() const;
	Frame* getOffsetFrame() const;
	const std::vector<Frame*>& getFrames() const;
	const std::vector<Frame*> getFrames(unsigned int beg, unsigned int end);
	Frame* getFrame(unsigned int idx) const;

	MotionInformation getMotionInformation();

	void addFrame(Frame* frame);

private:
	std::string m_motion_name;
	double m_frame_time;
	Frame* m_offset_frame;
	std::vector<Frame*> m_frames;

};

#endif //__MLA_MOTION_H__
