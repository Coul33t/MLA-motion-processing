/**
Class for the Motion. A motion is composed of a interframe time, and a vector of frames.
Character.h

@author Quentin Couland
@version 0.1

*/

#ifndef __MLA_MOTION_H__
#define __MLA_MOTION_H__


#include "MLATools_lib.h"
#include "MLAFrame.h"

class Motion {

public:
	Motion();
	Motion(const Motion& motion);

	Motion& operator=(const Motion& motion);

	void setFrameTime(const float& frameTime);
	void setFrames(const std::vector<Frame> Frames);

	const float& getFrameTime() const;
	const std::vector<Frame>& getFrames() const;

private:
	float m_frameTime;
	std::vector<Frame> m_frames;
};

#endif //__MLA_MOTION_H__
