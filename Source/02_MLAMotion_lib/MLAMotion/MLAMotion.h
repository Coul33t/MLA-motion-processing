/**
Class for the Motion. A motion is composed of a interframe time, and a vector of frames.
Character.h

@author Quentin Couland
@version 0.1

*/

#ifndef __MLA_MOTION_H__
#define __MLA_MOTION_H__



#include "MLAFrame.h"

class Motion {

public:
	Motion();
	~Motion();
	Motion(const Motion& motion);

	Motion& operator=(const Motion& motion);

	void setFrameTime(const double& frameTime);
	void setFrames(const std::vector<Frame*> Frames);

	const double& getFrameTime() const;
	const std::vector<Frame*>& getFrames() const;
	Frame* getFrame(unsigned int idx);

	void addFrame(Frame* frame);

	void interpolateJoint(Joint*, Joint*, Joint*, double);
	Frame* interpolateFrame(Frame*, Frame*, double);

private:
	std::string m_motionName;
	double m_frameTime;
	std::vector<Frame*> m_frames;
};

#endif //__MLA_MOTION_H__
