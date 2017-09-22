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

	void setName(const std::string& name);
	void setFrameTime(const double frameTime);
	void setFrames(const std::vector<Frame*> Frames);
	void setOffsetFrame(Frame* frame);

	const std::string& getName() const;
	const double& getFrameTime() const;
	Frame* getOffsetFrame() const;
	const std::vector<Frame*>& getFrames() const;
	const std::vector<Frame*> getFrames(unsigned int beg, unsigned int end);
	Frame* getFrame(unsigned int idx) const;

	void addFrame(Frame* frame);

private:
	std::string m_motionName;
	double m_frameTime;
	Frame* m_offsetFrame;
	std::vector<Frame*> m_frames;
};

#endif //__MLA_MOTION_H__
