#include "MLAMotionOperation.h"

/** Interpolate a joint from 2 joints and a mix factor.

		@param j1 the first joint
		@param j2 the second joint
		@param mixFactor the mix factor between the 2 joints

		@return interpolatedJoint the interpolated joint
*/
Joint* MotionOperation::interpolateJoint(Joint* j1, Joint* j2, double mixFactor) {
	Joint* interpolatedJoint = new Joint();

	glm::dvec3 pos1 = j1->getPositions();
	glm::dvec3 pos2 = j2->getPositions();

	interpolatedJoint->setPositions(glm::dvec3(pos2.x + (pos1.x - pos2.x)*mixFactor,
		pos2[1] + (pos1.y - pos2.y)*mixFactor,
		pos2[2] + (pos1.z - pos2.z)*mixFactor));

	interpolatedJoint->setOrientations(glm::slerp(j1->getOrientations(), j2->getOrientations(), mixFactor));
	return interpolatedJoint;
}

/** Interpolate a frame from 2 frame and a mix factor.

		@param f1 the first frame
		@param f2 the second frame
		@param mixFactor the mix factor between the 2 frames

		@return interpolatedFrame the interpolated frame
*/
Frame* MotionOperation::interpolateFrame(Frame* f1, Frame* f2, double mixFactor) {

	Frame* interpolatedFrame = new Frame();

	for (unsigned int i = 0; i<f1->getJoints().size(); i++) {
		Joint* newJoint = interpolateJoint(f1->getJoint(i), f2->getJoint(i), mixFactor);
		
		newJoint->setJointName(f1->getJoint(i)->getJointName());

		if (f1->getJoint(i)->getParent() == nullptr)
			interpolatedFrame->addRoot(newJoint);

		else {
			newJoint->setParent(interpolatedFrame->getJoint(f1->getJoint(i)->getParent()->getJointName()));
			newJoint->getParent()->addChild(newJoint);
		}


		interpolatedFrame->insertJoint(newJoint);
	}

	return interpolatedFrame;
}

/** Compute the speed of the joints between 2 frames.

		@param f1 the first frame
		@param f2 the second frame
		@param framTime the interframe time

		@return speedVector the linear speed of joints
*/
std::map<std::string, double> MotionOperation::jointsLinearSpeed(Frame* f1, Frame* f2, double frameTime) {
	std::map<std::string, glm::dvec3> globalCoord1;
	std::map<std::string, glm::dvec3> globalCoord2;

	// Only used in the recursive function
	// (hey I made a working recursive function, yay !)
	std::map<std::string, glm::dmat4> globalMat1;
	std::map<std::string, glm::dmat4> globalMat2;

	getGlobalCoordinates(f1->getJoint(0), globalCoord1, globalMat1);
	getGlobalCoordinates(f2->getJoint(0), globalCoord2, globalMat2);

	std::map<std::string, double> linSpeedVector;

	for (std::map<std::string, glm::dvec3> ::iterator it = globalCoord1.begin(); it != globalCoord1.end(); ++it) {
		glm::dvec3 v1 = globalCoord1.find(it->first)->second;
		glm::dvec3 v2 = globalCoord2.find(it->first)->second;

		// dx / dt -> cm / s
		// dx / (dt * 100) -> m / s
		double linSpeed = vectorLength(v1, v2) / (frameTime * 100);
		linSpeedVector.insert(std::pair<std::string, double>(it->first, linSpeed));
	}

	return linSpeedVector;
}

/** Compute the speed of the joints between 2 frames.

		@param f1 the first frame
		@param f2 the second frame
		@param framTime the interframe time

		@return speedVector the angular speed of joints
*/
std::map<std::string, double> MotionOperation::jointsAngularSpeed(Frame* f1, Frame* f2, double frameTime) {
	std::map<std::string, glm::dvec3> globalCoord1;
	std::map<std::string, glm::dvec3> globalCoord2;

	// Only used in the recursive function
	// (hey I made a working recursive function, yay !)
	std::map<std::string, glm::dmat4> globalMat1;
	std::map<std::string, glm::dmat4> globalMat2;

	getGlobalCoordinates(f1->getJoint(0), globalCoord1, globalMat1);
	getGlobalCoordinates(f2->getJoint(0), globalCoord2, globalMat2);

	std::map<std::string, double> angularSpeedVector;

	for (std::map<std::string, glm::dvec3> ::iterator it = globalCoord1.begin(); it != globalCoord1.end(); ++it) {
		glm::dvec3 p1 = globalCoord1.find(it->first)->second;
		glm::dvec3 p2 = globalCoord2.find(it->first)->second;

		if (glm::length(p1) * glm::length(p2) > 0) {
			// rad / sec
			double angSpeed = ( acos(glm::dot(p1, p2) / (glm::length(p1) * glm::length(p2))) ) / (frameTime * 1000);
			angularSpeedVector.insert(std::pair<std::string, double>(it->first, angSpeed));
		}

		else
			angularSpeedVector.insert(std::pair<std::string, double>(it->first, 0));
		
		
		/*double r = vectorLength(glm::dvec3(0, 0, 0), f1->getJoint(it->first)->getPositions());
		
		if (r > 0) {
			double alpha = acos(glm::radians(vectorLength(v1, v2) / pow(r, 2)));
			if ((unsigned)(vectorLength(v1, v2) / pow(r, 2)) > 1)
				std::cout << "ERROR: VALUE OUT OF RANGE " << vectorLength(v1, v2) << " / " << pow(r, 2) << " = " << vectorLength(v1, v2) / pow(r, 2) << std::endl;

			double angSpeed = alpha / (frameTime * 100);
			angularSpeedVector.insert(std::pair<std::string, double>(it->first, angSpeed));
		}

		else
			angularSpeedVector.insert(std::pair<std::string, double>(it->first, 0));*/
	}

	return angularSpeedVector;
}

/** Compute the mean speed on a set of frames.

		@param frames the set of frames

		@return meanLinSpeed the mean speed for each joint on the frames
*/
std::map<std::string, double> MotionOperation::MeanLinearSpeed(std::vector<Frame*> frames, double frameTime) {
	std::map<std::string, double> meanLinSpeed;

	std::vector<std::map<std::string, double>> linSpeed;
	std::map<std::string, std::vector<double>> tmp;

	for (std::vector<Frame*>::iterator it = frames.begin(); it != frames.end() - 1; ++it) {
		linSpeed.push_back(jointsLinearSpeed(*it, *(it + 1), frameTime));
	}

	for (std::vector<std::map<std::string, double>>::iterator itVec = linSpeed.begin(); itVec != linSpeed.end(); ++itVec) {
		for (std::map<std::string, double>::iterator itMap = (*itVec).begin(); itMap != (*itVec).end(); ++itMap) {
			tmp[itMap->first].push_back(itMap->second);
		}
	}

	for (std::map<std::string, std::vector<double>>::iterator it = tmp.begin(); it != tmp.end(); ++it) {
		meanLinSpeed[it->first] = std::accumulate(it->second.begin(), it->second.end(), 0.0) / it->second.size();
	}

	return meanLinSpeed;
}

/** Compute the mean speed on given intervals for the full motion.

		@param motion the motion
		@param n the number of interval

		@return meanLinSpeed A vector containing the mean speed for each joint on the frames, for each interval
*/
std::vector<std::map<std::string, double>> MotionOperation::MeanLinearSpeedInterval(Motion* motion, unsigned int n) {
	std::vector<std::map<std::string, double>> meanLinSpeedInter;

	unsigned int motionLength = motion->getFrames().size();
	double interval = static_cast<float>(motionLength) / static_cast<float>(n);


	for (unsigned int i = 0; i<n; i++) {
		if (i == n-1) {
			meanLinSpeedInter.push_back(MeanLinearSpeed(std::vector<Frame*>(motion->getFrames(static_cast<int>(floor(i * interval)), motionLength)), motion->getFrameTime()));
		}
		else {
			meanLinSpeedInter.push_back(MeanLinearSpeed(std::vector<Frame*>(motion->getFrames(static_cast<int>(floor(i * interval)), static_cast<int>(floor((i * interval) + interval)))), motion->getFrameTime()));
		}
	}

	return meanLinSpeedInter;
}

/** Compute the mean speed on given intervals for the full motion between two frames, with skeleton interpolation.

@param motion the motion
@param n the number of interval

@return meanLinSpeed A vector containing the mean speed for each joint on the frames, for each interval
*/
std::vector<std::map<std::string, double>> MotionOperation::MeanLinearSpeedIntervalFrame(Motion* motion, unsigned int n) {
	std::vector<std::map<std::string, double>> meanLinSpeedInter;


	Frame* begFrame = motion->getFrame(1);

	Frame* endFrame = nullptr;

	int endFrameIdx = -1;
	double mixFactor = 0;

	// -1 because motion->getFrame(0) = offsets
	double interval = static_cast<float>(motion->getFrames().size() - 1) / static_cast<float>(n);

	for (unsigned int i = 0; i < n; i++) {
		// Get the idx of the end frame before the " true " value
		// ex : 3.3333
		//   -> 3
		// Why -1 :
		// Let says a motion with 91 frames
		// interval = 45.5
		// Natural  : 1 -> 45.5, 45.5 -> 91
		// Computer : 0 -> 44.5, 44.5 -> 90
		endFrameIdx = static_cast<unsigned int>((i * interval) + interval) - 1;
		
		// Only keep the part after the comma, which is our mix factor
		// ex : 3.3333
		//   -> 0.3333
		mixFactor = (((i * interval) + interval) - 1) - endFrameIdx;

		// The true end frame
		// Why -1 :
		// The motion has 91 frames (from 1 to 91)
		// motion->getFrame(0) => motion->getFrame(90)
		// So, let say endFrameIdx = 45
		// Natural  : 45 -> 46
		// Computer : 44 -> 45 
		endFrame = interpolateFrame(motion->getFrame(endFrameIdx), motion->getFrame(endFrameIdx + 1), mixFactor);

		meanLinSpeedInter.push_back(jointsLinearSpeed(begFrame, endFrame, motion->getFrameTime()));

		// The old end frame became the new beginning frame for the next interval
		begFrame = endFrame;
	}

	return meanLinSpeedInter;
}


/** Recursively transforms initial joints (and its childs) local coordinates to global coordinates.

		@param currentJoint the initial joint
		@param globalCoord the global coordinates map
		@param globalMat the matrix used to compute global coordinates
*/
void MotionOperation::getGlobalCoordinates(Joint* currentJoint, std::map<std::string, glm::dvec3>& globalCoord, std::map<std::string, glm::dmat4>& globalMat) {
	// If it's the root
	if (!currentJoint->getParent()) {
		glm::dmat4 mat = glm::dmat4(1.0);
		mat = glm::translate(mat, currentJoint->getPositions());
		mat *= glm::mat4_cast(currentJoint->getOrientations());

		globalMat.insert(std::pair<std::string, glm::dmat4>(currentJoint->getJointName(), mat));

		globalCoord.insert(std::pair<std::string, glm::dvec3>(currentJoint->getJointName(), glm::dvec3(mat[3][0], mat[3][1], mat[3][2])));
	}

	else {
		glm::dmat4 mat = globalMat.find(currentJoint->getParent()->getJointName())->second;
		mat = glm::translate(mat, currentJoint->getPositions());
		mat *= glm::mat4_cast(currentJoint->getOrientations());

		glm::dvec3 global = glm::dvec3(mat[3][0], mat[3][1], mat[3][2]);

		globalMat.insert(std::pair<std::string, glm::dmat4>(currentJoint->getJointName(), mat));
		globalCoord.insert(std::pair<std::string, glm::dvec3>(currentJoint->getJointName(), glm::dvec3(mat[3][0], mat[3][1], mat[3][2])));

	}

	for (unsigned int i = 0; i < currentJoint->getChilds().size(); i++) {
		getGlobalCoordinates(currentJoint->getChilds().at(i), globalCoord, globalMat);
	}
}

/** Filter a motion, by eliminating position variations.

		@param motion the motion to be filtered
*/
void MotionOperation::motionFiltering(Motion* motion) {
	// j = 1 because the first frame is the reference
	for (unsigned int i = 1; i < motion->getFrames().size(); i++) {
		// j = 1 because the root actually has position information
		for (unsigned int j = 0; j < motion->getFrame(i)->getJoints().size(); j++) {
			// if the two strings are equal (case insensitive)
			// The old method of using j = 1 (skipping the ROOT) does not works in all cases,
			// as Motion Builder, for example, adds a " Reference " root. Fuck you MB.
			if (_stricmp(motion->getFrame(i)->getJoint(j)->getJointName().c_str(), "hips") != 0)
				motion->getFrame(i)->getJoint(j)->setPositions(glm::dvec3(motion->getFrame(0)->getJoint(j)->getPositions()));
		}
	}
}

double MotionOperation::vectorLength(glm::dvec3 p1, glm::dvec3 p2) {
	return sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2) + pow(p2.z - p1.z, 2));
}

double MotionOperation::roundToDecimal(double number, unsigned int decimal) {
	return (number * pow(10, decimal)) / pow(10,decimal);
}