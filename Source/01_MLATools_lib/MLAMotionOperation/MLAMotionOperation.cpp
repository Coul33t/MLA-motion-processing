#include "MLAMotionOperation.h"

namespace mla {
	namespace motionoperation {

		/** Interpolate a joint from 2 joints and a mix factor.

		@param j1 the first joint
		@param j2 the second joint
		@param mixFactor the mix factor between the 2 joints

		@return interpolatedJoint the interpolated joint
		*/
		Joint* interpolateJoint(Joint* j1, Joint* j2, double mixFactor) {
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
		Frame* interpolateFrame(Frame* f1, Frame* f2, double mixFactor) {

			Frame* interpolatedFrame = new Frame();

			for (unsigned int i = 0; i < f1->getJoints().size(); i++) {
				Joint* newJoint = interpolateJoint(f1->getJoint(i), f2->getJoint(i), mixFactor);

				newJoint->setJointName(f1->getJoint(i)->getJointName());

				if (f1->getJoint(i)->getParent() != nullptr) {
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
		std::map<std::string, double> jointsLinearSpeed(Frame* f1, Frame* f2, double frameTime) {
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
				double linSpeed = utility::vectorLength(v1, v2) / (frameTime * 100);
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
		std::map<std::string, double> jointsAngularSpeed(Frame* f1, Frame* f2, double frameTime) {
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
					double angSpeed = (acos(glm::dot(p1, p2) / (glm::length(p1) * glm::length(p2)))) / (frameTime * 1000);
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

		/** Compute the mean speed on given intervals for the full motion between two frames, with skeleton interpolation.

		@param motion the motion
		@param n the number of interval

		@return meanLinSpeed A vector containing the mean speed for each joint on the frames, for each interval
		*/
		std::vector<std::map<std::string, double>> MeanLinearSpeed(Motion* motion, unsigned int n) {
			std::vector<std::map<std::string, double>> meanLinSpeedInter;

			Frame* begFrame = nullptr;
			Frame* endFrame = nullptr;

			double frameTime = motion->getFrameTime();

			double  totalTime = motion->getFrames().size() * frameTime;

			double interval = totalTime / static_cast<float>(n);

			double firstFrameTime = 0;
			double endFrameTime = interval;

			for (unsigned int i = 0; i < n; i++) {
				begFrame = getFrameFromTime(motion, firstFrameTime, frameTime);
				endFrame = getFrameFromTime(motion, endFrameTime, frameTime);

				meanLinSpeedInter.push_back(jointsLinearSpeed(begFrame, endFrame, motion->getFrameTime()));

				firstFrameTime = endFrameTime;
				endFrameTime += interval;

				delete begFrame;
				delete endFrame;
			}

			return meanLinSpeedInter;
		}

		Frame* getFrameFromTime(Motion* motion, double time, double frameTime) {
			Frame* returnFrame = nullptr;

			unsigned int frameBef = static_cast<unsigned int>(time / frameTime);

			if (frameBef > 0)
				frameBef -= 1;

			unsigned int frameAft = frameBef + 1;

			double tpsBef = frameBef * frameTime;

			double mixFactor = ((time - tpsBef) / frameTime) - 1;

			if (mixFactor <= EPSILON)
				return (returnFrame = interpolateFrame(motion->getFrame(frameBef), motion->getFrame(frameBef), 0));
			else if (1 + EPSILON >= mixFactor && mixFactor >= 1 - EPSILON)
				return (returnFrame = interpolateFrame(motion->getFrame(frameAft), motion->getFrame(frameAft), 1));
			else
				return (returnFrame = interpolateFrame(motion->getFrame(frameBef), motion->getFrame(frameAft), mixFactor));
		}

		/** Recursively transforms initial joints (and its childs) local coordinates to global coordinates.

		@param currentJoint the initial joint
		@param globalCoord the global coordinates map
		@param globalMat the matrix used to compute global coordinates
		*/
		void getGlobalCoordinates(Joint* currentJoint, std::map<std::string, glm::dvec3>& globalCoord, std::map<std::string, glm::dmat4>& globalMat) {
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

		/** Find the next local minimum from the max value, if and only if this value is inferior to a threshold.

		@param data the data to process
		@param interFrameTime the interframe time
		@param direction the direction in which the local minimum is to be found (negative for left, positive for right)

		@return time the time at which the local minimum is found
		*/
		double getLocalMinimumFromMaximum(std::vector<double> data, double interFrameTime, int direction) {
			// We fix a threshold, to avoid minimums that are " too high ".
			// This threshold is set to (max + min) / 2.
			// UNUSED FOR THE MOMENT
			// double threshold = (utility::getMaxValue(data).first + utility::getMinValue(data).first) / 2.0;

			double last_value = utility::getMaxValue(data).first + 1;
			unsigned int idx = utility::getMaxValue(data).second;

			while (idx > 0 && idx < data.size() && last_value > data.at(idx)) {
				last_value = data.at(idx);
				idx += direction;
			}

			// idx - direction because we add the value even if we're good
			return (idx - direction) * interFrameTime;
		}
		/** Filter a motion, by eliminating position variations.

		@param motion the motion to be filtered
		*/
		void motionFiltering(Motion* motion) {
			// j = 1 because the first frame is the reference
			for (unsigned int i = 1; i < motion->getFrames().size(); i++) {
				// j = 1 because the root actually has position information
				for (unsigned int j = 0; j < motion->getFrame(i)->getJoints().size(); j++) {
					// if the joint has no parent = root
					if (motion->getFrame(i)->getJoint(j)->getParent() != nullptr)
						motion->getFrame(i)->getJoint(j)->setPositions(glm::dvec3(motion->getOffsetFrame()->getJoint(j)->getPositions()));
				}
			}
		}

	}
}