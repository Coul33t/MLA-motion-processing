#include "MLAMotionOperation.h"

namespace Mla {
	namespace MotionOperation {

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
		void jointsLinearSpeed(std::map<std::string, double>& linSpeedVector, Frame* f1, Frame* f2, double frameTime) {
			std::map<std::string, glm::dvec3> globalCoord1;
			std::map<std::string, glm::dvec3> globalCoord2;

			// Only used in the recursive function
			// (hey I made a working recursive function, yay !)
			std::map<std::string, glm::dmat4> globalMat1;
			std::map<std::string, glm::dmat4> globalMat2;

			getGlobalCoordinates(f1->getJoint(0), globalCoord1, globalMat1);
			getGlobalCoordinates(f2->getJoint(0), globalCoord2, globalMat2);

			for (std::map<std::string, glm::dvec3> ::iterator it = globalCoord1.begin(); it != globalCoord1.end(); ++it) {
				glm::dvec3 v1 = globalCoord1.find(it->first)->second;
				glm::dvec3 v2 = globalCoord2.find(it->first)->second;

				// dx / dt -> cm / s
				// dx / (dt * 100) -> m / s
				double linSpeed = Mla::Utility::vectorLength(v1, v2) / (frameTime * 100);
				linSpeedVector.insert(std::pair<std::string, double>(it->first, linSpeed));
			}

		}

		/** Compute the speed of the joints between 2 frames.

		@param f1 the first frame
		@param f2 the second frame
		@param framTime the interframe time

		@return speedVector the angular speed of joints
		*/
		void jointsAngularSpeed(std::map<std::string, double>& angularSpeedVector, Frame* f1, Frame* f2, double frameTime) {
			std::map<std::string, glm::dvec3> globalCoord1;
			std::map<std::string, glm::dvec3> globalCoord2;

			// Only used in the recursive function
			// (hey I made a working recursive function, yay !)
			std::map<std::string, glm::dmat4> globalMat1;
			std::map<std::string, glm::dmat4> globalMat2;

			getGlobalCoordinates(f1->getJoint(0), globalCoord1, globalMat1);
			getGlobalCoordinates(f2->getJoint(0), globalCoord2, globalMat2);

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

			}
		}

		/** Compute the mean speed on given intervals for the full motion between two frames, with skeleton interpolation.

		@param motion the motion
		@param n the number of interval

		@return meanLinSpeed A vector containing the mean speed for each joint on the frames, for each interval
		*/
		void MeanLinearSpeed(std::vector<std::map<std::string, double>>& meanLinSpeedInter, Motion* motion, unsigned int n) {

			Frame* begFrame = nullptr;
			Frame* endFrame = nullptr;

			double frameTime = motion->getFrameTime();

			double totalTime = motion->getFrames().size() * frameTime;

			double interval = totalTime / static_cast<float>(n);

			double firstFrameTime = 0;
			double endFrameTime = interval;

			for (unsigned int i = 0; i < n; i++) {
				begFrame = getFrameFromTime(motion, firstFrameTime, frameTime);
				endFrame = getFrameFromTime(motion, endFrameTime, frameTime);

				std::map<std::string, double> linSpeedVector;
				jointsLinearSpeed(linSpeedVector, begFrame, endFrame, motion->getFrameTime());
				meanLinSpeedInter.push_back(linSpeedVector);

				firstFrameTime = endFrameTime;
				endFrameTime += interval;

				delete begFrame;
				delete endFrame;
			}
		}

		/** Return an interpolated frame from a motion, and a time

		@param motion the motion from which the frame will be interpolated
		@param time the time at which the frame will be interpolated
		@param frame_time the interframe_time from the initial motion
		
		@return new_frame the interpolated frame
		*/
		Frame* getFrameFromTime(Motion* motion, double time, double frame_time) {
			Frame* returnFrame = nullptr;

			unsigned int frameBef = static_cast<unsigned int>(time / frame_time);

			// ???
			if (frameBef > 0)
				frameBef -= 1;

			unsigned int frameAft = frameBef + 1;

			double tpsBef = frameBef * frame_time;

			// -1 ???
			double mixFactor = ((time - tpsBef) / frame_time) - 1;

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
		@param direction the direction in which the local minimum is to be found (negative for left, positive for right)

		@return idx the index at which the local minimum is found
		*/
		int getLocalMinimumFromMaximum(std::vector<double>& data, int direction) {
			// We fix a threshold, to avoid minimums that are " too high ".
			// This threshold is set to (max + min) / 2.
			// UNUSED FOR THE MOMENT
			// double threshold = (Mla::Utility::getMaxValue(data).first + Mla::Utility::getMinValue(data).first) / 2.0;

			if (direction == 0) {
				std::cout << "ERROR: direction must be != 0 (d < 0 for left search, d > 0 for right search)" << std::endl;
				return -1;
			}

			double last_value = Mla::Utility::getMaxValue(data).first + 1;
			unsigned int idx = Mla::Utility::getMaxValue(data).second;

			while (idx > 0 && idx < data.size() && last_value > data.at(idx)) {
				last_value = data.at(idx);
				idx += direction;
			}

			// idx - direction because we add the value even if we're good
			return idx - direction;
		}

		/** Find the next local maximum in the data

		@param data the data to process
		@param direction the direction in which the local maximum is to be found (negative for left, positive for right)

		@return idx the index at which the local minimum is found
		*/
		int getLocalMaximum (std::vector<double>& data, int direction) {
			if (direction == 0) {
				std::cout << "ERROR: direction must be != 0 (d < 0 for left search, d > 0 for right search)" << std::endl;
				return -1;
			}
			
			// Case right
			double last_value = data[0];
			// Case left
			if (direction < 0)
				last_value = data.back();

			// - 1 because of the loop condition
			last_value--;

			// Case right
			int idx = 0;
			// Case left
			if (direction < 0)
				idx = data.size() - 1;

			while (idx > -1 && idx < data.size() && last_value < data.at(idx)) {
				last_value = data.at(idx);
				idx += direction;
			}

			// idx - direction because we add the value even if we're good
			return idx - direction;
		}

		/** Find the next local minmum in the data

		@param data the data to process
		@param direction the direction in which the local minimum is to be found (negative for left, positive for right)

		@return idx the index at which the local minimum is found
		*/
		int getLocalMinimum (std::vector<double>& data, int direction) {
			if (direction == 0) {
				std::cout << "ERROR: direction must be != 0 (d < 0 for left search, d > 0 for right search)" << std::endl;
				return -1;
			}

			// Case right
			double last_value = data[0];
			// Case left
			if (direction < 0)
				last_value = data.back();

			// - 1 because of the loop condition
			last_value++;

			// Case right
			int idx = 0;
			// Case left
			if (direction < 0)
				idx = data.size() - 1;

			while (idx > -1 && idx < data.size() && last_value > data.at(idx)) {
				last_value = data.at(idx);
				idx += direction;
			}

			// idx - direction because we add the value even if we're good
			return idx - direction;
		}
		
		/** Segment the motion into n segments, from one local minimum to another

		@param data the data to process
		@param left_cut the number of segment to the left
		@param right_cut the number of segment to the right
		@param interframe_time the interframe time
		@param cut_times the output vector, with a pair of (begin, end) for each segment of the motion
		*/
		void FindIndexSeparation(std::vector<double>& data, unsigned int left_cut, unsigned int right_cut, std::vector<std::pair<int, int>>& cut_times) {
			// Pair: idx begin, idx end
			std::pair<unsigned int, unsigned int> cut_time;

			// First, we segment the throw (" most useful " part of the motion)
			cut_time.first = getLocalMinimumFromMaximum(data, -1);
			cut_time.second = getLocalMinimumFromMaximum(data, 1);

			cut_times.push_back(cut_time);

			// Used to search the next local minimum from the local maximum
			int local_max = -1;

			// Used to get a subset of the data vector
			std::vector<double> sub_vector;

			// Then, we get the nth left cuts
			for (unsigned int i = 0; i < left_cut ; i++) {
				// The left of the last cut become the right of the new cut
				cut_time.second = cut_time.first;

				// We extract the data's part which have not been processed yet
				sub_vector = std::vector<double>(data.begin(), data.begin() + cut_time.second);

				local_max = getLocalMaximum(sub_vector, -1);

				// If we're at the beginning of the signal, then there's nothing
				// to the left, so there's no more left cut
				if (local_max == 0) {
					std::cout << "Warning: last left cut (" << i + 1 << "/" << left_cut << ") is halved (cause: beginning of signal)" << std::endl;
					cut_time.first = local_max;
					break;
				}

				sub_vector = std::vector<double>(data.begin(), data.begin() + local_max);

				cut_time.first = getLocalMinimum(sub_vector, -1);
				// We insert it at the beginning
				cut_times.insert(cut_times.begin(), cut_time);
			}


			// We take the initial cut (the throw) index back
			cut_time.first = cut_times.back().first;
			cut_time.second = cut_times.back().second;

			// The, we get the nth right cuts
			for (unsigned int i = 0; i < right_cut; i++) {
				// The right of the last cut become the left of the new cut
				cut_time.first = cut_time.second;

				sub_vector = std::vector<double>(data.begin() + cut_time.first, data.end());

				// + cut_time.first, because it doesn't start at the beginning
				local_max = getLocalMaximum(sub_vector, +1) + cut_time.first;

				// If we're at the end of the signal, then there's nothing
				// to the right, so there's no more right cut
				if (local_max == sub_vector.size() + cut_time.first - 1) {
					std::cout << "Warning: last right cut (" << i + 1 << "/" << right_cut << ") is halved (cause: end of signal)" << std::endl;
					cut_time.second = local_max;
					break;
				}

				sub_vector = std::vector<double>(data.begin() + local_max, data.end());

				cut_time.second = getLocalMinimum(sub_vector, +1) + local_max;
				cut_times.push_back(cut_time);
			}
		}

		/** Rebuild a motion from an initial one, with a new count of frames.

		@param original_motion the original motion
		@param frames_number the new frames count

		@return new_motion the new motion, constructed from the original one	
		*/
		void motionRebuilding (Motion* original_motion, Motion* new_motion, unsigned int frames_number) {
			if (frames_number < 2) {
				*new_motion = Motion();
				return;
			}
				
			Frame* frame_to_insert = new Frame();

			// First, we copy the name and the offset frame
			new_motion->setName(original_motion->getName());

			frame_to_insert = original_motion->getOffsetFrame();
			new_motion->setOffsetFrame(frame_to_insert);


			// Then, we get the time at which we'll have to interpolate frames
			// original_time / new_number_of_frames

			// Used for the getFrameFromTime() function
			double frame_time = original_motion->getFrameTime();

			// frameNumber - 1 
			double new_interval = (original_motion->getFrames().size() * frame_time) / static_cast<double>(frames_number - 1);
			new_motion->setFrameTime(new_interval);

			double current_time = 0.0;

			for (unsigned int i = 0; i < frames_number; i++) {
				frame_to_insert = getFrameFromTime(original_motion, current_time, frame_time);
				new_motion->addFrame(frame_to_insert);
				current_time += new_interval;
			}
				
		}

		/** Segment a motion into submotions.

		@param initial_motion The motion to be segmented
		@param left_cut The number of segment to the left of the global maximum
		@param right_cut The number of segment to the right of the global maximum
		@param savgol_window_size [SAVGOL] The window size for the savgol algorithm
		@param savgol_polynom_order [SAVGOL] The polynom order for the savgol algorithm
		@param frame_number_cut The final number of frame for each submotion
		@param motion_segments The different segments returned

		*/
		void MotionSegmentation (Motion* initial_motion, int left_cut, int right_cut, int savgol_window_size, int savgol_polynom_order, int frame_number_cut, std::vector<Motion*>& motion_segments) {
			Motion* sub_motion = nullptr;

			std::map<std::string, double> lin_speed;
			// Import data into a map, to separate the hand data from the rest of the joints
			std::vector<std::map<std::string, double>> map_full;

			for (unsigned int i = 0; i < initial_motion->getFrames().size() - 1; i++) {
				lin_speed.clear();
				Mla::MotionOperation::jointsLinearSpeed(lin_speed, initial_motion->getFrame(i), initial_motion->getFrame(i + 1), initial_motion->getFrameTime());
				map_full.push_back(lin_speed);
			}

			std::vector<double> hand_lin_speed;

			// Extracting hand values
			for (unsigned int i = 0; i < map_full.size(); ++i) {
				std::map<std::string, double> hand_map = map_full[i];
				hand_lin_speed.push_back(hand_map.find("LeftHand")->second);
			}

			// Savgol-ing the values
			std::vector<double> savgoled;

			Mla::Filters::Savgol(savgoled, hand_lin_speed, savgol_polynom_order, savgol_window_size);

			std::vector<std::pair<int, int>> separation_indexes;

			Mla::MotionOperation::FindIndexSeparation(savgoled, left_cut, right_cut, separation_indexes);

			for (unsigned int i = 0; i < separation_indexes.size(); i++) {
				
				sub_motion = new Motion();
				sub_motion->setName(initial_motion->getName());
				sub_motion->setFrameTime(initial_motion->getFrameTime());
				sub_motion->setOffsetFrame(initial_motion->getOffsetFrame()->duplicateFrame());

				for (int j = separation_indexes[i].first; j < separation_indexes[i].second + 1; j++) {
					sub_motion->addFrame(initial_motion->getFrame(j)->duplicateFrame());
				}

				Motion* cut_motion = new Motion();
				Mla::MotionOperation::motionRebuilding(sub_motion, cut_motion, 20);

				delete sub_motion;

				motion_segments.push_back(cut_motion);
			}


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