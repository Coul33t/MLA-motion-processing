#include "MLAMotionOperation.h"

namespace Mla {
	namespace MotionOperation {

		/** Interpolate a joint from 2 joints and a mix factor.

		@param j1 the first joint
		@param j2 the second joint
		@param mixFactor the mix factor between the 2 joints

		@return interpolatedJoint the interpolated joint
		*/
		Joint* interpolateJoint (Joint* j1, Joint* j2, double mix_factor) {
			Joint* interpolated_joint = new Joint();

			glm::dvec3 pos1 = j1->getPositions();
			glm::dvec3 pos2 = j2->getPositions();

			interpolated_joint->setPositions(glm::dvec3(pos2.x + (pos1.x - pos2.x) * mix_factor,
				pos2[1] + (pos1.y - pos2.y) * mix_factor,
				pos2[2] + (pos1.z - pos2.z) * mix_factor));

			interpolated_joint->setOrientations(glm::slerp(j1->getOrientations(), j2->getOrientations(), mix_factor));
			return interpolated_joint;
		}

		/** Interpolate a frame from 2 frame and a mix factor.

		@param f1 the first frame
		@param f2 the second frame
		@param mixFactor the mix factor between the 2 frames

		@return interpolatedFrame the interpolated frame
		*/
		Frame* interpolateFrame (Frame* f1, Frame* f2, double mix_factor) {

			Frame* interpolated_frame = new Frame();

			for (unsigned int i = 0; i < f1->getJoints().size(); i++) {
				Joint* new_joint = interpolateJoint(f1->getJoint(i), f2->getJoint(i), mix_factor);

				new_joint->setName(f1->getJoint(i)->getName());

				if (f1->getJoint(i)->getParent() != nullptr) {
					new_joint->setParent(interpolated_frame->getJoint(f1->getJoint(i)->getParent()->getName()));
					new_joint->getParent()->addChild(new_joint);
				}

				else {
					interpolated_frame->setRoot(new_joint);
				}

				interpolated_frame->insertJoint(new_joint);
			}

			return interpolated_frame;
		}

		/** Compute the speed of the joints between 2 frames.

		@param f1 the first frame
		@param f2 the second frame
		@param framTime the interframe time

		@return speedVector the linear speed of joints
		*/
		void jointsLinearSpeed (std::map<std::string, double>& lin_speed_vector, Frame* f1, Frame* f2, double frame_time) {
			
			Frame* global_frame_1 = f1->duplicateFrame();
			Frame* global_frame_2 = f2->duplicateFrame();

			getGlobalCoordinates(f1, global_frame_1, f1->getJoint("Hips"), glm::dmat4(1.0));
			getGlobalCoordinates(f2, global_frame_2, f2->getJoint("Hips"), glm::dmat4(1.0));

			double linear_speed = 0;

			for (unsigned int j = 0; j < global_frame_1->getJoints().size(); j++) {
				glm::dvec3 v1 = global_frame_1->getJoint(j)->getPositions();
				glm::dvec3 v2 = global_frame_2->getJoint(j)->getPositions();

				// dx / dt -> cm / s
				// dx / (dt * 100) -> m / s
				linear_speed = glm::distance(v1, v2) / (frame_time * 100);
				lin_speed_vector.insert(std::pair<std::string, double>(global_frame_1->getJoint(j)->getName(), linear_speed));
			}

			delete global_frame_1;
			delete global_frame_2;

		}

		/** Compute the speed of the joints between 2 frames.

		@param f1 the first frame
		@param f2 the second frame
		@param framTime the interframe time

		@return speedVector the linear speed of joints as a 3d vector
		*/
		void jointsLinearSpeed (std::map<std::string, glm::dvec3>& lin_speed_vector, Frame* f1, Frame* f2, double frame_time) {
			Frame* global_frame_1 = f1->duplicateFrame();
			Frame* global_frame_2 = f2->duplicateFrame();

			getGlobalCoordinates(f1, global_frame_1, f1->getJoint("Hips"), glm::dmat4(1.0));
			getGlobalCoordinates(f2, global_frame_2, f2->getJoint("Hips"), glm::dmat4(1.0));

			glm::dvec3 linear_speed = glm::dvec3();

			for (unsigned int j = 0; j < global_frame_1->getJoints().size(); j++) {
				glm::dvec3 v1 = global_frame_1->getJoint(j)->getPositions();
				glm::dvec3 v2 = global_frame_2->getJoint(j)->getPositions();

				// dx / dt -> cm / s
				// dx / (dt * 100) -> m / s
				linear_speed[0] = (v2.x - v1.x) / (frame_time * 100);
				linear_speed[1] = (v2.y - v1.y) / (frame_time * 100);
				linear_speed[2] = (v2.z - v1.z) / (frame_time * 100);		
				lin_speed_vector.insert(std::pair<std::string, glm::dvec3>(global_frame_1->getJoint(j)->getName(), linear_speed));
			}

			delete global_frame_1;
			delete global_frame_2;
		}

		/** Compute the speed (along an axis) of the joints between 2 frames.

		@param f1 the first frame
		@param f2 the second frame
		@param framTime the interframe time

		@return speedVector the linear speed of joints
		*/
		void jointsLinearSpeedAxis (std::map<std::string, double>& lin_speed_vector, Frame* f1, Frame* f2, double frame_time, const std::string& axis, bool normalise) {

			Frame* global_frame_1 = f1->duplicateFrame();
			Frame* global_frame_2 = f2->duplicateFrame();

			getGlobalCoordinates(f1, global_frame_1, f1->getJoint("Hips"), glm::dmat4(1.0));
			getGlobalCoordinates(f2, global_frame_2, f2->getJoint("Hips"), glm::dmat4(1.0));

			double linear_speed = 0;

			for (unsigned int j = 0; j < global_frame_1->getJoints().size(); j++) {
				glm::dvec3 v1 = global_frame_1->getJoint(j)->getPositions();
				glm::dvec3 v2 = global_frame_2->getJoint(j)->getPositions();
				
				double norm = glm::distance(v1, v2);
				// dx / dt -> cm / s
				// dx / (dt * 100) -> m / s
				if (axis == "x")
					linear_speed = (v2.x - v1.x) / (frame_time * 100);
				else if (axis == "y")
					linear_speed = (v2.y - v1.y) / (frame_time * 100);
				else if (axis == "z")
					linear_speed = (v2.z - v1.z) / (frame_time * 100);

				if (normalise && norm != 0.0)
					linear_speed = linear_speed / norm;

				lin_speed_vector.insert(std::pair<std::string,double>(global_frame_1->getJoint(j)->getName(), linear_speed));
			}

			delete global_frame_1;
			delete global_frame_2;

		}

		/** Compute the speed of the joints between 2 frames.

		@param f1 the first frame
		@param f2 the second frame
		@param framTime the interframe time

		@return speedVector the angular speed of joints
		*/
		void jointsAngularSpeed (std::map<std::string, double>& ang_speed_vector, Frame* f1, Frame* f2, double frame_time) {
			
			Frame* global_frame_1 = f1->duplicateFrame();
			Frame* global_frame_2 = f2->duplicateFrame();

			getGlobalCoordinates(f1, global_frame_1, f1->getJoint("Hips"), glm::dmat4(1.0));
			getGlobalCoordinates(f2, global_frame_2, f2->getJoint("Hips"), glm::dmat4(1.0));

			double angular_speed = 0;

			for (unsigned int j = 0; j < global_frame_1->getJoints().size(); j++) {
				glm::dvec3 p1 = global_frame_1->getJoint(j)->getPositions();
				glm::dvec3 p2 = global_frame_2->getJoint(j)->getPositions();

				if (glm::length(p1) * glm::length(p2) > 0) {
					// rad / sec
					angular_speed = (acos(glm::dot(p1, p2) / (glm::length(p1) * glm::length(p2)))) / (frame_time * 1000);
					ang_speed_vector.insert(std::pair<std::string, double>(global_frame_1->getJoint(j)->getName(), angular_speed));
				}

				else
					ang_speed_vector.insert(std::pair<std::string, double>(global_frame_1->getJoint(j)->getName(), 0));

			}
		}

		/** Compute the mean speed on given intervals for the full motion between two frames, with skeleton interpolation.

		@param motion the motion
		@param n the number of interval

		@return meanLinSpeed A vector containing the mean speed for each joint on the frames, for each interval
		*/
		void MeanLinearSpeed (std::vector<std::map<std::string, double>>& mean_lin_speed_inter, Motion* motion, unsigned int n) {

			Frame* beg_frame = nullptr;
			Frame* end_frame = nullptr;

			double frame_time = motion->getFrameTime();

			double total_time = motion->getFrames().size() * frame_time;

			double interval = total_time / static_cast<float>(n);

			double first_frame_time = 0;
			double end_frame_time = interval;

			for (unsigned int i = 0; i < n; i++) {
				beg_frame = getFrameFromTime(motion, first_frame_time, frame_time);
				end_frame = getFrameFromTime(motion, end_frame_time, frame_time);

				std::map<std::string, double> lin_speed_vector;
				jointsLinearSpeed(lin_speed_vector, beg_frame, end_frame, motion->getFrameTime());
				mean_lin_speed_inter.push_back(lin_speed_vector);

				first_frame_time = end_frame_time;
				end_frame_time += interval;

				delete beg_frame;
				delete end_frame;
			}
		}

		/** Compute the mean speed (on the 3 axis) on given intervals for the full motion between two frames, with skeleton interpolation.

		@param motion the motion
		@param n the number of interval

		@return meanLinSpeed A vector containing the mean speed (on the 3 axis) for each joint on the frames, for each interval
		*/
		void MeanLinearSpeedAxis (std::vector<std::map<std::string, double>>& mean_lin_speed_inter, Motion* motion, unsigned int n, const std::string& axis, bool normalise) {

			Frame* beg_frame = nullptr;
			Frame* end_frame = nullptr;

			double frame_time = motion->getFrameTime();

			double total_time = motion->getFrames().size() * frame_time;

			double interval = total_time / static_cast<float>(n);

			double first_frame_time = 0;
			double end_frame_time = interval;

			for (unsigned int i = 0; i < n; i++) {
				beg_frame = getFrameFromTime(motion, first_frame_time, frame_time);
				end_frame = getFrameFromTime(motion, end_frame_time, frame_time);

				std::map<std::string, double> lin_speed_vector;
				jointsLinearSpeedAxis(lin_speed_vector, beg_frame, end_frame, motion->getFrameTime(), axis, normalise);
				mean_lin_speed_inter.push_back(lin_speed_vector);

				first_frame_time = end_frame_time;
				end_frame_time += interval;

				delete beg_frame;
				delete end_frame;
			}
		}
		
		/** Return an interpolated frame from a motion, and a time

		@param motion the motion from which the frame will be interpolated
		@param time the time at which the frame will be interpolated
		@param frame_time the interframe_time from the initial motion
		
		@return new_frame the interpolated frame
		*/
		Frame* getFrameFromTime (Motion* motion, double time, double frame_time) {
			Frame* returnFrame = nullptr;

			unsigned int frame_bef = static_cast<unsigned int>(time / frame_time);

			// ???
			if (frame_bef > 0)
				frame_bef -= 1;

			unsigned int frame_aft = frame_bef + 1;

			double tpsBef = frame_bef * frame_time;

			// -1 ???
			double mixFactor = ((time - tpsBef) / frame_time) - 1;

			if (mixFactor <= EPSILON)
				return (returnFrame = interpolateFrame(motion->getFrame(frame_bef), motion->getFrame(frame_bef), 0));

			else if (1 + EPSILON >= mixFactor && mixFactor >= 1 - EPSILON)
				return (returnFrame = interpolateFrame(motion->getFrame(frame_aft), motion->getFrame(frame_aft), 1));

			else
				return (returnFrame = interpolateFrame(motion->getFrame(frame_bef), motion->getFrame(frame_aft), mixFactor));
		}

		/** Recursively transforms initial joints (and its childs) local coordinates to global coordinates.

		@param local_frame the initial frame (local coordinates)
		@param global_frame the (intially a copy of the local frame) frame containing global coordinates
		@param current_joint the joint currently being processed (from the local_frame)
		@param global_mat the modelview matrix (first call: identity matrix)
		*/
		void getGlobalCoordinates (Frame* local_frame, Frame* global_frame, Joint* current_joint, glm::dmat4 global_mat) {

			global_mat = glm::translate(global_mat, current_joint->getPositions());
			global_mat *= glm::mat4_cast(current_joint->getOrientations());

			global_frame->getJoint(current_joint->getName())->setPositions(glm::dvec3(global_mat[3][0], global_mat[3][1], global_mat[3][2]));
			global_frame->getJoint(current_joint->getName())->setOrientations(glm::dvec3(0, 0, 0));

			for (unsigned int i = 0; i < current_joint->getChilds().size(); i++) {
				getGlobalCoordinates(local_frame, global_frame, current_joint->getChilds()[i], global_mat);
			}
		}

		/** Find the next local minimum from the max value, if and only if this value is inferior to a threshold.

		@param data the data to process
		@param direction the direction in which the local minimum is to be found (negative for left, positive for right)

		@return idx the index at which the local minimum is found
		*/
		int getLocalMinimumFromMaximum (std::vector<double>& data, int direction) {
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

			if (idx == 0 && direction < 0)
				return 0;

			if (idx == data.size() && direction > 0)
				return data.size() - 1;

			// Problem here
			while (idx > 0 && idx < data.size() && last_value > data[idx]) {
				last_value = data[idx];
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

			// Why static_cast<int>(data.size())?
			// unsigned int i = 1;
			// int j = -1;
			// (i < j) -> false	(see https://stackoverflow.com/questions/5416414/signed-unsigned-comparisons)
			while (idx > -1 && idx < static_cast<int>(data.size()) && last_value < data[idx]) {
				last_value = data[idx];
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

			while (idx > -1 && idx < static_cast<int>(data.size()) && last_value > data[idx]) {
				last_value = data[idx];
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
		void FindIndexSeparation (std::vector<double>& data, unsigned int left_cut, unsigned int right_cut, std::vector<std::pair<int, int>>& cut_times) {
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
					cut_times.insert(cut_times.begin(), cut_time);
					break;
				}
				
				sub_vector = std::vector<double>(data.begin(), data.begin() + local_max);

				cut_time.first = getLocalMinimum(sub_vector, -1);
				// We insert it at the beginning
				cut_times.insert(cut_times.begin(), cut_time);

				if (cut_time.first == 0) {
					std::cout << "Warning: last left cut at " << i + 1 << "/" << left_cut << " (cause: beginning of signal)" << std::endl;
					break;
				}
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
				if (local_max == static_cast<int>(sub_vector.size() + cut_time.first - 1)) {
					std::cout << "Warning: last right cut (" << i + 1 << "/" << right_cut << ") is halved (cause: end of signal)" << std::endl;
					cut_time.second = local_max;
					cut_times.push_back(cut_time);
					break;
				}

				sub_vector = std::vector<double>(data.begin() + local_max, data.end());

				cut_time.second = getLocalMinimum(sub_vector, +1) + local_max;
				cut_times.push_back(cut_time);

				if (cut_time.second == static_cast<int>(sub_vector.size() + cut_time.first - 1)) {
					std::cout << "Warning: last right cut at " << i + 1 << "/" << left_cut << " (cause: end of signal)" << std::endl;
					break;
				}

			}
		}

		/** Rebuild a motion from an initial one, with a new count of frames.

		@param original_motion the original motion
		@param new_motion the new motion, constructed from the original one
		@param frames_number the new frames count	
		*/
		void motionRebuilding (Motion* original_motion, Motion* new_motion, unsigned int frames_number) {
			if (frames_number < 2) {
				*new_motion = Motion();
				return;
			}
				
			Frame* frame_to_insert = new Frame();

			// First, we copy the name and the offset frame
			new_motion->setName(original_motion->getName());

			frame_to_insert = original_motion->getOffsetFrame()->duplicateFrame();
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
		@param joint_to_segment The join tused to find the [mini/maxi]mums
		*/
		void MotionSegmentation (Motion* initial_motion, SegmentationInformation& seg_info, std::vector<Motion*>& motion_segments, const std::string& joint_to_segment) {
			Motion* sub_motion = nullptr;

			SpeedData speed_data(initial_motion->getFrames().size() - 1, 
								 initial_motion->getFrameTime(), 
								 initial_motion->getFrames().size());

			motionSpeedComputing(initial_motion, speed_data);

			std::vector<double> hand_lin_speed;
			speed_data.getMeanSpeedValues(hand_lin_speed, joint_to_segment);

			// Savgol-ing the values
			std::vector<double> savgoled;

			// hand_lin_speed -> accessor class
			Mla::Filters::Savgol(savgoled, hand_lin_speed, seg_info.savgol_polynom_order, seg_info.savgol_window_size);

			// Finding the separation indexes
			std::vector<std::pair<int, int>> separation_indexes;

			Mla::MotionOperation::FindIndexSeparation(savgoled, seg_info.left_cut, seg_info.right_cut, separation_indexes);

			// Creating the new submotions
			for (unsigned int i = 0; i < separation_indexes.size(); i++) {
				
				sub_motion = new Motion();
				sub_motion->setName(initial_motion->getName());
				sub_motion->setFrameTime(initial_motion->getFrameTime());
				sub_motion->setOffsetFrame(initial_motion->getOffsetFrame()->duplicateFrame());

				
				for (int j = separation_indexes[i].first; j < separation_indexes[i].second + 1; j++) {
					sub_motion->addFrame(initial_motion->getFrame(j)->duplicateFrame());
				}

				Motion* cut_motion = new Motion();
				// reconstruct the motion
				Mla::MotionOperation::motionRebuilding(sub_motion, cut_motion, seg_info.final_frame_number);

				delete sub_motion;

				motion_segments.push_back(cut_motion);
			}

		}

		/** TODO: doc
		
		*/
		void getBegEndIndexes(Motion* motion, SegmentationInformation& seg_info, const std::string& joint_to_segment, std::vector<int>& throw_idx) {
			SpeedData speed_data(motion->getFrames().size() - 1,
				motion->getFrameTime(),
				motion->getFrames().size());

			motionSpeedComputing(motion, speed_data);

			std::vector<double> hand_lin_speed;
			speed_data.getMeanSpeedValues(hand_lin_speed, joint_to_segment);

			// Savgol-ing the values
			std::vector<double> savgoled;

			Mla::Filters::Savgol(savgoled, hand_lin_speed, seg_info.savgol_polynom_order, seg_info.savgol_window_size);

			// We get the indexes of : beginning of the throw, maximum value and end of the throw
			throw_idx.push_back(getLocalMinimumFromMaximum(savgoled, -1));
			throw_idx.push_back(Mla::Utility::getMaxValue(savgoled).second);
			throw_idx.push_back(getLocalMinimumFromMaximum(savgoled, 1));
		}

		/** TODO: doc
		
		*/
		void BegMaxEndAcceleration (Motion* motion, SegmentationInformation& seg_info, const std::string& joint_to_segment, std::vector<std::map<std::string, glm::dvec3>>& speed_difference) {
			std::vector<int> throw_idx;
			getBegEndIndexes(motion, seg_info, joint_to_segment, throw_idx);

			SpeedData speed_data(motion->getFrames().size() - 1,
				motion->getFrameTime(),
				motion->getFrames().size());

			motionSpeedComputing(motion, speed_data);

			std::map<std::string, glm::dvec3> diff_beg_max;
			std::map<std::string, glm::dvec3> diff_max_end;
			glm::dvec3 diff = glm::dvec3();

			// For each joint, we're going to populate the vector of map
			for (unsigned int j = 0; j < motion->getFrame(0)->getJoints().size(); ++j) {
				// v2 - v1
				diff = speed_data.getJointSpeed(motion->getFrame(0)->getJoint(j)->getName(), throw_idx[1]) -
					   speed_data.getJointSpeed(motion->getFrame(0)->getJoint(j)->getName(), throw_idx[0]);
				diff_beg_max.insert(std::pair<std::string, glm::dvec3>(motion->getFrame(0)->getJoint(j)->getName(), diff));

				// v2 - v1
				diff = speed_data.getJointSpeed(motion->getFrame(0)->getJoint(j)->getName(), throw_idx[2]) -
					   speed_data.getJointSpeed(motion->getFrame(0)->getJoint(j)->getName(), throw_idx[1]);
				diff_max_end.insert(std::pair<std::string, glm::dvec3>(motion->getFrame(0)->getJoint(j)->getName(), diff));
			}

			speed_difference.push_back(diff_beg_max);
			speed_difference.push_back(diff_max_end);
		}
		
		/** TODO: doc
		
		*/
		void BegMaxEndSpeed (Motion* motion, SegmentationInformation& seg_info, const std::string& joint_to_segment, std::vector<std::map<std::string, glm::dvec3>>& speed_values) {
			std::vector<int> throw_idx;
			getBegEndIndexes(motion, seg_info, joint_to_segment, throw_idx);
			
			SpeedData speed_data(motion->getFrames().size() - 1,
				motion->getFrameTime(),
				motion->getFrames().size());

			motionSpeedComputing(motion, speed_data);

			std::map<std::string, glm::dvec3> speed_beg;
			std::map<std::string, glm::dvec3> speed_max;
			std::map<std::string, glm::dvec3> speed_end;

			// For each joint, we're going to populate the vector of map
			for (unsigned int j = 0; j < motion->getFrame(0)->getJoints().size(); ++j) {
				speed_beg.insert(std::pair<std::string, glm::dvec3>(motion->getFrame(0)->getJoint(j)->getName(), speed_data.getJointSpeed(motion->getFrame(0)->getJoint(j)->getName(), throw_idx[0])));
				speed_max.insert(std::pair<std::string, glm::dvec3>(motion->getFrame(0)->getJoint(j)->getName(), speed_data.getJointSpeed(motion->getFrame(0)->getJoint(j)->getName(), throw_idx[1])));
				speed_end.insert(std::pair<std::string, glm::dvec3>(motion->getFrame(0)->getJoint(j)->getName(), speed_data.getJointSpeed(motion->getFrame(0)->getJoint(j)->getName(), throw_idx[2])));
			}

			speed_values.push_back(speed_beg);
			speed_values.push_back(speed_max);
			speed_values.push_back(speed_end);
		}

		/** TODO: doc
		
		*/
		void ThrowDuration (Motion* motion, SegmentationInformation& seg_info, const std::string& joint_to_segment) {
			SpeedData speed_data(motion->getFrames().size() - 1,
				motion->getFrameTime(),
				motion->getFrames().size());

			motionSpeedComputing(motion, speed_data);

			std::vector<double> hand_lin_speed;
			speed_data.getMeanSpeedValues(hand_lin_speed, joint_to_segment);

			// Savgol-ing the values
			std::vector<double> savgoled;

			Mla::Filters::Savgol(savgoled, hand_lin_speed, seg_info.savgol_polynom_order, seg_info.savgol_window_size);

			// We get the indexes of : beginning of the throw, maximum value and end of the throw
			std::vector<unsigned int> throw_idx;
			throw_idx.push_back(getLocalMinimumFromMaximum(savgoled, -1));
			throw_idx.push_back(Mla::Utility::getMaxValue(savgoled).second);
			throw_idx.push_back(getLocalMinimumFromMaximum(savgoled, 1));

			seg_info.throw_idx = std::pair<int, int>(throw_idx[0], throw_idx[2]);
		}

		/** Compute the speed of the joints for a whole motion.

		@param motion the motion
		*/
		void motionSpeedComputing (Motion* motion, SpeedData& speed_data) {
			std::map<std::string, glm::dvec3> lin_speed;

			for (unsigned int i = 0; i < motion->getFrames().size() - 1; i++) {
				lin_speed.clear();
				Mla::MotionOperation::jointsLinearSpeed(lin_speed, motion->getFrame(i), motion->getFrame(i + 1), motion->getFrameTime());
				speed_data.addFrameSpeed(lin_speed, i * speed_data.getIntervalTime());
			}
		}

		/** Compute the acceleration from a SpeedData class. The axis/normalisation is dependant on the way SpeedData has been computed.
			
			@param speed_data The speed data used to compute the acceleration
			@param acc_vector The returned acceleration vector
		
		*/
		void motionAccelerationComputing (SpeedData& speed_data, std::vector<std::map<std::string, glm::dvec3>>& acc_vector, bool normalise) {
			if (!speed_data.isEmpty()) {
				// I cannot think of an elegant way to do it for now
				std::vector<std::map<std::string, glm::dvec3>> lin_speed_values;
				speed_data.getAllValues(lin_speed_values);
				std::map<std::string, glm::dvec3> acc_map;
				
				glm::dvec3 acc_val = glm::dvec3();

				// For every speed info -1 (since we're doing (v2 - v1) / time)
				for (unsigned int i = 0; i < lin_speed_values.size() - 1; ++i) {
					
					acc_map.clear();
					
					// For each joint (C++11 for each)
					for (auto& kv : lin_speed_values[i]) {
						// (v2 - v1) / time
						// (it works because GLM allows us to do that)
						// /!\ We use the SpeedData time here, as it may not be the same
						// as the initial motion's one
						acc_val = (lin_speed_values[i + 1][kv.first] - kv.second) / speed_data.getIntervalTime();
						
						if (normalise)
							acc_val = glm::normalize(acc_val);

						acc_map.insert(std::pair<std::string, glm::dvec3>(kv.first, acc_val));
					}

					acc_vector.push_back(acc_map);
				}
			}

			else {
				std::cout << "ERROR: The SpeedData class has no speed information." << std::endl;
			}
		}

		/** Compute the speed data for all segments of a motion.

		@param motion_segments The segmented motion
		@param speed_data_vector The returned speed data corresponding to the motion's segments
		
		*/
		void ComputeSpeedData (std::vector<Motion*>& motion_segments, std::vector<SpeedData>& speed_data_vector) {
			
			speed_data_vector.clear();

			for (auto it = motion_segments.begin(); it != motion_segments.end(); it++) {
				SpeedData speed_data((*it)->getFrames().size() - 1,
									 (*it)->getFrameTime(),
									 (*it)->getFrames().size());

				motionSpeedComputing((*it), speed_data);

				speed_data_vector.push_back(speed_data);
			}
		}

		/** Filter a motion, by eliminating position variations.

		@param motion the motion to be filtered
		*/
		void motionFiltering (Motion* motion) {
			// j = 1 because the first frame is the reference
			for (unsigned int i = 0; i < motion->getFrames().size(); i++) {
				// j = 1 because the root actually has position information
				for (unsigned int j = 0; j < motion->getFrame(i)->getJoints().size(); j++) {
					// if the joint has no parent = root
					if (motion->getFrame(i)->getJoint(j)->getParent() != nullptr)
						motion->getFrame(i)->getJoint(j)->setPositions(glm::dvec3(motion->getOffsetFrame()->getJoint(j)->getPositions()));
				}
			}
		}

	} // namespace MotionOperation
} // namespace Mla