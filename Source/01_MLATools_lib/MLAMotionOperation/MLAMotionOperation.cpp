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
		@param frame_time the interframe time

		@return speedVector the linear speed of joints
		*/
		void jointsLinearSpeed (std::map<std::string, double>& lin_speed_vector, Frame* f1, Frame* f2, double frame_time) {
			
			Frame* global_frame_1 = f1->duplicateFrame();
			Frame* global_frame_2 = f2->duplicateFrame();

			getGlobalCoordinates(f1, global_frame_1, f1->getJoint("Hips"), glm::dmat4(1.0));
			getGlobalCoordinates(f2, global_frame_2, f2->getJoint("Hips"), glm::dmat4(1.0));
				
			double linear_speed = 0;

			for (unsigned int j = 0; j < global_frame_1->getJoints().size(); j++) {
				glm::dvec3 vec1 = global_frame_1->getJoint(j)->getPositions();
				glm::dvec3 vec2 = global_frame_2->getJoint(j)->getPositions();

				// dx / dt -> cm / s
				// dx / (dt * 100) -> m / s
				linear_speed = glm::distance(vec1, vec2) / (2.0 * frame_time * 100.0);
				lin_speed_vector.insert(std::pair<std::string, double>(global_frame_1->getJoint(j)->getName(), linear_speed));
			}

			delete global_frame_1;
			delete global_frame_2;

		}

		/** Compute the speed of the joints between 2 frames.

		@param f1 the first frame
		@param f2 the second frame
		@param frame_time the interframe time

		@return speedVector the linear speed of joints as a 3d vector
		*/
		void jointsLinearSpeed (std::map<std::string, glm::dvec3>& lin_speed_vector, Frame* f1, Frame* f2, double frame_time, bool normalise) {
			Frame* global_frame_1 = f1->duplicateFrame();
			Frame* global_frame_2 = f2->duplicateFrame();

			getGlobalCoordinates(f1, global_frame_1, f1->getJoint("Hips"), glm::dmat4(1.0));
			getGlobalCoordinates(f2, global_frame_2, f2->getJoint("Hips"), glm::dmat4(1.0));

			glm::dvec3 linear_speed = glm::dvec3();

			//TODO: iterator
			for (unsigned int j = 0; j < global_frame_1->getJoints().size(); j++) {
				glm::dvec3 vec1 = global_frame_1->getJoint(j)->getPositions();
				glm::dvec3 vec2 = global_frame_2->getJoint(j)->getPositions();

				// dx / dt -> cm / s
				// dx / (dt * 100) -> m / s
				linear_speed.x = (vec2.x - vec1.x) / (2.0 * frame_time * 100.0);
				linear_speed.y = (vec2.y - vec1.y) / (2.0 * frame_time * 100.0);
				linear_speed.z = (vec2.z - vec1.z) / (2.0 * frame_time * 100.0);

				if (normalise) {
					double norm = sqrt(pow(linear_speed.x, 2) + pow(linear_speed.y, 2) + pow(linear_speed.z, 2));
					if (norm != 0) {
						linear_speed.x = linear_speed.x / norm;
						linear_speed.y = linear_speed.y / norm;
						linear_speed.z = linear_speed.z / norm;
					}
				}

				lin_speed_vector.insert(std::pair<std::string, glm::dvec3>(global_frame_1->getJoint(j)->getName(), linear_speed));
			}

			delete global_frame_1;
			delete global_frame_2;
		}

		/** Compute the speed (along an axis) of the joints between 2 frames.

		@param f1 the first frame
		@param f2 the second frame
		@param frame_time the interframe time

		@return speedVector the linear speed of joints
		*/
		void jointsLinearSpeedAxis (std::map<std::string, double>& lin_speed_vector, Frame* f1, Frame* f2, double frame_time, const std::string& axis, bool normalise) {

			Frame* global_frame_1 = f1->duplicateFrame();
			Frame* global_frame_2 = f2->duplicateFrame();

			getGlobalCoordinates(f1, global_frame_1, f1->getJoint("Hips"), glm::dmat4(1.0));
			getGlobalCoordinates(f2, global_frame_2, f2->getJoint("Hips"), glm::dmat4(1.0));

			double linear_speed = 0;

			for (unsigned int j = 0; j < global_frame_1->getJoints().size(); j++) {
				glm::dvec3 vec1 = global_frame_1->getJoint(j)->getPositions();
				glm::dvec3 vec2 = global_frame_2->getJoint(j)->getPositions();
				
				double norm = glm::distance(vec1, vec2);
				// dx / dt -> cm / s
				// dx / (dt * 100) -> m / s
				if (axis == "x")
					linear_speed = (vec2.x - vec1.x) / (2.0 * frame_time * 100.0);
				else if (axis == "y")
					linear_speed = (vec2.y - vec1.y) / (2.0 * frame_time * 100.0);
				else if (axis == "z")
					linear_speed = (vec2.z - vec1.z) / (2.0 * frame_time * 100.0);

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
		
		/*      */
		void jointsLinearAcc(std::map<std::string, double>& lin_acc_vector, Frame* f1, Frame* f2, Frame* f3, double frame_time) {
			Frame* global_frame_1 = f1->duplicateFrame();
			Frame* global_frame_3 = f3->duplicateFrame();
			Frame* global_frame_2 = f2->duplicateFrame();

			getGlobalCoordinates(f1, global_frame_1, f1->getJoint("Hips"), glm::dmat4(1.0));
			getGlobalCoordinates(f2, global_frame_2, f2->getJoint("Hips"), glm::dmat4(1.0));
			getGlobalCoordinates(f3, global_frame_3, f2->getJoint("Hips"), glm::dmat4(1.0));

			double linear_acc = 0.0;
			glm::dvec3 lacc_vec;

			//TODO: iterator
			for (unsigned int j = 0; j < global_frame_1->getJoints().size(); j++) {
				lacc_vec = glm::dvec3(0.0, 0.0, 0.0);

				glm::dvec3 vec1 = global_frame_1->getJoint(j)->getPositions();
				glm::dvec3 vec2 = global_frame_2->getJoint(j)->getPositions();
				glm::dvec3 vec3 = global_frame_3->getJoint(j)->getPositions();

				// dv / dt -> cm / s
				// dv / (dt * 100) -> m / s^(-2)
				lacc_vec.x = (vec3.x - (2.0 * vec2.x) + vec1.x) / (frame_time * frame_time * 100.0);
				lacc_vec.y = (vec3.y - (2.0 * vec2.y) + vec1.y) / (frame_time * frame_time * 100.0);
				lacc_vec.z = (vec3.z - (2.0 * vec2.z) + vec1.z) / (frame_time * frame_time * 100.0);
				linear_acc = sqrt(pow(lacc_vec.x, 2) + pow(lacc_vec.y, 2) + pow(lacc_vec.z, 2));
				lin_acc_vector.insert(std::pair<std::string, double>(global_frame_1->getJoint(j)->getName(), linear_acc));
			}

			delete global_frame_1;
			delete global_frame_2;
		}

		/*      */
		void jointsLinearAcc(std::map<std::string, glm::dvec3>& lin_acc_vector, Frame* f1, Frame* f2, Frame* f3, double frame_time, bool normalise) {
			// f2 should ALWAYS be != nullptr, else the problem is somewhere else.
			Frame* global_frame_1 = f1->duplicateFrame();
			Frame* global_frame_2 = f2->duplicateFrame();
			Frame* global_frame_3 = f3->duplicateFrame();

			getGlobalCoordinates(f1, global_frame_1, f1->getJoint("Hips"), glm::dmat4(1.0));
			getGlobalCoordinates(f2, global_frame_2, f2->getJoint("Hips"), glm::dmat4(1.0));
			getGlobalCoordinates(f3, global_frame_3, f2->getJoint("Hips"), glm::dmat4(1.0));

			glm::dvec3 linear_acc = glm::dvec3();

			//TODO: iterator
			for (unsigned int j = 0; j < global_frame_1->getJoints().size(); j++) {
				glm::dvec3 vec1 = global_frame_1->getJoint(j)->getPositions();
				glm::dvec3 vec2 = global_frame_2->getJoint(j)->getPositions();
				glm::dvec3 vec3 = global_frame_3->getJoint(j)->getPositions();

				// dv / dt -> cm / s
				// dv / (dt * 100) -> m / s^(-2)
				linear_acc.x = (vec3.x - (2.0 * vec2.x) + vec1.x) / (frame_time * frame_time * 100.0);
				linear_acc.y = (vec3.y - (2.0 * vec2.y) + vec1.y) / (frame_time * frame_time * 100.0);
				linear_acc.z = (vec3.z - (2.0 * vec2.z) + vec1.z) / (frame_time * frame_time * 100.0);

				if (normalise) {
					double norm = sqrt(pow(linear_acc.x, 2) + pow(linear_acc.y, 2) + pow(linear_acc.z, 2));
					if (norm != 0) {
						linear_acc.x = linear_acc.x / norm;
						linear_acc.y = linear_acc.y / norm;
						linear_acc.z = linear_acc.z / norm;
					}
				}

				lin_acc_vector.insert(std::pair<std::string, glm::dvec3>(global_frame_1->getJoint(j)->getName(), linear_acc));
			}

			delete global_frame_1;
			delete global_frame_2;
		}

		/*      */
		void jointLinearAccAxis(std::map<std::string, double>& lin_acc_vector, Frame* f1, Frame* f2, Frame* f3, double frame_time, const std::string& axis, bool normalise) {

			Frame* global_frame_1 = f1->duplicateFrame();
			Frame* global_frame_2 = f2->duplicateFrame();
			Frame* global_frame_3 = f3->duplicateFrame();

			getGlobalCoordinates(f1, global_frame_1, f1->getJoint("Hips"), glm::dmat4(1.0));
			getGlobalCoordinates(f2, global_frame_2, f2->getJoint("Hips"), glm::dmat4(1.0));
			getGlobalCoordinates(f3, global_frame_3, f3->getJoint("Hips"), glm::dmat4(1.0));

			double linear_acc = 0.0;
			glm::dvec3 lacc_vec;

			for (unsigned int j = 0; j < global_frame_1->getJoints().size(); j++) {
				if (normalise)
					lacc_vec = glm::dvec3(0.0, 0.0, 0.0);

				glm::dvec3 vec1 = global_frame_1->getJoint(j)->getPositions();
				glm::dvec3 vec2 = global_frame_2->getJoint(j)->getPositions();
				glm::dvec3 vec3 = global_frame_3->getJoint(j)->getPositions();

				if (axis == "x")
					linear_acc = (vec3.x - (2.0 * vec2.x) + vec1.x) / (frame_time * frame_time * 100.0);
				else if (axis == "y")
					linear_acc = (vec3.y - (2.0 * vec2.y) + vec1.y) / (frame_time * frame_time * 100.0);
				else if (axis == "z")
					linear_acc = (vec3.z - (2.0 * vec2.z) + vec1.z) / (frame_time * frame_time * 100.0);

				if (normalise) {
					lacc_vec.x = (vec3.x - (2.0 * vec2.x) + vec1.x) / (frame_time * frame_time * 100.0);
					lacc_vec.y = (vec3.y - (2.0 * vec2.y) + vec1.y) / (frame_time * frame_time * 100.0);
					lacc_vec.z = (vec3.z - (2.0 * vec2.z) + vec1.z) / (frame_time * frame_time * 100.0);
					linear_acc = linear_acc / sqrt(pow(lacc_vec.x, 2) + pow(lacc_vec.y, 2) + pow(lacc_vec.z, 2));
				}
					

				// dx / dt -> cm / s
				// dx / (dt * 100) -> m / s
				lin_acc_vector.insert(std::pair<std::string, double>(global_frame_1->getJoint(j)->getName(), linear_acc));
			}

			delete global_frame_1;
			delete global_frame_2;

		}
		
		/** Return an interpolated frame from a motion, and a time

		@param motion the motion from which the frame will be interpolated
		@param time the time at which the frame will be interpolated
		@param frame_time the interframe_time from the initial motion
		
		@return new_frame the interpolated frame
		*/
		Frame* getFrameFromTime (Motion* motion, double current_time, double original_frame_time) {
			Frame* returnFrame = nullptr;

			double frame_wanted = current_time / original_frame_time;

			// If we actually want an already existing frame
			if (fabs(frame_wanted - round(frame_wanted)) < EPSILON) {
				
				if (static_cast<unsigned int>(round(frame_wanted)) == motion->getFrames().size())
					frame_wanted -= 1;
				returnFrame = motion->getFrame(static_cast<unsigned int>(round(frame_wanted)))->duplicateFrame();
				return returnFrame;
			}
				
			// If it's not, we compute the frame before, and after, etc.
			unsigned int frame_bef = static_cast<unsigned int>(frame_wanted);

			// W H Y ???
			// This is all because the time -> frame space
			// let say, 11 frames, from 0 to 5s (interframe_time = 0.5)
			// std::vector<Frame*>.size() == 11 [0 to 10]
			// t(f0) = 0s, t(f11) = 5s
			// if time = 5s,
			// frame_bef = time / frame_time = 10
			// frame_after = 11 -> ouch
			// Only happens when we want the last frame, so this workaround should work:
			// If the time / frame_time = last frame (which is last vector element + 1),
			// We make it back to the before-last vector idx, so that frame_aft = last array idx.
			// mixFactor should be = 1 in these case, which gives us the last frame
			/*if (frame_bef >= motion->getFrames().size() - 1)
				frame_bef = motion->getFrames().size() - 2;*/

			unsigned int frame_aft = frame_bef + 1;

			double mixFactor = frame_wanted - static_cast<int>(frame_wanted);

			if (mixFactor <= EPSILON)
				return (interpolateFrame(motion->getFrame(frame_bef), motion->getFrame(frame_bef), 0));

			else if (1 + EPSILON >= mixFactor && mixFactor >= 1 - EPSILON)
				return (interpolateFrame(motion->getFrame(frame_aft), motion->getFrame(frame_aft), 1));

			else
				return (interpolateFrame(motion->getFrame(frame_bef), motion->getFrame(frame_aft), mixFactor));
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
			while (last_value > data[idx]) {
				if ((idx <= 0 && direction < 0) || (idx >= data.size() && direction > 0))
					break;
				last_value = data[idx];
				idx += direction;

				if (idx == data.size())
					break;
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

		/*      */
		void FindThrowIndex (std::vector<double>& data, std::pair<int, int>& cut_time) {
			// Pair: idx begin, idx end

			// The mean above which we consider it's still part of the throw motion
			double data_mean = std::accumulate(data.begin(), data.end(), 0.0) / data.size();

			// First, we segment the throw (" most useful " part of the motion)
			cut_time.first = getLocalMinimumFromMaximum(data, -1);
			cut_time.second = getLocalMinimumFromMaximum(data, 1);

			// Used to search the next local minimum from the local maximum
			int local_max = -1;

			// Used to get a subset of the data vector
			std::vector<double> sub_vector;

			unsigned int left_idx = cut_time.first;
			unsigned int right_idx = cut_time.second;

			// Then, we add the LEFT parts until it's under the mean
			while(data[left_idx] >= data_mean) {
				
				if (left_idx == 0)
					break;

				// The left of the last cut become the right of the new cut
				right_idx = left_idx;

				// We extract the data's part which have not been processed yet
				sub_vector = std::vector<double>(data.begin(), data.begin() + right_idx);

				local_max = getLocalMaximum(sub_vector, -1);

				// If we're at the beginning of the signal, then there's nothing
				// to the left, so there's no more left cut
				if (local_max == 0) {
					left_idx = 0;
					cut_time.first = 0;
					break;
				}

				sub_vector = std::vector<double>(data.begin(), data.begin() + local_max);

				left_idx = getLocalMinimum(sub_vector, -1);

				if (left_idx == 0)
					break;
			}

			// We got the first correct left index
			// (i.e. at a local minimum and UNDER the average of the data)
			cut_time.first = left_idx;

			// Then, we add the RIGHT parts until it's under the mean
			while (data[right_idx] >= data_mean) {
				// The right of the last cut become the left of the new cut
				left_idx = right_idx;

				sub_vector = std::vector<double>(data.begin() + left_idx, data.end());

				// + cut_time.first, because it doesn't start at the beginning
				local_max = getLocalMaximum(sub_vector, +1) + left_idx;

				// If we're at the end of the signal, then there's nothing
				// to the right, so there's no more right cut
				if (local_max == static_cast<int>(sub_vector.size() + left_idx - 1)) {
					right_idx = local_max;
					cut_time.second = local_max;
					break;
				}

				sub_vector = std::vector<double>(data.begin() + local_max, data.end());

				right_idx = getLocalMinimum(sub_vector, +1) + local_max;

				if (right_idx == static_cast<int>(sub_vector.size() + cut_time.first - 1))
					break;
			}

			cut_time.second = right_idx;
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

			if (frames_number == original_motion->getFrames().size()) {
				*new_motion = Motion(*original_motion);
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
			double original_frame_time = original_motion->getFrameTime();

			// - 1
			// let 
			//	original_interframe_time = 0.008
			//  original_frame_number    = 5
			//  new_frame_number         = 20
			// frame original time: f0 = 0s, f1 = 0.008s, f2 = 0.016s, f3 = 0.024s, f4 = 0.032s
			// new_interframe_time = (original_frame_number * original_interframe_time) / (frames_number - 1)
			// f0 = 0s, f1 = 0.0016s, f2 = 0.0024s, f3 = 0.0032s, f4 = 0.004s, ... f19 = 0.032s
			double new_interval = ((original_motion->getFrames().size() - 1) * original_frame_time) / (frames_number - 1);
			new_motion->setFrameTime(new_interval);

			double current_time = 0.0;

			for (unsigned int i = 0; i < frames_number; i++) {
				frame_to_insert = getFrameFromTime(original_motion, current_time, original_frame_time);
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
			speed_data.getNorm(hand_lin_speed, joint_to_segment);

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
		void MotionThrowSegmentation(Motion* initial_motion, SegmentationInformation& seg_info, Motion* result_motion, const std::string& joint_to_segment) {
			Motion* sub_motion = nullptr;

			SpeedData speed_data(initial_motion->getFrames().size() - 1,
				initial_motion->getFrameTime(),
				initial_motion->getFrames().size());

			motionSpeedComputing(initial_motion, speed_data);

			std::vector<double> hand_lin_speed;
			speed_data.getNorm(hand_lin_speed, joint_to_segment);

			// Savgol-ing the values
			std::vector<double> savgoled;

			// hand_lin_speed -> accessor class
			Mla::Filters::Savgol(savgoled, hand_lin_speed, seg_info.savgol_polynom_order, seg_info.savgol_window_size);

			// Finding the separation indexes
			std::pair<int, int> separation_indexes;

			Mla::MotionOperation::FindThrowIndex(savgoled, separation_indexes);

			// Creating the new submotion

			sub_motion = new Motion();
			sub_motion->setName(initial_motion->getName());
			sub_motion->setFrameTime(initial_motion->getFrameTime());
			sub_motion->setOffsetFrame(initial_motion->getOffsetFrame()->duplicateFrame());


			for (int j = separation_indexes.first; j < separation_indexes.second + 1; j++) {
				sub_motion->addFrame(initial_motion->getFrame(j)->duplicateFrame());
			}

			seg_info.final_frame_number = (separation_indexes.second - separation_indexes.first) + 1;
			seg_info.final_interframe_time = sub_motion->getFrames().size() * sub_motion->getFrameTime() / static_cast<double>(seg_info.final_frame_number - 1);
			// reconstruct the motion
			Mla::MotionOperation::motionRebuilding(sub_motion, result_motion, seg_info.final_frame_number);

			delete sub_motion;
		}

		/** TODO: doc
		
		*/
		void getBegEndIndexes (Motion* motion, SegmentationInformation& seg_info, const std::string& joint_to_segment, std::vector<int>& throw_idx) {
			SpeedData speed_data(motion->getFrames().size() - 1,
				motion->getFrameTime(),
				motion->getFrames().size());

			motionSpeedComputing(motion, speed_data);

			std::vector<double> hand_lin_speed;
			speed_data.getNorm(hand_lin_speed, joint_to_segment);

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
			ComputeSavgol(speed_data, seg_info);

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

		/*       */
		void BegMaxEndSpeedThrow (Motion* motion, SegmentationInformation& seg_info, const std::string& joint_to_segment, std::vector<std::map<std::string, glm::dvec3>>& speed_values, bool normalise) {
			// Check if maximum is to the left or to the right
			// If it 's the case, redo the algorithm on the extracted part
			// With the 2nd maximum value

			speed_values.clear();
			
			std::pair<int, int> throw_idx;

			SpeedData speed_data(motion->getFrames().size() - 1,
				motion->getFrameTime(),
				motion->getFrames().size());

			motionSpeedComputing(motion, speed_data);
			ComputeSavgol(speed_data, seg_info);

			std::vector<double> speed_norm;
			speed_data.getNorm(speed_norm, joint_to_segment);
			FindThrowIndex(speed_norm, throw_idx);
			int max_idx = Mla::Utility::getMaxValue(speed_norm).second;

			std::vector<std::map<std::string, glm::dvec3>> normalised_val;

			speed_data.getAllValues(normalised_val, normalise);

			std::map<std::string, glm::dvec3> speed_beg;
			std::map<std::string, glm::dvec3> speed_max;
			std::map<std::string, glm::dvec3> speed_end;

			// For each joint, we're going to populate the vector of map
			for (unsigned int j = 0; j < motion->getFrame(0)->getJoints().size(); ++j) {
				std::string joint_name = motion->getFrame(0)->getJoint(j)->getName();
				speed_beg.insert(std::pair<std::string, glm::dvec3>(joint_name, normalised_val[throw_idx.first][joint_name]));
				speed_max.insert(std::pair<std::string, glm::dvec3>(joint_name, normalised_val[max_idx][joint_name]));
				speed_end.insert(std::pair<std::string, glm::dvec3>(joint_name, normalised_val[throw_idx.second][joint_name]));
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
			speed_data.getNorm(hand_lin_speed, joint_to_segment);

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

			// v(t) = ( v(t+1) - v(t-1) ) / 2dt
			for (unsigned int i = 0; i < motion->getFrames().size(); i++) {
				lin_speed.clear();
				if (i == 0)
					Mla::MotionOperation::jointsLinearSpeed(lin_speed, motion->getFrame(i), motion->getFrame(i + 1), motion->getFrameTime());
				else if (i == motion->getFrames().size() - 1)
					Mla::MotionOperation::jointsLinearSpeed(lin_speed, motion->getFrame(i - 1), motion->getFrame(i), motion->getFrameTime());
				else
					Mla::MotionOperation::jointsLinearSpeed(lin_speed, motion->getFrame(i - 1), motion->getFrame(i + 1), motion->getFrameTime());
				
				speed_data.addFrameSpeed(lin_speed, i * speed_data.getIntervalTime());
			}
		}

		/** Compute the acceleration from a SpeedData class. The axis/normalisation is dependant on the way SpeedData has been computed.
			
			@param speed_data The speed data used to compute the acceleration
			@param acc_vector The returned acceleration vector
		
		*/
		void motionAccelerationComputing (Motion* motion, AccData& acc_data) {
			std::map<std::string, glm::dvec3> lin_acc;

			// a(t) = ( x(t+1) - 2 x(t)  + x(t-1) ) / dt²
			for (unsigned int i = 0; i < motion->getFrames().size(); i++) {
				lin_acc.clear();
				if (i == 0)
					Mla::MotionOperation::jointsLinearAcc(lin_acc, motion->getFrame(i), motion->getFrame(i), motion->getFrame(i + 1), motion->getFrameTime());
				else if (i == motion->getFrames().size() - 1)
					Mla::MotionOperation::jointsLinearAcc(lin_acc, motion->getFrame(i - 1), motion->getFrame(i), motion->getFrame(i), motion->getFrameTime());
				else
					Mla::MotionOperation::jointsLinearAcc(lin_acc, motion->getFrame(i - 1), motion->getFrame(i), motion->getFrame(i + 1), motion->getFrameTime());

				acc_data.addFrameAcc(lin_acc, i * acc_data.getIntervalTime());
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

		/*      */
		void ComputeAccData (std::vector<Motion*>& motion_segments, std::vector<AccData>& acc_data_vector) {

			acc_data_vector.clear();

			for (auto it = motion_segments.begin(); it != motion_segments.end(); it++) {
				AccData acc_data((*it)->getFrames().size() - 1,
					(*it)->getFrameTime(),
					(*it)->getFrames().size());

				motionAccelerationComputing((*it), acc_data);

				acc_data_vector.push_back(acc_data);
			}
		}
		
		/** Compute a Savgol for a full motion speed values. Note that this is NOT reversible.
			TODO: un-optimised af, do it
			
			@param speed_data The speed_data
			@param data The savgol informations
		
		*/
		void ComputeSavgol (SpeedData& speed_data, SegmentationInformation& seg_info) {
			
			// This vector will contain the extracted speed values (for 1 joint)
			// from the SpeedData class
			std::vector<glm::dvec3> speed_3d_vector;

			// This vector will contain the savgoled speed values (for 1 joint)
			std::vector<glm::dvec3> savgoled_speed;
			
			// This vector will store the speed values for 1 component
			std::vector<double> tmp_speed_component;
			// This vector will store the savgoled values for 1 component
			std::vector<double> tmp_savgol_component;

			std::vector<std::string> joint_names = speed_data.getJointNames();

			// For each joint
			for (auto it_j = joint_names.begin(); it_j != joint_names.end(); ++it_j) {
				// We get the speed vector
				speed_data.getAllValues(speed_3d_vector, *it_j);

				// We savgol each component of the 3d vector
				for (auto axis = 0; axis < 3; ++axis) {
					// We extract x, y, then z
					Mla::Utility::ExtractComponent(speed_3d_vector, tmp_speed_component, axis);
					// We savgol it
					Mla::Filters::Savgol(tmp_savgol_component, tmp_speed_component, seg_info.savgol_polynom_order, seg_info.savgol_window_size);

					// And now we put it back into the speed_data
					speed_data.setSpeedSet(tmp_savgol_component, *it_j, axis);
				}

				speed_data.getNorm(tmp_speed_component, *it_j);
				Mla::Filters::Savgol(tmp_savgol_component, tmp_speed_component, seg_info.savgol_polynom_order, seg_info.savgol_window_size);
				speed_data.setNormSet(tmp_savgol_component, *it_j);
			}

		}

		/*      */
		void ComputeSavgol(AccData& acc_data, SegmentationInformation& seg_info) {

			// This vector will contain the extracted speed values (for 1 joint)
			// from the SpeedData class
			std::vector<glm::dvec3> acc_3d_vector;

			// This vector will contain the savgoled speed values (for 1 joint)
			std::vector<glm::dvec3> savgoled_acc;

			// This vector will store the speed values for 1 component
			std::vector<double> tmp_acc_component;
			// This vector will store the savgoled values for 1 component
			std::vector<double> tmp_savgol_component;

			std::vector<std::string> joint_names = acc_data.getJointNames();

			// For each joint
			for (auto it_j = joint_names.begin(); it_j != joint_names.end(); ++it_j) {
				// We get the speed vector
				acc_data.getAllValues(acc_3d_vector, *it_j);

				// We savgol each component of the 3d vector
				for (auto axis = 0; axis < 3; ++axis) {
					// We extract x, y, then z
					Mla::Utility::ExtractComponent(acc_3d_vector, tmp_acc_component, axis);
					// We savgol it
					Mla::Filters::Savgol(tmp_savgol_component, tmp_acc_component, seg_info.savgol_polynom_order, seg_info.savgol_window_size);

					// And now we put it back into the speed_data
					acc_data.setAccSet(tmp_savgol_component, *it_j, axis);
				}

				acc_data.getNorm(tmp_acc_component, *it_j);
				Mla::Filters::Savgol(tmp_savgol_component, tmp_acc_component, seg_info.savgol_polynom_order, seg_info.savgol_window_size);
				acc_data.setNormSet(tmp_savgol_component, *it_j);
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