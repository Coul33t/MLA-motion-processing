/**
* \file MlaMotionOperation.cpp
* \brief This file contains all the motion operations function, such as filtering, descriptors extraction, etc.
* \author Quentin.C
* \version 0.1
* \date 12 February 2019
*/

#include "MLAMotionOperation.h"

namespace Mla {
	namespace MotionOperation {

		/** Interpolate a joint position and orientation from two original joints and a mix factor.
		* @param j1 First joint
		* @param j2 Second joint
		* @param mixFactor Mix factor between the 2 joints
		* @return An interpolated joint
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

		/** Interpolate a frame from two original frames and a mix factor.
		* @param f1 First frame
		* @param f2 Second frame
		* @param mixFactor Mix factor between the 2 frames
		* @return An interpolated frame
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

		/** Returns the positions of the joints in a specific frame.
		* @param pos_vector Output vector
		* @param f1 Desired frame
		* @param abs_or_rel Absolute or relative coordinates
		* @return A vector containing the positions of all the joints
		*/
		void jointsPositions(std::map<std::string, glm::dvec3>& pos_vector, Frame* f1, std::vector<std::string>& KC) {
			// I choose to put this here instead of a simple " else " at the end for code
			// readability and avoid unnecessary initilisation and/or code duplication.

			Frame* global_frame_1 = f1->duplicateFrame();
			getGlobalCoordinates(f1, global_frame_1, f1->getJoint("Hips"), glm::dmat4(1.0));

			glm::dvec3 vec_pos;

			double n_coef = 0.0;
			glm::dvec3 current_root = global_frame_1->getJoint(KC.front())->getPositions();
			glm::dvec3 extremity = global_frame_1->getJoint(KC.back())->getPositions();
			double dRE = glm::distance(current_root, extremity);
			double sumKC = 0;
			for (unsigned int i = 0; i < KC.size() - 1; i++) {
				sumKC += glm::distance(global_frame_1->getJoint(KC[i])->getPositions(),
					global_frame_1->getJoint(KC[i + 1])->getPositions());
			}
			n_coef = dRE / pow(sumKC, 2);

			for (unsigned int j = 0; j < f1->getJoints().size(); j++) {
				vec_pos = global_frame_1->getJoint(j)->getPositions();
				vec_pos[0] = vec_pos[0] * n_coef;
				vec_pos[1] = vec_pos[1] * n_coef;
				vec_pos[2] = vec_pos[2] * n_coef;
				pos_vector.insert(std::pair<std::string, glm::dvec3>(global_frame_1->getJoint(j)->getName(), vec_pos));
			}
		}

		void jointsPositions(std::map<std::string, glm::dvec3>& pos_vector, Frame* f1, const std::string& abs_or_rel) {
			// I choose to put this here instead of a simple " else " at the end for code
			// readability and avoid unnecessary initilisation and/or code duplication.
			if (abs_or_rel != "abs" && abs_or_rel != "rel") {
				std::cout << "ERROR: position must be absolute (abs) or relative (rel)." << std::endl;
				return;
			}

			glm::dvec3 vec_pos;

			if (abs_or_rel == "abs") {
				Frame* global_frame_1 = f1->duplicateFrame();
				getGlobalCoordinates(f1, global_frame_1, f1->getJoint("Hips"), glm::dmat4(1.0));

				for (unsigned int j = 0; j < global_frame_1->getJoints().size(); j++) {
					vec_pos = global_frame_1->getJoint(j)->getPositions();
					pos_vector.insert(std::pair<std::string, glm::dvec3>(global_frame_1->getJoint(j)->getName(), vec_pos));
				}

				delete global_frame_1;
			}

			else if (abs_or_rel == "rel") {
				for (unsigned int j = 0; j < f1->getJoints().size(); j++) {
					vec_pos = f1->getJoint(j)->getPositions();
					pos_vector.insert(std::pair<std::string, glm::dvec3>(f1->getJoint(j)->getName(), vec_pos));
				}
			}
		}
		
		/** Returns the positions of the joints in a specific frame along one axis.
		* @param pos_vector Output vector
		* @param f1 Desired frame
		* @param abs_or_rel Absolute or relative coordinates
		* @param axis Axis along which the positions values will be returned
		* @return A vector containing the positions of all the joints
		*/
		void jointsPositionsAxis(std::map<std::string, double>& pos_vector, Frame* f1, const std::string& abs_or_rel, const std::string& axis) {
			// I choose to put this here instead of a simple " else " at the end for code
			// readability and avoid unnecessary initilisation and/or code duplication.
			if (abs_or_rel != "abs" && abs_or_rel != "rel") {
				std::cout << "ERROR: position must be absolute (abs) or relative (rel)." << std::endl;
				return;
			}

			glm::dvec3 vec_pos;

			if (abs_or_rel == "abs") {
				Frame* global_frame_1 = f1->duplicateFrame();
				getGlobalCoordinates(f1, global_frame_1, f1->getJoint("Hips"), glm::dmat4(1.0));

				for (unsigned int j = 0; j < global_frame_1->getJoints().size(); j++) {
					vec_pos = global_frame_1->getJoint(j)->getPositions();
					if (axis == "x")
						pos_vector.insert(std::pair<std::string, double>(global_frame_1->getJoint(j)->getName(), vec_pos[0]));
					else if (axis == "y")
						pos_vector.insert(std::pair<std::string, double>(global_frame_1->getJoint(j)->getName(), vec_pos[1]));
					else if (axis == "z")
						pos_vector.insert(std::pair<std::string, double>(global_frame_1->getJoint(j)->getName(), vec_pos[2]));
				}

				delete global_frame_1;
			}

			else if (abs_or_rel == "rel") {
				for (unsigned int j = 0; j < f1->getJoints().size(); j++) {
					vec_pos = f1->getJoint(j)->getPositions();
					if (axis == "x")
						pos_vector.insert(std::pair<std::string, double>(f1->getJoint(j)->getName(), vec_pos[0]));
					else if (axis == "y")
						pos_vector.insert(std::pair<std::string, double>(f1->getJoint(j)->getName(), vec_pos[1]));
					else if (axis == "z")
						pos_vector.insert(std::pair<std::string, double>(f1->getJoint(j)->getName(), vec_pos[2]));
				}
			}
		}


		/** Compute the speed of the joints at a given point in time (t).
		* @param lin_speed_vector Output vector
		* @param f1 First frame (t-1)
		* @param f2 Second frame (t+1)
		* @param frame_time Interframe time
		* @return A vector containing the linear speed of all the joints
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

		/** Compute the normalised (or not) speed of the joints at a given point in time (t).
		* @param lin_speed_vector Output vector
		* @param f1 First frame (t-1)
		* @param f2 Second frame (t+1)
		* @param frame_time Interframe time
		* @return A vector containing the linear speed of the joints
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

		/** Compute the normalised (or not) speed of the joints at a given point in time (t) along one axis.
		* @param lin_speed_vector Output vector
		* @param f1 First frame (t-1)
		* @param f2 Second frame (t+1)
		* @param frame_time Interframe time
		* @param axis Axis along which the linear speed will be computed
		* @return A vector containing the linear speed of all the joints
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

		/** Compute the angular speed of the joints at a given point in time (t).
		* @param ang_speed_vector Output vector
		* @param f1 First frame (t-1)
		* @param f2 Second frame (t+1)
		* @param frame_time Interframe time
		* @return A vector containing the angular speed of all the joints
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
		@param mean_lin_speed_inter Output vector
		@param motion Motion
		@param n Number of interval
		@return A vector containing the mean speed for each joint on the frames, for each interval
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

		/** Compute the mean speed on given intervals for the full motion between two frames, with skeleton interpolation.
		* @param mean_lin_speed_inter Output vector
		* @param motion Motion
		* @param n Number of interval
		* @param axis Axis along which the mean speed will be computed
		* @return A vector containing the mean speed (on the 3 axis) for each joint on the frames, for each interval
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
		
		/* Compute the acceleration of the joints at a given point in time (t).
		* @param lin_acc_vector Output vector
		* @param f1 First frame (t-1)
		* @param f2 Second frame (t)
		* @param f3 Third frame (t+1)
		* @param frame_time Interframe time
		* @return A vector containing the acceleration
		*/
		void jointsLinearAcc(std::map<std::string, double>& lin_acc_vector, Frame* f1, Frame* f2, Frame* f3, double frame_time) {
			Frame* global_frame_1 = f1->duplicateFrame();
			Frame* global_frame_2 = f2->duplicateFrame();
			Frame* global_frame_3 = f3->duplicateFrame();

			getGlobalCoordinates(f1, global_frame_1, f1->getJoint("Hips"), glm::dmat4(1.0));
			getGlobalCoordinates(f2, global_frame_2, f2->getJoint("Hips"), glm::dmat4(1.0));
			getGlobalCoordinates(f3, global_frame_3, f3->getJoint("Hips"), glm::dmat4(1.0));

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
				lacc_vec.x = (vec3.x - (2.0 * vec2.x) + vec1.x) / (pow(frame_time, 2) * 100.0);
				lacc_vec.y = (vec3.y - (2.0 * vec2.y) + vec1.y) / (pow(frame_time, 2) * 100.0);
				lacc_vec.z = (vec3.z - (2.0 * vec2.z) + vec1.z) / (pow(frame_time, 2) * 100.0);
				linear_acc = sqrt(pow(lacc_vec.x, 2) + pow(lacc_vec.y, 2) + pow(lacc_vec.z, 2));
				lin_acc_vector.insert(std::pair<std::string, double>(global_frame_1->getJoint(j)->getName(), linear_acc));
			}

			delete global_frame_1;
			delete global_frame_2;
			delete global_frame_3;
		}

		/* Compute the normalised (or not) acceleration of the joints at a given point in time (t).
		* @param lin_acc_vector Output vector
		* @param f1 First frame (t-1)
		* @param f2 Second frame (t)
		* @param f3 Third frame (t+1)
		* @param frame_time Interframe time
		* @param normalise Indicating if the values must be normalised or not 
		* @return A vector containing the acceleration
		*/
		void jointsLinearAcc(std::map<std::string, glm::dvec3>& lin_acc_vector, Frame* f1, Frame* f2, Frame* f3, double frame_time, bool normalise) {
			Frame* global_frame_1 = f1->duplicateFrame();
			Frame* global_frame_2 = f2->duplicateFrame();
			Frame* global_frame_3 = f3->duplicateFrame();

			getGlobalCoordinates(f1, global_frame_1, f1->getJoint("Hips"), glm::dmat4(1.0));
			getGlobalCoordinates(f2, global_frame_2, f2->getJoint("Hips"), glm::dmat4(1.0));
			getGlobalCoordinates(f3, global_frame_3, f3->getJoint("Hips"), glm::dmat4(1.0));

			glm::dvec3 linear_acc = glm::dvec3();

			//TODO: iterator
			for (unsigned int j = 0; j < global_frame_1->getJoints().size(); j++) {
				glm::dvec3 vec1 = global_frame_1->getJoint(j)->getPositions();
				glm::dvec3 vec2 = global_frame_2->getJoint(j)->getPositions();
				glm::dvec3 vec3 = global_frame_3->getJoint(j)->getPositions();

				// dv / dt -> cm / s
				// dv / (dt * 100) -> m / s^(-2)
				linear_acc.x = (vec3.x - (2.0 * vec2.x) + vec1.x) / (pow(frame_time, 2) * 100.0);
				linear_acc.y = (vec3.y - (2.0 * vec2.y) + vec1.y) / (pow(frame_time, 2) * 100.0);
				linear_acc.z = (vec3.z - (2.0 * vec2.z) + vec1.z) / (pow(frame_time, 2) * 100.0);

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
			delete global_frame_3;
		}

		/* Compute the normalised (or not) acceleration of the joints at a given point in time (t) along one axis.
		* @param lin_acc_vector Output vector
		* @param f1 First frame (t-1)
		* @param f2 Second frame (t)
		* @param f3 Third frame (t+1)
		* @param frame_time Interframe time
		* @param axis Axis along which the acceleration will be computed
		* @param normalise Indicating if the values must be normalised or not
		* @return A vector containing the acceleration
		*/
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
					linear_acc = (vec3.x - (2.0 * vec2.x) + vec1.x) / (pow(frame_time, 2) * 100.0);
				else if (axis == "y")
					linear_acc = (vec3.y - (2.0 * vec2.y) + vec1.y) / (pow(frame_time, 2) * 100.0);
				else if (axis == "z")
					linear_acc = (vec3.z - (2.0 * vec2.z) + vec1.z) / (pow(frame_time, 2) * 100.0);

				if (normalise) {
					lacc_vec.x = (vec3.x - (2.0 * vec2.x) + vec1.x) / (pow(frame_time, 2) * 100.0);
					lacc_vec.y = (vec3.y - (2.0 * vec2.y) + vec1.y) / (pow(frame_time, 2) * 100.0);
					lacc_vec.z = (vec3.z - (2.0 * vec2.z) + vec1.z) / (pow(frame_time, 2) * 100.0);
					linear_acc = linear_acc / sqrt(pow(lacc_vec.x, 2) + pow(lacc_vec.y, 2) + pow(lacc_vec.z, 2));
				}
					

				// dx / dt -> cm / s
				// dx / (dt * 100) -> m / s
				lin_acc_vector.insert(std::pair<std::string, double>(global_frame_1->getJoint(j)->getName(), linear_acc));
			}

			delete global_frame_1;
			delete global_frame_2;
			delete global_frame_3;

		}
		
		/* Compute the jerk of the joints at a given point in time (t).
		* @param jerk_vector Output vector
		* @param f1 First frame (t+2)
		* @param f2 Second frame (t+1)
		* @param f3 Third frame (t-1)
		* @param f4 Fourth frame (t-2)
		* @param frame_time Interframe time
		* @return A vector containing the jerk
		*/
		void jointsJerk(std::map<std::string, glm::vec3>& jerk_vector, Frame* f1, Frame* f2, Frame* f3, Frame* f4, double frame_time) {
			// f1 -> t+2
			// f2 -> t+1
			// f3 -> t-1
			// f4 -> t-2
			Frame* global_frame_1 = f1->duplicateFrame();
			Frame* global_frame_2 = f2->duplicateFrame();
			Frame* global_frame_3 = f3->duplicateFrame();
			Frame* global_frame_4 = f4->duplicateFrame();

			getGlobalCoordinates(f1, global_frame_1, f1->getJoint("Hips"), glm::dmat4(1.0));
			getGlobalCoordinates(f2, global_frame_2, f2->getJoint("Hips"), glm::dmat4(1.0));
			getGlobalCoordinates(f3, global_frame_3, f3->getJoint("Hips"), glm::dmat4(1.0));
			getGlobalCoordinates(f4, global_frame_4, f4->getJoint("Hips"), glm::dmat4(1.0));

			glm::dvec3 jerk = glm::dvec3();
			jerk_vector.clear();

			for (unsigned int j = 0; j < global_frame_1->getJoints().size(); j++) {
				glm::dvec3 vec1 = global_frame_1->getJoint(j)->getPositions();
				glm::dvec3 vec2 = global_frame_2->getJoint(j)->getPositions();
				glm::dvec3 vec3 = global_frame_3->getJoint(j)->getPositions();
				glm::dvec3 vec4 = global_frame_4->getJoint(j)->getPositions();

				jerk.x = (vec1.x - 2.0 * vec2.x + 2.0 * vec3.x - vec4.x) / (2.0 * pow(frame_time, 3) * 100.0);
				jerk.y = (vec1.y - 2.0 * vec2.y + 2.0 * vec3.y - vec4.y) / (2.0 * pow(frame_time, 3) * 100.0);
				jerk.z = (vec1.z - 2.0 * vec2.z + 2.0 * vec3.z - vec4.z) / (2.0 * pow(frame_time, 3) * 100.0);

				jerk_vector.insert(std::pair<std::string, glm::dvec3>(global_frame_1->getJoint(j)->getName(), jerk));
			}

			delete global_frame_1;
			delete global_frame_2;
			delete global_frame_3;
			delete global_frame_4;
		}

		/* Compute the normalised jerk of the joints at a given point in time (t).
		* @param jerk_vector Output vector
		* @param f1 First frame (t+2)
		* @param f2 Second frame (t+1)
		* @param f3 Third frame (t-1)
		* @param f4 Fourth frame (t-2)
		* @param frame_time Interframe time
		* @return A vector containing the jerk
		*/
		void jointsJerkNorm(std::map<std::string, double>& jerk_vector, Frame* f1, Frame* f2, Frame* f3, Frame* f4, double frame_time) {
			// f1 -> t+2
			// f2 -> t+1
			// f3 -> t-1
			// f4 -> t-2
			Frame* global_frame_1 = f1->duplicateFrame();
			Frame* global_frame_2 = f2->duplicateFrame();
			Frame* global_frame_3 = f3->duplicateFrame();
			Frame* global_frame_4 = f4->duplicateFrame();

			getGlobalCoordinates(f1, global_frame_1, f1->getJoint("Hips"), glm::dmat4(1.0));
			getGlobalCoordinates(f2, global_frame_2, f2->getJoint("Hips"), glm::dmat4(1.0));
			getGlobalCoordinates(f3, global_frame_3, f3->getJoint("Hips"), glm::dmat4(1.0));
			getGlobalCoordinates(f4, global_frame_4, f4->getJoint("Hips"), glm::dmat4(1.0));

			glm::dvec3 jerk = glm::dvec3();
			double jerk_norm = 0;

			jerk_vector.clear();

			for (unsigned int j = 0; j < global_frame_1->getJoints().size(); j++) {
				glm::dvec3 vec1 = global_frame_1->getJoint(j)->getPositions();
				glm::dvec3 vec2 = global_frame_2->getJoint(j)->getPositions();
				glm::dvec3 vec3 = global_frame_3->getJoint(j)->getPositions();
				glm::dvec3 vec4 = global_frame_4->getJoint(j)->getPositions();

				jerk.x = (vec1.x - 2.0 * vec2.x + 2.0 * vec3.x - vec4.x) / (2.0 * pow(frame_time, 3) * 100.0);
				jerk.y = (vec1.y - 2.0 * vec2.y + 2.0 * vec3.y - vec4.y) / (2.0 * pow(frame_time, 3) * 100.0);
				jerk.z = (vec1.z - 2.0 * vec2.z + 2.0 * vec3.z - vec4.z) / (2.0 * pow(frame_time, 3) * 100.0);

				jerk_norm = sqrt(pow(jerk.x, 2) + pow(jerk.y, 2) + pow(jerk.z, 2));
				jerk_vector.insert(std::pair<std::string, double>(global_frame_1->getJoint(j)->getName(), jerk_norm));
			}

			delete global_frame_1;
			delete global_frame_2;
			delete global_frame_3;
			delete global_frame_4;
		}

		/* Compute the normalised (or not) jerk of the joints at a given point in time (t) along one axis.
		* @param jerk_vector Output vector
		* @param f1 First frame (t+2)
		* @param f2 Second frame (t+1)
		* @param f3 Third frame (t-1)
		* @param f4 Fourth frame (t-2)
		* @param frame_time Interframe time
		* @param axis Axis along which the acceleration will be computed
		* @param normalise Indicating if the values must be normalised or not  
		* @return A vector containing the jerk
		*/
		void jointsJerkAxis(std::map<std::string, double>& jerk_vector, Frame* f1, Frame* f2, Frame* f3, Frame* f4, double frame_time, const std::string& axis, bool normalise) {
			// f1 -> t+2
			// f2 -> t+1
			// f3 -> t-1
			// f4 -> t-2
			Frame* global_frame_1 = f1->duplicateFrame();
			Frame* global_frame_2 = f2->duplicateFrame();
			Frame* global_frame_3 = f3->duplicateFrame();
			Frame* global_frame_4 = f4->duplicateFrame();

			getGlobalCoordinates(f1, global_frame_1, f1->getJoint("Hips"), glm::dmat4(1.0));
			getGlobalCoordinates(f2, global_frame_2, f2->getJoint("Hips"), glm::dmat4(1.0));
			getGlobalCoordinates(f3, global_frame_3, f3->getJoint("Hips"), glm::dmat4(1.0));
			getGlobalCoordinates(f4, global_frame_4, f4->getJoint("Hips"), glm::dmat4(1.0));

			double jerk = 0.0;
			glm::dvec3 jerk_vec;

			jerk_vector.clear();

			for (unsigned int j = 0; j < global_frame_1->getJoints().size(); j++) {

				if (normalise)
					jerk_vec = glm::dvec3(0.0, 0.0, 0.0);

				jerk = 0.0;

				glm::dvec3 vec1 = global_frame_1->getJoint(j)->getPositions();
				glm::dvec3 vec2 = global_frame_2->getJoint(j)->getPositions();
				glm::dvec3 vec3 = global_frame_3->getJoint(j)->getPositions();
				glm::dvec3 vec4 = global_frame_4->getJoint(j)->getPositions();

				if (axis == "x")
					jerk = (vec1.x - 2.0 * vec2.x + 2.0 * vec3.x - vec4.x) / (2.0 * pow(frame_time, 3) * 100.0);
				if (axis == "y")
					jerk = (vec1.y - 2.0 * vec2.y + 2.0 * vec3.y - vec4.y) / (2.0 * pow(frame_time, 3) * 100.0);
				if (axis == "z")
					jerk = (vec1.z - 2.0 * vec2.z + 2.0 * vec3.z - vec4.z) / (2.0 * pow(frame_time, 3) * 100.0);

				if (normalise) {
					jerk_vec.x = (vec1.x - 2.0 * vec2.x + 2.0 * vec3.x - vec4.x) / (2.0 * pow(frame_time, 3) * 100.0);
					jerk_vec.y = (vec1.y - 2.0 * vec2.y + 2.0 * vec3.y - vec4.y) / (2.0 * pow(frame_time, 3) * 100.0);
					jerk_vec.z = (vec1.z - 2.0 * vec2.z + 2.0 * vec3.z - vec4.z) / (2.0 * pow(frame_time, 3) * 100.0);

					double norm = sqrt(pow(jerk_vec.x, 2) + pow(jerk_vec.y, 2) + pow(jerk_vec.z, 2));
					if (norm != 0)
						jerk = jerk / norm;
				}

				jerk_vector.insert(std::pair<std::string, double>(global_frame_1->getJoint(j)->getName(), jerk));
			}

			delete global_frame_1;
			delete global_frame_2;
			delete global_frame_3;
			delete global_frame_4;
		}

		/** Compute the bounding box (-x, +x, -y, +y, -z, +z) for one frame, on a set of given joints.
		* @param bounding_box Output vector
		* @param f1 the frame on which the bounding box will be computed
		* @param joints_to_check the joints to check for the bouding box
		* @return bouding_box the vector containing the 6 values for the bounding box
		*/
		void jointsBoundingBox(std::map<std::string, std::vector<double>>& bounding_box, Frame* f1, std::vector<std::string>& joints_to_check, bool normalise) {
			Frame* global_frame_1 = f1->duplicateFrame();

			getGlobalCoordinates(f1, global_frame_1, f1->getJoint("Hips"), glm::dmat4(1.0));

			// Initialise the vector with six 0 values
			bounding_box = std::map<std::string, std::vector<double>>();
			std::string final_name = "";
			for (auto it = joints_to_check.begin(); it != joints_to_check.end(); it++) {
				final_name += (*it);
			}

			// Compute the coefficient
			// The initial formula, for a kinematic chain KC is:
			// nKC = distance(root, extremity) / sum(segments)
			// We aren't interested into the value of nKC, but the
			// coefficient by which we will multiply the coordinates
			// to normalise them. This coefficient is defined by
			// nKC/sumKC -> distance(root, extremity) / sum(segments)²
			double n_coef = 1;

			if (normalise) {
				glm::dvec3 current_root = global_frame_1->getJoint(joints_to_check.front())->getPositions();
				glm::dvec3 extremity = global_frame_1->getJoint(joints_to_check.back())->getPositions();
				double dRE = glm::distance(current_root, extremity);
				double sumKC = 0;
				for (unsigned int i = 0; i < joints_to_check.size() - 1; i++) {
					sumKC += glm::distance(global_frame_1->getJoint(joints_to_check[i])->getPositions(),
						global_frame_1->getJoint(joints_to_check[i + 1])->getPositions());
				}
				n_coef = dRE / pow(sumKC, 2);
			}

			std::vector<double> values = std::vector<double>(6, 0);
			bounding_box[final_name] = std::vector<double>(6, 0);

			for (auto j_it = joints_to_check.begin(); j_it != joints_to_check.end(); j_it++) {
				glm::dvec3 vec = global_frame_1->getJoint(*j_it)->getPositions();
				// If it's the first joint, the bounding box is its x, y, and z (bounding point ?)
				// n_coef = 1 if normalise is false (avoids code duplication and stuff)
				// - root to recenter the origin to (0,0,0)
				// root = (0,0,0) if normalise is false 
				if (j_it == joints_to_check.begin()) {
					values[0] = vec[0] * n_coef;
					values[1] = vec[0] * n_coef;
					values[2] = vec[1] * n_coef;
					values[3] = vec[1] * n_coef;
					values[4] = vec[2] * n_coef;
					values[5] = vec[2] * n_coef;
				}

				// Else we check each coordinates (-x, +x, -y, +y, -z, +z)
				else {
					if (vec[0] * n_coef < values[0])
						values[0] = vec[0] * n_coef;

					if (vec[0] * n_coef > values[1])
						values[1] = vec[0] * n_coef;

					if (vec[1] * n_coef < values[2])
						values[2] = vec[1] * n_coef;

					if (vec[1] * n_coef > values[3])
						values[3] = vec[1] * n_coef;

					if (vec[2] * n_coef < values[4])
						values[4] = vec[2] * n_coef;

					if (vec[2] * n_coef > values[5])
						values[5] = vec[2] * n_coef;
				}
			}

			bounding_box[final_name] = values;

			delete global_frame_1;
		}
		
		/** Return an interpolated frame from a motion, and a time
		* @param motion Motion from which the frame will be interpolated
		* @param time Time at which the frame will be interpolated
		* @param Interframe_timeInterframe time from the initial motion
		* @return The interpolated frame
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
		* @param local_frame Initial frame (local coordinates)
		* @param global_frame Frame containing global coordinates (intially a copy of the local frame)
		* @param current_joint Joint currently being processed (from the local_frame)
		* @param global_mat Modelview matrix (first call: identity matrix)
		* @return The frame in global coordinates
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

		/** Gives the global maximum of a serie of values.
		* @param data Vector containing the data
		* @param global_max Output pair
		* @return The value and position of the global maximum
		*/
		void getGlobalMaximum(std::vector<double>& data, std::pair<double, unsigned int>& global_max) {
			std::vector<double> sub_vector;
			global_max = Mla::Utility::getMaxValue(data);

			unsigned int temporal_threshold = unsigned int(std::round(data.size() / 10));

			if (global_max.second < temporal_threshold) {
				sub_vector = std::vector<double>(data.begin() + temporal_threshold, data.end());
				int local_min = getLocalMinimum(sub_vector, 1);
				sub_vector = std::vector<double>(sub_vector.begin() + local_min, sub_vector.end());
				global_max = Mla::Utility::getMaxValue(sub_vector);
			}
				
			if (global_max.second > data.size() - temporal_threshold) {
				sub_vector = std::vector<double>(data.begin(), data.begin() + data.size() - temporal_threshold);
				int local_min = getLocalMinimum(sub_vector, -1);
				sub_vector = std::vector<double>(sub_vector.begin(), sub_vector.begin() + sub_vector.size() - local_min);
			}
		}

		/** Find the next local minimum from the max value, if and only if this value is inferior to a threshold.
		* @param data Vector containing the data
		* @param direction Direction in which the local minimum is to be found (negative for left, positive for right)
		* @return The index at which the local minimum is found
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

			// There was a +1 in the initial code
			// TODO: why tho
			// EDIT: I think I got it (we're gonna look for an <= value, something like that)
			std::pair<double, unsigned int> global_max;
			getGlobalMaximum(data, global_max);
			double last_value = global_max.first + 1;
			unsigned int idx = global_max.second;

			if (idx == 0 && direction < 0)
				return 0;

			if (idx == data.size() && direction > 0)
				return data.size() - 1;

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

		/** Find the next local maximum in the data.
		* @param data Vector containing the data
		* @param direction Direction in which the local maximum is to be found (negative for left, positive for right)
		* @return The index at which the local minimum is found
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

		/** Find the next local minmum in the data.
		* @param data Vector containing the data
		* @param direction Direction in which the local minimum is to be found (negative for left, positive for right)
		* @return The index at which the local minimum is found
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
		
		/** Segment the motion into n segments, from one local minimum to another.
		* @param data Vector containing the data
		* @param left_cut Number of segment to the left
		* @param right_cut Number of segment to the right
		* @param interframe_time Interframe time
		* @param cut_times Output vector
		* @return A vector of pair containing the beginning and the end indexes for each segment of the motion
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

		/*  TODO:DOC    */
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

		/** Rebuild a motion from an initial one, with a new number of frames.
		* @param original_motion Motion to rebuild
		* @param new_motion Output motion
		* @param frames_number New frames count
		* @return A rebuilded motion
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
		* @param initial_motion Motion to be segmented
		* @param seg_info Structure containing all the information needed for the segmentation (see SegmentationInformation)
		* @param motion_segments Output vector
		* @param joint_to_segment Joint used to find the [mini/maxi]mums
		* @return The different segments of the motion
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

		/** Extract the the throwing part of a motion (i.e. the part going from the left minimum local to the right minimum local of the highest speed value).
		* @param initial_motion Motion to be segmented
		* @param seg_info Structure containing all the information needed for the segmentation (see SegmentationInformation)
		* @param motion_segments Output motion
		* @param joint_to_segment Joint used to find the [mini/maxi]mums
		* @return The different segments of the motion
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

		/** Get the indexes of the beginning, max speed value and end of a throw, which correspond respectively to the left local minimum, maximum speed value and right local minimum.
		* @param motion Motion from which the indexes will be retrieved
		* @param seg_info Structure containing all the information needed for the segmentation (see SegmentationInformation)
		* @param joint_to_segment Joint used to find the indexes
		* @param throw_idx Output vector
		* @return The 3 desired indexes 
		*/
		void getBegMaxEndIndexes (Motion* motion, SegmentationInformation& seg_info, const std::string& joint_to_segment, std::vector<int>& throw_idx) {
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
		
		/** Get the values of the beginning, max speed value and end of a throw, which correspond respectively to the left local minimum, maximum speed value and right local minimum.
		* @param motion Motion from which the values will be retrieved
		* @param seg_info Structure containing all the information needed for the segmentation (see SegmentationInformation)
		* @param joint_to_segment Joint used to find the values
		* @param speed_values Output vector
		* @return The 3 desired speed values
		*/
		void BegMaxEndSpeed (Motion* motion, SegmentationInformation& seg_info, const std::string& joint_to_segment, std::vector<std::map<std::string, glm::dvec3>>& speed_values) {
			std::vector<int> throw_idx;
			getBegMaxEndIndexes(motion, seg_info, joint_to_segment, throw_idx);
			
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

		/** Get the normalised (or not) values of the beginning, max speed value and end of a throw, which correspond respectively to the left local minimum, maximum speed value and right local minimum.
		* @param motion Motion from which the values will be retrieved
		* @param seg_info Structure containing all the information needed for the segmentation (see SegmentationInformation)
		* @param joint_to_segment Joint used to find the values
		* @param speed_values Output vector
		* @param normalise Indicating if the values must be normalised or not 
		* @return The 3 desired speed values
		*/
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

		/** Computes the duration of the throw.
		* @param motion Motion from which the values will be retrieved
		* @param seg_info Structure containing all the information needed for the segmentation (see SegmentationInformation)
		* @param joint_to_segment Joint used to find the values
		* @return The indexes of the beginning and the end of the throw
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
		* @param motion Initial motion
		* @param speed_data Output class (see SpeedData)
		* @return The computed speed data in a class (see SpeedData)
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
		* @param speed_data The speed data used to compute the acceleration
		* @param acc_vector Output class (see AccData)
		* @return The computed acceleration data in a class (see AccData)
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
		* @param motion_segments The segmented motion
		* @param speed_data_vector Output vector 
		* @return The speed data corresponding to the motion's segments
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

		/** Compute the acceleration data for all segments of a motion.
		* @param motion_segments The segmented motion
		* @param speed_data_vector Output vector
		* @return The acceleration data corresponding to the motion's segments
		*/
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
		
		/** Computes the normalised (or not) jerk of a motion along an axis.
		* @param motion Initial motion
		* @param jerk_vec Output vector
		* @param axis Axis along which the jerk will be computed
		* @param normalise Indicating if the values must be normalised or not 
		* @return The jerk values along one axis 
		*/
		void computeJerk(Motion* motion, std::vector<std::map<std::string, double>>& jerk_vec, std::string& axis, bool normalise) {
			
			if (axis != "x" && axis != "y" && axis != "z" && axis != "norm") {
				std::cout << "computeJerk() error: axis parameter must be x, y, z or norm" << std::endl;
					return;
			}
			
			jerk_vec.clear();

			std::map<std::string, double> frame_jerk;

			Frame* f1 = nullptr;
			Frame* f2 = nullptr;
			Frame* f3 = nullptr;
			Frame* f4 = nullptr;

			// jerk = x(t+2) - 2x(t+1) + 2x(t-1) - x(t-2) / 2t^3 
			for (unsigned int i = 0; i < motion->getFrames().size(); i++) {
				frame_jerk.clear();

				if (i == 0) {
					f1 = motion->getFrame(i + 2);
					f2 = motion->getFrame(i + 1);
					f3 = motion->getFrame(i);
					f4 = motion->getFrame(i);
				}
						
				else if (i == 1) {
					f1 = motion->getFrame(i + 2);
					f2 = motion->getFrame(i + 1);
					f3 = motion->getFrame(i - 1);
					f4 = motion->getFrame(i);
				}
						
				else if (i == motion->getFrames().size() - 2) {
					f1 = motion->getFrame(i);
					f2 = motion->getFrame(i + 1);
					f3 = motion->getFrame(i - 1);
					f4 = motion->getFrame(i - 2);
				}

				else if (i == motion->getFrames().size() - 1) {
					f1 = motion->getFrame(i);
					f2 = motion->getFrame(i);
					f3 = motion->getFrame(i - 1);
					f4 = motion->getFrame(i - 2);
				}

				else {
					f1 = motion->getFrame(i + 2);
					f2 = motion->getFrame(i + 1);
					f3 = motion->getFrame(i - 1);
					f4 = motion->getFrame(i - 2);
				}

				if (axis == "norm")
					jointsJerkNorm(frame_jerk, f1, f2, f3, f4, motion->getFrameTime());

				else if (axis == "x" || axis == "y" || axis == "z")
					jointsJerkAxis(frame_jerk, f1, f2, f3, f4, motion->getFrameTime(), axis, normalise);

				jerk_vec.push_back(frame_jerk);
			}
		}

		/** Computes the bounding boxes for each frames of a motion for a given set of joints.
		* @param motion Motion from which the bounding boxes will be extracted
		* @param joints_to_check Joints on which the bounding boxes will be computed
		* @param bounding_boxes Output vector
		* @return A vector countaining all the bounding boxes
		*/
		void computeBoundingBoxes(Motion* motion, std::vector<std::map<std::string, std::vector<double>>>& bounding_boxes, std::vector<std::string>& joints_to_check) {
			if (joints_to_check.empty()) {
				std::cout << "ERROR: please specify a set of joints to extract bounding boxes." << std::endl;
				return;
			}

			bounding_boxes.clear();

			std::map<std::string, std::vector<double>> bb;
			glm::dvec3 root = glm::dvec3(0, 0, 0);

			for (unsigned int i = 0; i < motion->getFrames().size(); i++) {
				jointsBoundingBox(bb, motion->getFrame(i), joints_to_check);
				bounding_boxes.push_back(bb);
			}
		}

		/** Computes the total bounding boxes for one motion for a given set of joints.
		* @param motion Motion from which the bounding box will be extracted
		* @param joints_to_check Joints on which the bounding box will be computed
		* @param bounding_boxes Output vector
		* @return The final bouding box coordinates (-x, +x, -y, +y, -z, +z)
		*/
		void computeFinalBoudingBox(Motion* motion, std::vector<std::map<std::string, std::vector<double>>>& bounding_box, std::vector<std::string>& KC, bool normalise) {
			if (KC.empty()) {
				std::cout << "ERROR: please specify a set of joints to extract bounding boxes." << std::endl;
				return;
			}

			bounding_box.clear();

			std::vector<std::map<std::string, std::vector<double>>> bounding_boxes;
			std::map<std::string, std::vector<double>> bb;

			for (unsigned int i = 0; i < motion->getFrames().size(); i++) {
				jointsBoundingBox(bb, motion->getFrame(i), KC, normalise);
				bounding_boxes.push_back(bb);
			}	

			bb.clear();

			std::string final_name = "";
			for (auto it = KC.begin(); it != KC.end(); it++) {
				final_name += (*it);
			}

			for (auto it = bounding_boxes.begin(); it != bounding_boxes.end(); it++) {
				if (it == bounding_boxes.begin()) {
					bb[final_name] = (*it)[final_name];
				}

				else {
					if ((*it)[final_name][0] < bb[final_name][0])
						bb[final_name][0] = (*it)[final_name][0];

					if ((*it)[final_name][1] > bb[final_name][1])
						bb[final_name][1] = (*it)[final_name][1];

					if ((*it)[final_name][2] < bb[final_name][2])
						bb[final_name][2] = (*it)[final_name][2];

					if ((*it)[final_name][3] > bb[final_name][3])
						bb[final_name][3] = (*it)[final_name][3];

					if ((*it)[final_name][4] < bb[final_name][4])
						bb[final_name][4] = (*it)[final_name][4];

					if ((*it)[final_name][5] > bb[final_name][5])
						bb[final_name][5] = (*it)[final_name][5];
				}
			}

			bounding_box.push_back(bb);
		}
		
		/** Get the joints positions at the beginning of a throw (corresponding to the left local minimum next to the global maximum speed value).
		* @param motion Motion from which the values will be retrieved
		* @param seg_info Structure containing all the information needed for the segmentation (see SegmentationInformation)
		* @param joint_to_segment Joint used to find the values
		* @param pos_values Output vector
		* @return The desired positions values for all joints for all frames
		*/
		void computePositionBeforeThrow(Motion* motion, SegmentationInformation& seg_info, const std::string& joint_to_segment, 
			std::vector<std::map<std::string, glm::dvec3>>& pos_values, const std::string& abs_or_rel) {
			pos_values.clear();

			std::pair<int, int> throw_idx;

			SpeedData speed_data(motion->getFrames().size() - 1,
				motion->getFrameTime(),
				motion->getFrames().size());

			motionSpeedComputing(motion, speed_data);
			ComputeSavgol(speed_data, seg_info);

			std::vector<double> speed_norm;
			speed_data.getNorm(speed_norm, joint_to_segment);
			FindThrowIndex(speed_norm, throw_idx);

			std::map<std::string, glm::dvec3> pos_beg;

			jointsPositions(pos_beg, motion->getFrame(throw_idx.first), abs_or_rel);
			pos_values.push_back(pos_beg);
			
		}

		
		void computePositionBeforeThrow(Motion* motion, SegmentationInformation& seg_info, const std::string& joint_to_segment,
			std::vector<std::map<std::string, glm::dvec3>>& pos_values, std::vector<std::string>& KC) {
			pos_values.clear();

			std::pair<int, int> throw_idx;

			SpeedData speed_data(motion->getFrames().size() - 1,
				motion->getFrameTime(),
				motion->getFrames().size());

			motionSpeedComputing(motion, speed_data);
			ComputeSavgol(speed_data, seg_info);

			std::vector<double> speed_norm;
			speed_data.getNorm(speed_norm, joint_to_segment);
			FindThrowIndex(speed_norm, throw_idx);

			std::map<std::string, glm::dvec3> pos_beg;

			jointsPositions(pos_beg, motion->getFrame(throw_idx.first), KC);
			pos_values.push_back(pos_beg);

		}

		/** Compute a Savgol for a full motion speed values. Note that this is NOT reversible. TODO: un-optimised af, do it
		* @param speed_data Class containing the speed informations
		* @param data The savgol informations
		* @return The SpeedData class Savgol'ed
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

		/** Compute a Savgol for a full motion acceleration values. Note that this is NOT reversible. TODO: un-optimised af, do it
		* @param speed_data Class containing the acceleration informations
		* @param data The savgol informations
		* @return The AccData class Savgol'ed
		*/
		void ComputeSavgol (AccData& acc_data, SegmentationInformation& seg_info) {

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
		* @param motion the motion to be filtered
		* @return The motion with fixed members lengths
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