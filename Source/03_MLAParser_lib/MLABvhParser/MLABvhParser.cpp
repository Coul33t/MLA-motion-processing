#include "MLABvhParser.h"

namespace Mla {
	namespace BvhParser {
		// Anonymous namespace, because these functions are never
		// used outside the parseBvh function.
		namespace {
			bool searchForward(std::ifstream& infile, const std::string word) {
				std::string chunk;

				do {
					infile >> chunk;

					if (chunk == word)
						return true;

				} while (!infile.eof());

				return false;
			}

			//returns True if the str is a valid double, false else
			bool isDouble(const char* str) {
				char* endptr = 0;

				// try to parse str into a floating point number (returns double, although
				// we do not use the returned value as it would be redundant here,
				// see http://stackoverflow.com/questions/27973759/return-value-of-strtod-if-string-equals-to-zero)
				std::strtod(str, &endptr);

				if (*endptr != '\0' || endptr == str)
					return false;

				return true;
			}
		}

		//TODO: use return value of searchForward()
		std::unique_ptr<Motion> parseBvh(const std::string& folder, const std::string& input_file) {

			std::ifstream infile((folder + input_file).c_str());

			if (infile.fail()) {
				std::ifstream infile((folder + input_file +"/").c_str());
				if (infile.fail()) {
					return nullptr;
				}
			}

			std::string current_word;

			// Declarations for building the skeleton
			std::vector<std::string> joint_list;

			std::vector<double> tmp_offsets;

			int channel_number;

			std::vector<std::string> joint_channels;
			std::vector<std::string> parent_names;
			std::string parent;

			std::map<std::string, std::vector<std::string>> channels;

			//creating the first frame, which will hold the offsets
			std::shared_ptr<Frame> initial_frame = std::make_shared<Frame>();

			// Declaration for the frames part
			unsigned int frame_number;

			// END
			std::cout << "Building skeleton ..." << std::endl;

			searchForward(infile, "ROOT");
			infile >> current_word;

			//offset part
			while (current_word != "MOTION") {
				joint_list.push_back(current_word);
				searchForward(infile, "OFFSET");

				infile >> current_word;

				while (isDouble(current_word.c_str())) {
					tmp_offsets.push_back(atof(current_word.c_str()));
					infile >> current_word;
				}

				if (current_word != "CHANNELS")
					searchForward(infile, "CHANNELS");

				infile >> current_word;

				joint_channels.clear();
				channel_number = atoi(current_word.c_str());

				for (int i = 0; i < channel_number; i++) {
					infile >> current_word;
					joint_channels.push_back(current_word);
				}

				channels[joint_list.back()] = joint_channels;

				parent = "NONE";

				if (!parent_names.empty()) {
					parent = parent_names.back();
				}

				std::shared_ptr<Joint> current_joint = std::make_shared<Joint>();
				current_joint->setName(joint_list.back());
				current_joint->setPositions(glm::dvec3(tmp_offsets[0], tmp_offsets[1], tmp_offsets[2]));
				current_joint->setOrientations(glm::quat());
				current_joint->setParent(nullptr);

				// If it has parents, we add them
				if (parent != "NONE") {
					current_joint->setParent(initial_frame->getJoint(parent));
					initial_frame->getJoint(parent)->addChild(current_joint);
				}

				// Else, it's the root
				else {
					initial_frame->setRoot(current_joint);
				}

				initial_frame->insertJoint(current_joint);

				tmp_offsets.clear();

				parent_names.push_back(joint_list.back());

				infile >> current_word;

				if (current_word == "End") {
					searchForward(infile, "OFFSET");

					infile >> current_word;

					while (isDouble(current_word.c_str())) {
						tmp_offsets.push_back(atof(current_word.c_str()));
						infile >> current_word;
					}

					std::shared_ptr<Joint> end_joint = std::make_shared<Joint>();
					end_joint->setName("End" + joint_list.back());
					end_joint->setPositions(glm::dvec3(tmp_offsets[0], tmp_offsets[1], tmp_offsets[2]));
					end_joint->setOrientations(glm::quat());

					parent = parent_names.back();

					end_joint->setParent(initial_frame->getJoint(joint_list.back()));
					initial_frame->getJoint(parent)->addChild(end_joint);

					initial_frame->insertJoint(end_joint);
					joint_list.push_back("End" + joint_list.back());
					tmp_offsets.clear();
				}

				if (current_word == "}") {
					infile >> current_word;
					while (current_word == "}"){
						if (!parent_names.empty())
							parent_names.pop_back();
						infile >> current_word;
					}
				}

				if (current_word == "JOINT") {
					infile >> current_word;
				}

			}


			// Motion ended
			std::cout << "Skeleton built." << std::endl << std::endl;

			// Now that the skeleton is built, we build the motion
			std::cout << "Building motion ..." << std::endl;

			std::unique_ptr<Motion> motion = std::make_unique<Motion>();

			// Ugly way to remove .bvh at the end of the string
			motion->setName(input_file.substr(0, input_file.size() - 4));

			// Frame number
			searchForward(infile, "Frames:");
			infile >> current_word;
			frame_number = atoi(current_word.c_str());

			// Interframe time
			searchForward(infile, "Time:");
			infile >> current_word;
			if (current_word == "0.017")
				motion->setFrameTime(0.01666666);
			if (current_word == "0.008")
				motion->setFrameTime(0.00833333);

			motion->setOffsetFrame(initial_frame);

			unsigned int percentage = 0;

			// UPDATE: no more used 
			// TODO: verify and delete

			// using initial_frame->getNames().size() (to get the number  
			// of joints) used up to 57% of this function's running time. 
			// Using initial_frame->getJoints().size() is incredibly faster 
			// (approximately 100 times faster), but why make a call to this 
			// function each time, rather than doing it only 1 time ?

			// unsigned int jointNumber = initial_frame->getJoints().size();

			std::cout << "Copying frames ..." << std::endl;

			// Why not: " Frame* copied_frame = new Frame(*initial_frame); " ?
			// Because if we do so, the member std::vector<Joint*> m_joints
			// will be copied as is, and the pointers inside will point to
			// the same data as the initial frame. So if you modify one
			// value, it will modify all the value (since the pointers all
			// points to the same joint in the memory).

			// UPDATE: the frame duplication code has been moved inside
			// the Frame class, see duplicateFrame()

			for (unsigned int i = 0; i < frame_number; i++) {

				percentage = 100 * i / frame_number;
				std::cout << percentage << " % (" << i << "/" << frame_number << ")" << "\r";

				std::shared_ptr<Frame> copied_frame = initial_frame->duplicateFrame();

				motion->addFrame(copied_frame);
			}

			std::cout << "Frames copied." << std::endl;
			infile >> current_word;

			percentage = 0;
			std::cout << "Motion data gathering ..." << std::endl;

			std::vector<std::string> current_channels;

			// f = frame number
			// = 1, since the first frame is already inserted
			// frame_number+1, since the first frame is inserted
			for (unsigned int f = 0; f < frame_number; f++) {

				percentage = 100 * f / frame_number;
				std::cout << percentage << " % (" << f << "/" << frame_number << ")" << "\r";

				// j = joint
				for (unsigned int j = 0; j < joint_list.size(); j++) {

					// Endsites do not have information for the animation,
					// so we skip them
					if (!(joint_list[j].find("End") == 0)) {
						current_channels = channels[joint_list[j]];
						glm::dvec3 pos;
						glm::quat ori;

						// Because if we don't check this, we will insert a (0,0,0)
						// vector inside the motion, regardless of if there was 
						// position information or not
						bool posFlag = false;

						// c = channel
						for (unsigned int c = 0; c < current_channels.size(); c++) {

							if (current_channels[c] == "Xposition") {
								pos.x = atof(current_word.c_str());
								posFlag = true;
							}

							else if (current_channels[c] == "Yposition") {
								pos.y = atof(current_word.c_str());
								posFlag = true;
							}

							else if (current_channels[c] == "Zposition") {
								pos.z = atof(current_word.c_str());
								posFlag = true;
							}

							else if (current_channels[c] == "Xrotation")
								ori = ori * glm::quat(glm::dvec3(glm::radians(atof(current_word.c_str())), 0.0, 0.0));
							else if (current_channels[c] == "Yrotation")
								ori = ori * glm::quat(glm::dvec3(0.0, glm::radians(atof(current_word.c_str())), 0.0));
							else if (current_channels[c] == "Zrotation")
								ori = ori * glm::quat(glm::dvec3(0.0, 0.0, glm::radians(atof(current_word.c_str()))));

							infile >> current_word;
						}

						if (posFlag)
							motion->getFrame(f)->getJoint(joint_list[j])->setPositions(pos);

						motion->getFrame(f)->getJoint(joint_list[j])->setOrientations(ori);
					}
				}
			}

			std::cout << "Motion data gathered." << std::endl;
			std::cout << "Animation frames built." << std::endl;

			infile.close();

			return motion;
		}


	}
}

