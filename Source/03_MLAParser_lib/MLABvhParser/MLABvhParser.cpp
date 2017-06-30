#include "MLABvhParser.h"

BvhParser::BvhParser() {

}

//TODO: use return value of searchForward()
Motion* BvhParser::parseBvh(const std::string& inputFile) {
	
	std::ifstream infile(inputFile.c_str());

	if (infile.fail()) {
		return false;
	}

	std::string currentWord;

	// Declarations for building the skeleton
	std::vector<std::string> jointList;

	std::vector<double> tmpOffsets;

	int channelNumber;

	std::vector<std::string> jointChannels;
	std::vector<std::string> parentNames;
	std::string parent;

	std::map<std::string, std::vector<std::string>> channels;
	
	//creating the first frame, which will hold the offsets
	Frame* initialFrame = new Frame();

	// Declaration for the frames part
	unsigned int frameNumber;

	// END
	std::cout << "Building skeleton ..." << std::endl;

	searchForward(infile, "ROOT");
	infile >> currentWord;

	//offset part
	while(currentWord != "MOTION") {
		jointList.push_back(currentWord);
		searchForward(infile, "OFFSET");

		infile >> currentWord;

		while(isDouble(currentWord.c_str())) {
			tmpOffsets.push_back(atof(currentWord.c_str()));
			infile >> currentWord;
		}

		if(currentWord != "CHANNELS")
			searchForward(infile, "CHANNELS");

		infile >> currentWord;

		jointChannels.clear();
		channelNumber = atoi(currentWord.c_str());

		for(int i=0; i<channelNumber ; i++) {
			infile >> currentWord;
			jointChannels.push_back(currentWord);
		}

		channels[jointList.back()] = jointChannels;

		parent = "NONE";

		if(!parentNames.empty()) {
			parent = parentNames.back();
		}

		Joint* currentJoint = new Joint();
		currentJoint->setJointName(jointList.back());
		currentJoint->setPositions(glm::dvec3(tmpOffsets.at(0), tmpOffsets.at(1), tmpOffsets.at(2)));
		currentJoint->setOrientations(glm::quat());
		currentJoint->setParent(0);

		if(parent != "NONE") {
			currentJoint->setParent(initialFrame->getJoint(parent));
			initialFrame->getJoint(parent)->addChild(currentJoint);
		}
		else if (!(jointList.back().find("End") == 0)){
			initialFrame->addRoot(currentJoint);
		}

		initialFrame->insertJoint(currentJoint);

		tmpOffsets.clear();

		parentNames.push_back(jointList.back());

		infile >> currentWord;

		if(currentWord == "End") {
			searchForward(infile, "OFFSET");
				
			infile >> currentWord;

			while (isDouble(currentWord.c_str())) {
				tmpOffsets.push_back(atof(currentWord.c_str()));
				infile >> currentWord;
			}

			Joint* endJoint = new Joint();
			endJoint->setJointName("End" + jointList.back());
			endJoint->setPositions(glm::dvec3(tmpOffsets.at(0), tmpOffsets.at(1), tmpOffsets.at(2)));
			endJoint->setOrientations(glm::quat());
				
			parent = parentNames.back();

			endJoint->setParent(initialFrame->getJoint(jointList.back()));
			initialFrame->getJoint(parent)->addChild(endJoint);

			initialFrame->insertJoint(endJoint);
			jointList.push_back("End" + jointList.back());
			tmpOffsets.clear();
		}

		if(currentWord == "}") {
			infile >> currentWord;
			while (currentWord == "}"){
				if(!parentNames.empty())
					parentNames.pop_back();
				infile >> currentWord;
			}
		}

		if(currentWord == "JOINT") {
			infile >> currentWord;
		}

	}

	
	// Motion ended
	std::cout << "Skeleton built." << std::endl << std::endl;

	// Now that the skeleton is built, we build the motion
	std::cout << "Building motion ..." << std::endl;

	Motion* motion = new Motion();

	// Frame number
	searchForward(infile, "Frames:");
	infile >> currentWord;
	frameNumber = atoi(currentWord.c_str());

	// Interframe time
	searchForward(infile, "Time:");
	infile >> currentWord;
	motion->setFrameTime(atof(currentWord.c_str()));

	motion->addFrame(initialFrame);

	unsigned int percentage = 0;

	// using initialFrame->getNames().size() (to get the number  
	// of joints) used up to 57% of this function's running time. 
	// Using initialFrame->getJoints().size() is incredibly faster 
	// (approximately 100 times faster), but why make a call to this 
	// function each time, rather than doing it only 1 time ?
	unsigned int jointNumber = initialFrame->getJoints().size();

	std::cout << "Copying frames ..." << std::endl;

	// Why not : " Frame* copiedFrame = new Frame(*initialFrame); " ?
	// Because if we do so, the member std::vector<Joint*> m_joints
	// will be copied as is, and the pointers inside will point to
	// the same data as the initial frame. So if you modify one
	// value, it will modify all the value (since the pointers all
	// points to the same joint in the memory).

	for(unsigned int i=0 ; i<frameNumber ; i++) {
		
		percentage = 100 * i / frameNumber;
		std::cout << percentage << " % (" << i << "/" << frameNumber << ")" << "\r";

		Frame* copiedFrame = new Frame();

		copiedFrame->setNames(initialFrame->getNames());
		copiedFrame->setRoots(initialFrame->getRoots());

		for (unsigned int i = 0; i<jointNumber; i++) {
			Joint* new_joint = new Joint(*initialFrame->getJoints().at(i));
			
			new_joint->setParent(0);

			if (initialFrame->getJoints().at(i)->getParent()) {
				std::string parentName = initialFrame->getJoints().at(i)->getParent()->getJointName();
				new_joint->setParent(copiedFrame->getJoint(parentName));
				new_joint->getParent()->addChild(new_joint);
			}
			
			copiedFrame->insertJoint(new_joint);

		}

		motion->addFrame(copiedFrame);
	}

	std::cout << "Frames copied." << std::endl;
	infile >> currentWord;

	percentage = 0;
	std::cout << "Motion data gathering ..." << std::endl;

	std::vector<std::string> currentChannels;
	
	// f = frame number
	// = 1, since the first frame is already inserted
	// frameNumber+1, since the first frame is inserted
	for(unsigned int f=1 ; f<frameNumber+1 ; f++) {

		percentage = 100 * f / frameNumber;
		std::cout << percentage << " % (" << f << "/" << frameNumber << ")" << "\r";

		// j = joint
		for(unsigned int j=0 ; j<jointList.size() ; j++) {

			// Endsites do not have information for the animation,
			// so we skip them
			if (!(jointList.at(j).find("End") == 0)) {
				currentChannels = channels[jointList.at(j)];
				glm::dvec3 pos;
				glm::quat ori;

				// Because if we don't check this, we will insert a (0,0,0)
				// vector inside the motion, regardless of if there was 
				// position information or not
				bool posFlag = false;
				
				// c = channel
				for (unsigned int c = 0; c < currentChannels.size(); c++) {

					if (currentChannels.at(c) == "Xposition") {
						pos.x = atof(currentWord.c_str());
						posFlag = true;
					}
						
					else if (currentChannels.at(c) == "Yposition") {
						pos.y = atof(currentWord.c_str());
						posFlag = true;
					}
						
					else if (currentChannels.at(c) == "Zposition") {
						pos.z = atof(currentWord.c_str());
						posFlag = true;
					}
						
					else if (currentChannels.at(c) == "Xrotation")
						ori = ori * glm::quat(glm::dvec3(glm::radians(atof(currentWord.c_str())), 0.0, 0.0));
					else if (currentChannels.at(c) == "Yrotation")
						ori = ori * glm::quat(glm::dvec3(0.0, glm::radians(atof(currentWord.c_str())), 0.0));
					else if (currentChannels.at(c) == "Zrotation")
						ori = ori * glm::quat(glm::dvec3(0.0, 0.0, glm::radians(atof(currentWord.c_str()))));

					infile >> currentWord;
				}

				if(posFlag)
					motion->getFrame(f)->getJoint(jointList.at(j))->setPositions(pos);

				motion->getFrame(f)->getJoint(jointList.at(j))->setOrientations(ori);
			}
		}
	}

	std::cout << "Motion data gathered." << std::endl;
	std::cout << "Animation frames built." << std::endl;

	infile.close();
	
	return motion;
}

bool BvhParser::searchForward(std::ifstream& infile, const std::string word) {
	std::string chunk;

	do {
		infile >> chunk;

		if (chunk == word)
			return true;

	} while (!infile.eof());

	return false;
}

//returns True if the str is a valid double, false else
bool BvhParser::isDouble(const char* str) {
	char* endptr = 0;

	// try to parse str into a floating point number (returns double, although
	// we do not use the returned value as it would be redundant here,
	// see http://stackoverflow.com/questions/27973759/return-value-of-strtod-if-string-equals-to-zero)
	std::strtod(str, &endptr);

	if (*endptr != '\0' || endptr == str)
		return false;

	return true;
}
