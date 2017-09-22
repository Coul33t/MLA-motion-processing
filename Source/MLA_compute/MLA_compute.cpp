#include "MLA.h"

unsigned int ProcessCut();
unsigned int ProcessClassic();
unsigned int MemoryLeakChaser();

int main(int argc, char *argv[]) {
	ProcessClassic();
	std::cout << std::endl << "Press any key to quit...";
	std::cin.get();
	return 0;
}

unsigned int ProcessCut() {
	BvhParser parser;

	std::vector<std::string> names;

	// Ugly af, but no fast/easy way to get all files into a folder
	for (int i = 0; i<20; i++)  {
		std::string name = "throw_";
		name += std::to_string(i + 1);
		name += "_cut.bvh";
		names.push_back(name);
	}

	for (unsigned int i = 0; i < names.size(); i++) {
		std::cout << std::endl << std::endl << "Processing file " << names.at(i) << std::endl;
		std::string path = MLA_INPUT_BVH_PATH;
		path += "cut/";
		Motion* motion = parser.parseBvh(path, names.at(i));

		if (motion == false) {
			std::cout << "Failed to parse bvh file (is the path/namefile correct ?)" << std::endl;
			std::cout << "Press ENTER to quit...";
			std::cin.get();
			return 1;
		}

		MotionOperation motionOp;

		motionOp.motionFiltering(motion);

		/*for (unsigned int i = 1; i<motion->getFrames().size() - 1; i++) {
		std::string linName = "lin_full_" + std::to_string(i);
		std::string angName = "ang_full_" + std::to_string(i);

		std::string linFolder = "lin\\";
		std::string angFolder = "ang\\";

		CSVExport::ExportData(motionOp.jointsLinearSpeed(motion->getFrame(i), motion->getFrame(i + 1), motion->getFrameTime()), motion->getName(), linFolder, linName);
		CSVExport::ExportData(motionOp.jointsAngularSpeed(motion->getFrame(i), motion->getFrame(i + 1), motion->getFrameTime()), motion->getName(), angFolder, angName);
	}*/
		for (unsigned int cut = 2; cut < 21; cut++){
			std::cout << "Processing cut " << cut << std::endl;
			std::vector<std::map<std::string, double>> meanLinSpeedInter = motionOp.MeanLinearSpeedIntervalFrame(motion, cut);

			for (unsigned int i = 0; i < meanLinSpeedInter.size(); i++) {
				std::string Name = "lin_mean_" + std::to_string(i);
				std::string Folder = "lin_mean_" + std::to_string(cut) + "_cut\\";

				//CSVExport::EraseFolderContent(MLA_INPUT_BVH_PATH + motion->getName() + "/lin_mean/");
				CSVExport::ExportData(meanLinSpeedInter.at(i), motion->getName(), Folder, Name);
			}
		}

		delete motion;
	}

	return 0;
}

unsigned int ProcessClassic() {
	BvhParser parser;

	std::vector<std::string> names;

	MotionOperation motionOp;

	Motion* motion = nullptr;

	std::vector<std::map<std::string, double>> meanLinSpeedInter;

	// Ugly af, but no fast/easy way to get all files into a folder
	for (int i = 0; i<20; i++)  {
		std::string name = "throw_";
		name += std::to_string(i + 1);
		name += "_gimbal_smooth_16.bvh";
		names.push_back(name);
	}

	for (unsigned int i = 0; i < names.size(); i++) {
		std::cout << std::endl << std::endl << "Processing file " << names.at(i) << std::endl;
		motion = parser.parseBvh(MLA_INPUT_BVH_PATH, names.at(i));

		if (motion == nullptr) {
			std::cout << "Failed to parse bvh file (is the path/namefile correct ?)" << std::endl;
			std::cout << "Press ENTER to quit...";
			std::cin.get();
			return 1;
		}

		motionOp.motionFiltering(motion);

		/*for (unsigned int i = 1; i<motion->getFrames().size() - 1; i++) {
		std::string linName = "lin_full_" + std::to_string(i);
		std::string angName = "ang_full_" + std::to_string(i);

		std::string linFolder = "lin\\";
		std::string angFolder = "ang\\";

		CSVExport::ExportData(motionOp.jointsLinearSpeed(motion->getFrame(i), motion->getFrame(i + 1), motion->getFrameTime()), motion->getName(), linFolder, linName);
		CSVExport::ExportData(motionOp.jointsAngularSpeed(motion->getFrame(i), motion->getFrame(i + 1), motion->getFrameTime()), motion->getName(), angFolder, angName);
		}*/

		for (unsigned int cut = 20; cut < 220; cut += 20) {
			std::cout << "Processing cut " << cut << std::endl;
			meanLinSpeedInter = motionOp.MeanLinearSpeedIntervalFrame(motion, cut);

			for (unsigned int i = 0; i < meanLinSpeedInter.size(); i++) {
				std::string Name = "lin_mean_" + std::to_string(i);
				std::string Folder = "lin_mean_" + std::to_string(cut) + "_cut\\";

				//CSVExport::EraseFolderContent(MLA_INPUT_BVH_PATH + motion->getName() + "/lin_mean/");
				//CSVExport::ExportData(meanLinSpeedInter.at(i), motion->getName(), Folder, Name);
			}
		}

		delete motion;
	}

	return 0;
}

unsigned int MemoryLeakChaser(){
	Motion* motion = nullptr;
	std::string name = "apur.bvh";

	BvhParser parser;

	MotionOperation motionOp;

	for (unsigned int i=0 ; i<100 ; i++) {
		std::cout << std::endl << std::endl << "Processing file " << name << std::endl;
		motion = parser.parseBvh(MLA_INPUT_BVH_PATH, name);

		if (motion == nullptr) {
			std::cout << "Failed to parse bvh file (is the path/namefile correct ?)" << std::endl;
			std::cout << "Press ENTER to quit...";
			std::cin.get();
			return 1;
		}

		motionOp.MeanLinearSpeedIntervalFrame(motion, 10);
		delete motion;

	}	

	return 0;
}