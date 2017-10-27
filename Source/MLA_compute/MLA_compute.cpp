#include "MLA.h"

unsigned int LinearSpeed();
unsigned int ProcessCut();
unsigned int ProcessClassic();
unsigned int FilteringTest();
unsigned int SavgolTest();
unsigned int MemoryLeakChaser();
unsigned int MinimaTest();
unsigned int GlobalTest();

int main(int argc, char *argv[]) {
	SavgolTest();
	//std::cout << std::endl << "Press any key to quit...";
	//std::cin.get();
	return 0;
}

unsigned int LinearSpeed() {
	Motion* motion = nullptr;

	motion = mla::bvhparser::parseBvh(MLA_INPUT_BVH_PATH, "Damien_4_Char00.bvh");

	mla::motionoperation::motionFiltering(motion);

	for (unsigned int i = 1; i < motion->getFrames().size() - 1; i++) {
		std::string linName = "lin_full_" + std::to_string(i);
		std::string angName = "ang_full_" + std::to_string(i);

		std::string linFolder = "lin\\";
		std::string angFolder = "ang\\";

		mla::csvexport::ExportData(mla::motionoperation::jointsLinearSpeed(motion->getFrame(i), motion->getFrame(i + 1), motion->getFrameTime()), motion->getName(), linFolder, linName);
		mla::csvexport::ExportData(mla::motionoperation::jointsAngularSpeed(motion->getFrame(i), motion->getFrame(i + 1), motion->getFrameTime()), motion->getName(), angFolder, angName);
	}

	return 0;
}

unsigned int ProcessCut() {

	std::vector<std::string> names;

	Motion* motion = nullptr;

	std::vector<std::map<std::string, double>> meanLinSpeedInter;

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
		motion = mla::bvhparser::parseBvh(path, names.at(i));

		if (motion == false) {
			std::cout << "Failed to parse bvh file (is the path/namefile correct ?)" << std::endl;
			std::cout << "Press ENTER to quit...";
			std::cin.get();
			return 1;
		}

		mla::motionoperation::motionFiltering(motion);

		for (unsigned int cut = 2; cut < 21; cut++){
			std::cout << "Processing cut " << cut << std::endl;
			meanLinSpeedInter = mla::motionoperation::MeanLinearSpeed(motion, cut);

			for (unsigned int i = 0; i < meanLinSpeedInter.size(); i++) {
				std::string Name = "lin_mean_" + std::to_string(i);
				std::string Folder = "lin_mean_" + std::to_string(cut) + "_cut\\";

				//CSVExport::EraseFolderContent(MLA_INPUT_BVH_PATH + motion->getName() + "/lin_mean/");
				mla::csvexport::ExportData(meanLinSpeedInter.at(i), motion->getName(), Folder, Name);
			}
		}

		delete motion;
	}

	return 0;
}

unsigned int ProcessClassic() {

	std::vector<std::string> names;

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
		motion = mla::bvhparser::parseBvh(MLA_INPUT_BVH_PATH, names.at(i));

		if (motion == nullptr) {
			std::cout << "Failed to parse bvh file (is the path/namefile correct ?)" << std::endl;
			std::cout << "Press ENTER to quit...";
			std::cin.get();
			return 1;
		}

		mla::motionoperation::motionFiltering(motion);

		for (unsigned int cut = 20; cut < 220; cut += 20) {
			std::cout << "Processing cut " << cut << std::endl;
			meanLinSpeedInter = mla::motionoperation::MeanLinearSpeed(motion, cut);

			for (unsigned int i = 0; i < meanLinSpeedInter.size(); i++) {
				std::string Name = "lin_mean_" + std::to_string(i);
				std::string Folder = "lin_mean_" + std::to_string(cut) + "_cut\\";

				//csvexport::EraseFolderContent(MLA_INPUT_BVH_PATH + motion->getName() + "/lin_mean/");
				mla::csvexport::ExportData(meanLinSpeedInter.at(i), motion->getName(), Folder, Name);
			}
		}

		delete motion;
	}

	return 0;
}

unsigned int FilteringTest() {
	Motion* motion = nullptr;

	motion = mla::bvhparser::parseBvh(MLA_INPUT_BVH_PATH, "Damien_4_Char00.bvh");

	mla::motionoperation::motionFiltering(motion);

	std::vector<std::map<std::string, double>> map_to_test;

	for (unsigned int i = 1; i < motion->getFrames().size() - 1; i++)
		map_to_test.push_back(mla::motionoperation::jointsLinearSpeed(motion->getFrame(i), motion->getFrame(i + 1), motion->getFrameTime()));

	std::vector<double> values;

	for (unsigned int i = 0; i < map_to_test.size(); ++i) {
		std::map<std::string, double> slt = map_to_test[i];
		values.push_back(slt.find("LeftHand")->second);
	}

	std::vector<double> output;
	mla::filters::MeanShift(output, values, 45);
	mla::csvexport::ExportData(output, "Damien", "TEST_FILTRE", "test_filtre_45");

	return 0;
}

unsigned int SavgolTest() {
	Motion* motion = nullptr;

	motion = mla::bvhparser::parseBvh(MLA_INPUT_BVH_PATH, "Damien_4_Char00.bvh");

	mla::motionoperation::motionFiltering(motion);

	std::vector<std::map<std::string, double>> map_to_test;

	for (unsigned int i = 1; i < motion->getFrames().size() - 1; i++)
		map_to_test.push_back(mla::motionoperation::jointsLinearSpeed(motion->getFrame(i), motion->getFrame(i + 1), motion->getFrameTime()));

	std::vector<double> values;

	for (unsigned int i = 0; i < map_to_test.size(); ++i) {
		std::map<std::string, double> slt = map_to_test[i];
		values.push_back(slt.find("LeftHand")->second);
	}

	std::vector<double> output;
	mla::filters::Savgol(output, values, 3, 21);

	mla::csvexport::ExportData(output, "Damien", "TEST_SAVGOL", "savgol_test");
	return 0;
}

unsigned int MinimaTest() {

	Motion* motion = nullptr;

	motion = mla::bvhparser::parseBvh(MLA_INPUT_BVH_PATH, "Damien_1_Char00.bvh");

	mla::motionoperation::motionFiltering(motion);

	// Import data into a map, to separate the hand data from the rest of the joints
	std::vector<std::map<std::string, double>> map_to_test;

	for (unsigned int i = 1; i < motion->getFrames().size() - 1; i++)
		map_to_test.push_back(mla::motionoperation::jointsLinearSpeed(motion->getFrame(i), motion->getFrame(i + 1), motion->getFrameTime()));

	std::vector<double> values;

	for (unsigned int i = 0; i < map_to_test.size(); ++i) {
		std::map<std::string, double> slt = map_to_test[i];
		values.push_back(slt.find("LeftHand")->second);
	}

	// Savgol-ing the values
	std::vector<double> output;
	mla::filters::Savgol(output, values, 3, 21);

	// Finding the left time and right time for the minimums around the global maximumS
	double left_min = mla::motionoperation::getLocalMinimumFromMaximum(output, motion->getFrameTime(), -1);
	double right_min = mla::motionoperation::getLocalMinimumFromMaximum(output, motion->getFrameTime(), 1);

	unsigned int beg = (int)(left_min / motion->getFrameTime());
	unsigned int end = (int)(right_min / motion->getFrameTime());

	std::vector<double> to_export;
	for (unsigned int i = beg; i < end + 1; i++)
		to_export.push_back(output[i]);

	mla::csvexport::ExportData(to_export, "Damien", "VALUES", "values_extract");

	return 0;
}

unsigned int GlobalTest() {

	std::vector<std::string> names;

	// Ugly af, but no fast/easy way to get all files into a folder
	for (unsigned int i = 0; i<10; i++)  {
		std::string name = "Damien_";
		name += std::to_string(i + 1);
		name += "_Char00.bvh";
		names.push_back(name);
	}

	Motion* motion = nullptr;

	// Getting the useful part
	for (unsigned int i = 0; i < names.size(); i++) {
		std::cout << std::endl << std::endl << "Processing file " << names.at(i) << std::endl;
		motion = mla::bvhparser::parseBvh(MLA_INPUT_BVH_PATH, names.at(i));

		if (motion == nullptr) {
			std::cout << "Failed to parse bvh file (is the path/namefile correct ?)" << std::endl;
			std::cout << "Press ENTER to quit...";
			std::cin.get();
			return 1;
		}

		mla::motionoperation::motionFiltering(motion);


		// Import data into a map, to separate the hand data from the rest of the joints
		std::vector<std::map<std::string, double>> map_to_test;

		for (unsigned int j = 0; j < motion->getFrames().size() - 1; j++) {
			map_to_test.push_back(mla::motionoperation::jointsLinearSpeed(motion->getFrame(j), motion->getFrame(j + 1), motion->getFrameTime()));
		}
			

		std::vector<double> values;

		for (unsigned int j = 0; j < map_to_test.size(); ++j) {
			std::map<std::string, double> slt = map_to_test[j];
			values.push_back(slt.find("LeftHand")->second);
		}

		std::string Name = "original_" + motion->getName();
		std::string Folder = "original";
		mla::csvexport::ExportData(values, motion->getName(), Folder, Name);

		// Savgol-ing the values
		std::vector<double> values_savgol;
		mla::filters::Savgol(values_savgol, values, 3, 21);

		Name = "savgol_" + motion->getName();
		Folder = "savgol";
		mla::csvexport::ExportData(values_savgol, motion->getName(), Folder, Name);

		// Finding the left time and right time for the minimums around the global maximumS
		double left_min = mla::motionoperation::getLocalMinimumFromMaximum(values_savgol, motion->getFrameTime(), -1);
		double right_min = mla::motionoperation::getLocalMinimumFromMaximum(values_savgol, motion->getFrameTime(), 1);

		unsigned int beg = (int)(left_min / motion->getFrameTime());
		unsigned int end = (int)(right_min / motion->getFrameTime());

		std::vector<double> to_export;
		for (unsigned int j = beg; j < end + 1; j++)
			to_export.push_back(values_savgol[j]);

		Name = "useful_" + motion->getName();
		Folder = "useful";

		mla::csvexport::ExportData(to_export, motion->getName(), Folder, Name);
	}
	
	return 0;
}

unsigned int MemoryLeakChaser(){
	Motion* motion = nullptr;
	std::string name = "apur.bvh";

	for (unsigned int i=0 ; i<100 ; i++) {
		std::cout << std::endl << std::endl << "Processing file " << name << std::endl;
		motion = mla::bvhparser::parseBvh(MLA_INPUT_BVH_PATH, name);

		if (motion == nullptr) {
			std::cout << "Failed to parse bvh file (is the path/namefile correct ?)" << std::endl;
			std::cout << "Press ENTER to quit...";
			std::cin.get();
			return 1;
		}

		mla::motionoperation::MeanLinearSpeed(motion, 10);

		delete motion;

	}	

	return 0;
}