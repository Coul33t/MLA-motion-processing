#include "MLA.h"
#include <algorithm>

// TODO: automatically find the joint
//       with the highest speed
#define JOINT_OF_INTEREST "LeftHand"

unsigned int LinearSpeed();
unsigned int MotionCut(int n);
unsigned int ProcessCut();
unsigned int ProcessClassic();
unsigned int FilteringTest();
unsigned int SavgolTest();
unsigned int MinimaTest();
unsigned int GlobalTest();
unsigned int FindIndexSeparationTest();
unsigned int ReconstructTest();
unsigned int copyFrameTest();
unsigned int SegmentTest();
unsigned int SpeedDataTest();
unsigned int OneMotionExportTest(std::string&, std::string&);
unsigned int MultipleMotionExportTest();
unsigned int DataClassTest(std::string&, std::string&);
unsigned int JESAISPASQUOITEST();
unsigned int GlmFunctionsTest();
unsigned int FullDataTest();

unsigned int MemoryLeakChaser();

int main(int argc, char *argv[]) {
	//MultipleMotionExportTest();
	//std::string folder = "C:/Users/quentin/Documents/Programmation/C++/MLA/Data/Bvh/";
	//std::string file = "Guillaume_1_Char00.bvh";
	//DataClassTest(folder, file);

	FullDataTest();

	//GlmFunctionsTest();
	/*std::cout << std::endl << "Press any key to quit...";
	std::cin.get();*/
	return 0;
}

unsigned int LinearSpeed() {
	Motion* motion = nullptr;

	motion = Mla::BvhParser::parseBvh(MLA_INPUT_BVH_PATH, "Damien_1_Char00.bvh");

	Mla::MotionOperation::motionFiltering(motion);

	std::map<std::string, double> values_lin;
	std::map<std::string, double> values_ang;

	for (unsigned int i = 0; i < motion->getFrames().size() - 1; i++) {
		std::string linName = "lin_full_" + std::to_string(i);
		std::string angName = "ang_full_" + std::to_string(i);

		std::string linFolder = "lin\\";
		std::string angFolder = "ang\\";

		values_lin.clear();
		Mla::MotionOperation::jointsLinearSpeed(values_lin, motion->getFrame(i), motion->getFrame(i + 1), motion->getFrameTime());
		Mla::CsvExport::ExportData(values_lin, motion->getName(), linFolder, linName);

		values_ang.clear();
		Mla::MotionOperation::jointsLinearSpeed(values_ang, motion->getFrame(i), motion->getFrame(i + 1), motion->getFrameTime());
		Mla::CsvExport::ExportData(values_ang, motion->getName(), linFolder, linName);
	}

	return 0;
}

unsigned int MotionCut(int n) {
	Motion* motion = nullptr;

	motion = Mla::BvhParser::parseBvh(MLA_INPUT_BVH_PATH, "Damien_1_Char00.bvh");

	Mla::MotionOperation::motionFiltering(motion);

	std::vector<std::map<std::string, double>> lin_speed;

	if (n < 0)
		n = motion->getFrames().size();

	Mla::MotionOperation::MeanLinearSpeed(lin_speed, motion, n);
	
	for (unsigned int i = 0; i < lin_speed.size(); i++) {
		std::string Name = "lin_mean_" + std::to_string(i);
		std::string Folder = "lin_mean_" + std::to_string(n) + "_cut\\";

		//CSVExport::EraseFolderContent(MLA_INPUT_BVH_PATH + motion->getName() + "/lin_mean/");
		Mla::CsvExport::ExportData(lin_speed[i], "Damien", "TEST_CUT_MAX", Name);

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

	if (motion == false) {
		std::cout << "Failed to parse bvh file (is the path/namefile correct ?)" << std::endl;
		std::cout << "Press ENTER to quit...";
		std::cin.get();
		return 1;
	}

	for (unsigned int i = 0; i < names.size(); i++) {
		std::cout << std::endl << std::endl << "Processing file " << names[i] << std::endl;
		std::string path = MLA_INPUT_BVH_PATH;
		path += "cut/";
		motion = Mla::BvhParser::parseBvh(path, names[i]);

		if (motion == false) {
			std::cout << "Failed to parse bvh file (is the path/namefile correct ?)" << std::endl;
			std::cout << "Press ENTER to quit...";
			std::cin.get();
			return 1;
		}

		Mla::MotionOperation::motionFiltering(motion);

		for (unsigned int cut = 2; cut < 21; cut++){
			std::cout << "Processing cut " << cut << std::endl;

			meanLinSpeedInter.clear();
			Mla::MotionOperation::MeanLinearSpeed(meanLinSpeedInter, motion, cut);

			for (unsigned int j = 0; j < meanLinSpeedInter.size(); j++) {
				std::string Name = "lin_mean_" + std::to_string(j);
				std::string Folder = "lin_mean_" + std::to_string(cut) + "_cut\\";

				//CSVExport::EraseFolderContent(MLA_INPUT_BVH_PATH + motion->getName() + "/lin_mean/");
				Mla::CsvExport::ExportData(meanLinSpeedInter[j], motion->getName(), Folder, Name);
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
		std::cout << std::endl << std::endl << "Processing file " << names[i] << std::endl;
		motion = Mla::BvhParser::parseBvh(MLA_INPUT_BVH_PATH, names[i]);

		if (motion == nullptr) {
			std::cout << "Failed to parse bvh file (is the path/namefile correct ?)" << std::endl;
			std::cout << "Press ENTER to quit...";
			std::cin.get();
			return 1;
		}

		Mla::MotionOperation::motionFiltering(motion);

		for (unsigned int cut = 20; cut < 220; cut += 20) {
			std::cout << "Processing cut " << cut << std::endl;

			meanLinSpeedInter.clear();
			Mla::MotionOperation::MeanLinearSpeed(meanLinSpeedInter, motion, cut);

			for (unsigned int j = 0; j < meanLinSpeedInter.size(); j++) {
				std::string Name = "lin_mean_" + std::to_string(j);
				std::string Folder = "lin_mean_" + std::to_string(cut) + "_cut\\";

				//csvexport::EraseFolderContent(MLA_INPUT_BVH_PATH + motion->getName() + "/lin_mean/");
				Mla::CsvExport::ExportData(meanLinSpeedInter[j], motion->getName(), Folder, Name);
			}
		}

		delete motion;
	}

	return 0;
}

unsigned int FilteringTest() {
	Motion* motion = nullptr;

	motion = Mla::BvhParser::parseBvh(MLA_INPUT_BVH_PATH, "Damien_4_Char00.bvh");

	Mla::MotionOperation::motionFiltering(motion);

	std::map<std::string, double> lin_speed;
	std::vector<std::map<std::string, double>> map_to_test;

	for (unsigned int i = 0; i < motion->getFrames().size() - 1; i++) {
		lin_speed.clear();
		Mla::MotionOperation::jointsLinearSpeed(lin_speed, motion->getFrame(i), motion->getFrame(i + 1), motion->getFrameTime());
		map_to_test.push_back(lin_speed);
	}
		

	std::vector<double> values;

	for (unsigned int i = 0; i < map_to_test.size(); ++i) {
		std::map<std::string, double> slt = map_to_test[i];
		values.push_back(slt.find("LeftHand")->second);
	}

	std::vector<double> output;
	Mla::Filters::MeanShift(output, values, 45);
	Mla::CsvExport::ExportData(output, "Damien", "TEST_FILTRE", "test_filtre_45");

	return 0;
}

unsigned int SavgolTest() {
	Motion* motion = nullptr;

	motion = Mla::BvhParser::parseBvh(MLA_INPUT_BVH_PATH, "Damien_4_Char00.bvh");

	Mla::MotionOperation::motionFiltering(motion);

	std::map<std::string, double> lin_speed;
	std::vector<std::map<std::string, double>> map_to_test;

	for (unsigned int i = 0; i < motion->getFrames().size() - 1; i++) {
		lin_speed.clear();
		Mla::MotionOperation::jointsLinearSpeed(lin_speed, motion->getFrame(i), motion->getFrame(i + 1), motion->getFrameTime());
		map_to_test.push_back(lin_speed);
	}

	std::vector<double> values;

	for (unsigned int i = 0; i < map_to_test.size(); ++i) {
		std::map<std::string, double> slt = map_to_test[i];
		values.push_back(slt.find("LeftHand")->second);
	}

	std::vector<double> output;
	Mla::Filters::Savgol(output, values, 3, 51);

	Mla::CsvExport::ExportData(output, "Damien", "TEST_SAVGOL", "savgol_test_51");
	return 0;
}

unsigned int MinimaTest() {

	Motion* motion = nullptr;

	motion = Mla::BvhParser::parseBvh(MLA_INPUT_BVH_PATH, "Damien_1_Char00.bvh");

	Mla::MotionOperation::motionFiltering(motion);

	std::map<std::string, double> lin_speed;
	// Import data into a map, to separate the hand data from the rest of the joints
	std::vector<std::map<std::string, double>> map_to_test;

	for (unsigned int i = 0; i < motion->getFrames().size() - 1; i++) {
		lin_speed.clear();
		Mla::MotionOperation::jointsLinearSpeed(lin_speed, motion->getFrame(i), motion->getFrame(i + 1), motion->getFrameTime());
		map_to_test.push_back(lin_speed);
	}

	std::vector<double> values;

	for (unsigned int i = 0; i < map_to_test.size(); ++i) {
		std::map<std::string, double> slt = map_to_test[i];
		values.push_back(slt.find("LeftHand")->second);
	}

	// Savgol-ing the values
	std::vector<double> output;
	Mla::Filters::Savgol(output, values, 3, 21);

	// Finding the left time and right time for the minimums around the global maximumS
	unsigned int left_min = Mla::MotionOperation::getLocalMinimumFromMaximum(output, -1);
	unsigned int right_min = Mla::MotionOperation::getLocalMinimumFromMaximum(output, 1);


	std::vector<double> to_export;
	for (unsigned int i = left_min; i < right_min + 1; i++)
		to_export.push_back(output[i]);

	Mla::CsvExport::ExportData(to_export, "Damien", "VALUES", "values_extract");

	return 0;
}

unsigned int FindIndexSeparationTest() {
	Motion* motion = nullptr;

	motion = Mla::BvhParser::parseBvh(MLA_INPUT_BVH_PATH, "Damien_1_Char00.bvh");

	Mla::MotionOperation::motionFiltering(motion);

	std::map<std::string, double> lin_speed;
	// Import data into a map, to separate the hand data from the rest of the joints
	std::vector<std::map<std::string, double>> map_to_test;

	for (unsigned int i = 0; i < motion->getFrames().size() - 1; i++) {
		lin_speed.clear();
		Mla::MotionOperation::jointsLinearSpeed(lin_speed, motion->getFrame(i), motion->getFrame(i + 1), motion->getFrameTime());
		map_to_test.push_back(lin_speed);
	}

	std::vector<double> values;

	for (unsigned int i = 0; i < map_to_test.size(); ++i) {
		std::map<std::string, double> slt = map_to_test[i];
		values.push_back(slt.find("LeftHand")->second);
	}

	// Savgol-ing the values
	std::vector<double> savgoled;
	Mla::Filters::Savgol(savgoled, values, 3, 21);

	std::vector<std::pair<int, int>> output;

	Mla::MotionOperation::FindIndexSeparation(savgoled, 2, 8, output);

	Mla::CsvExport::ExportData(output, "Damien", "SEGMENT", "segment_intervals_2_8");

	return 0;
}

unsigned int SegmentTest() {
	Motion* motion = nullptr;

	motion = Mla::BvhParser::parseBvh(MLA_INPUT_BVH_PATH, "Damien_2_Char00.bvh");

	Mla::MotionOperation::motionFiltering(motion);

	std::vector<Motion*> motion_vector;

	SegmentationInformation seg_info = {
		30,	// left cut
		30, // right cut
		51, // window size
		3,  // polynom order
		20, // final frame number
	};

	Mla::MotionOperation::MotionSegmentation(motion, seg_info, motion_vector, JOINT_OF_INTEREST);

	for (unsigned int i = 0; i < motion_vector.size(); i++) {
		for (unsigned int j = 0; j < motion_vector[i]->getFrames().size(); j++) {
			
			std::string subfolder_name = "SEGMENT_" + std::to_string(i);
			std::string filename = "Frame_" + std::to_string(j);

			Mla::CsvExport::ExportData(motion_vector[i]->getFrame(j), "Damien_segmented", subfolder_name, filename);
		}
	}
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
		std::cout << std::endl << std::endl << "Processing file " << names[i] << std::endl;
		motion = Mla::BvhParser::parseBvh(MLA_INPUT_BVH_PATH, names[i]);

		if (motion == nullptr) {
			std::cout << "Failed to parse bvh file (is the path/namefile correct ?)" << std::endl;
			std::cout << "Press ENTER to quit...";
			std::cin.get();
			return 1;
		}

		Mla::MotionOperation::motionFiltering(motion);

		std::map<std::string, double> lin_speed;
		// Import data into a map, to separate the hand data from the rest of the joints
		std::vector<std::map<std::string, double>> map_to_test;

		for (unsigned int j = 0; j < motion->getFrames().size() - 1; j++) {
			lin_speed.clear();
			Mla::MotionOperation::jointsLinearSpeed(lin_speed, motion->getFrame(j), motion->getFrame(j + 1), motion->getFrameTime());
			map_to_test.push_back(lin_speed);
		}
			

		std::vector<double> values;

		for (unsigned int j = 0; j < map_to_test.size(); ++j) {
			std::map<std::string, double> slt = map_to_test[j];
			values.push_back(slt.find("LeftHand")->second);
		}

		std::string Name = "original_" + motion->getName();
		std::string Folder = "original";
		Mla::CsvExport::ExportData(values, motion->getName(), Folder, Name);

		// Savgol-ing the values
		std::vector<double> values_savgol;
		Mla::Filters::Savgol(values_savgol, values, 3, 21);

		Name = "savgol_" + motion->getName();
		Folder = "savgol";
		Mla::CsvExport::ExportData(values_savgol, motion->getName(), Folder, Name);

		// Finding the left time and right time for the minimums around the global maximumS
		unsigned int  left_min = Mla::MotionOperation::getLocalMinimumFromMaximum(values_savgol, -1);
		unsigned int  right_min = Mla::MotionOperation::getLocalMinimumFromMaximum(values_savgol, 1);

		std::vector<double> to_export;
		for (unsigned int j = left_min; j < right_min + 1; j++)
			to_export.push_back(values_savgol[j]);

		Name = "useful_" + motion->getName();
		Folder = "useful";

		Mla::CsvExport::ExportData(to_export, motion->getName(), Folder, Name);
	}
	
	return 0;
}

unsigned int ReconstructTest() {
	Motion* motion = nullptr;

	motion = Mla::BvhParser::parseBvh(MLA_INPUT_BVH_PATH, "Damien_1_Char00.bvh");

	Mla::MotionOperation::motionFiltering(motion);

	Motion motion_rec;

	Mla::MotionOperation::motionRebuilding(motion, &motion_rec, 20);

	return 0;
}

unsigned int copyFrameTest() {
	Motion* motion = nullptr;

	motion = Mla::BvhParser::parseBvh(MLA_INPUT_BVH_PATH, "Damien_1_Char00.bvh");

	Mla::MotionOperation::motionFiltering(motion);

	Frame* copied_frame = motion->getFrame(2)->duplicateFrame();

	delete copied_frame;

	return 0;
}

unsigned int SpeedDataTest() {
	Motion* motion = nullptr;

	motion = Mla::BvhParser::parseBvh(MLA_INPUT_BVH_PATH, "Damien_2_Char00.bvh");

	Mla::MotionOperation::motionFiltering(motion);

	std::vector<Motion*> motion_vector;

	SegmentationInformation seg_info = {
		30,	// left cut
		30, // right cut
		51, // window size
		3,  // polynom order
		20, // final frame number
	};

	Mla::MotionOperation::MotionSegmentation(motion, seg_info, motion_vector, JOINT_OF_INTEREST);

	// To macro
	std::string subfolder_name = "SUBFOLDER_NONE";
	std::string filename = "FILENAME_NONE";
	std::string folder_name = "DAMIEN_SEGMENTED";

	// Code has changed, not working anymore
	/*for (unsigned int i = 0; i < motion_vector.size(); i++) {
		SpeedData speed_data(motion_vector[i]->getFrames().size() - 1, 
							 motion_vector[i]->getFrameTime(), 
							 motion_vector[i]->getFrames().size(),
							 motion_vector[i]);
		
		std::vector<std::map<std::string, double>> speed_set;

		speed_data.getSpeedSetVector(speed_set);

		subfolder_name = "SEGMENT_" + std::to_string(i);

		for (unsigned int j = 0; j < speed_set.size(); j++) {
			filename = "lin_speed_" + std::to_string(j);

			Mla::CsvExport::ExportData(speed_set[i], folder_name, subfolder_name, filename);
		}
	}*/
	return 0;
}

unsigned int OneMotionExportTest(std::string& motion_folder_name, std::string& motion_name) {
	Motion* motion = nullptr;

	motion = Mla::BvhParser::parseBvh(motion_folder_name, motion_name);

	if (motion == nullptr) {
		std::cout << "Error reading motion (make sure the name and folder are correct)." << std::endl;
		std::cout << "File name: " << motion_name << std::endl;
		std::cout << "Folder name: " << motion_folder_name << std::endl;
		return 1;
	}

	Mla::MotionOperation::motionFiltering(motion);

	std::vector<Motion*> motion_vector;

	SegmentationInformation seg_info = {
		0,	// left cut
		0,  // right cut
		51, // window size
		3,  // polynom order
		20, // final frame number
	};

	seg_info.final_interframe_time = motion->getFrames().size() * motion->getFrameTime() / static_cast<double>(seg_info.final_frame_number - 1);

	Mla::MotionOperation::MotionSegmentation(motion, seg_info, motion_vector, JOINT_OF_INTEREST);

	std::vector<SpeedData> speed_data_set;

	Mla::MotionOperation::ComputeSpeedData(motion_vector, speed_data_set);

	// Name of the motion (Damien_2_Char00_SEGMENTED)
	std::string folder_name = motion_name.std::string::substr(0, motion_name.size() - 4) + "_BATCH_TEST";
	// Name of the segmentation (NB_SEG_X)
	std::string subfolder_name = "NB_SEG_";
	// Name of the lin_speed file (lin_speed_x)
	std::string file_name = "lin_speed_";
	
	Mla::CsvExport::ExportData(speed_data_set, motion->getMotionInformation(), seg_info, folder_name, subfolder_name, file_name);
	
	for (unsigned int i = 0; i < motion_vector.size(); i++)
		delete motion_vector[i];

	delete motion;

	return 0;
}

unsigned int MultipleMotionExportTest() {
	std::vector<std::string> file_names;
	std::string folder_name = "C:\\Users\\quentin\\Documents\\Programmation\\C++\\MLA\\Data\\Bvh\\batch_test_Guillaume\\";
	Mla::Utility::readDirectory(folder_name, file_names);

	for (std::vector<std::string>::iterator it = file_names.begin(); it != file_names.end(); it++) {
		std::cout << "Processing " << *it << std::endl;
		OneMotionExportTest(folder_name, *it);
	}
		
	return 0;
}

// Be careful: 0 cut is hardcoded here (data.insertNewData("Speedx", speed_data_set[0].getAllValues());)
//                                                                         here
unsigned int DataClassTest(std::string& motion_folder_name, std::string& motion_name) {
	Motion* motion = nullptr;

	motion = Mla::BvhParser::parseBvh(motion_folder_name, motion_name);

	if (motion == nullptr) {
		std::cout << "Error reading motion (make sure the name and folder are correct)." << std::endl;
		std::cout << "File name: " << motion_name << std::endl;
		std::cout << "Folder name: " << motion_folder_name << std::endl;
		return 1;
	}

	Mla::MotionOperation::motionFiltering(motion);

	std::vector<Motion*> motion_vector;

	SegmentationInformation seg_info = {
		0,	// left cut
		0,  // right cut
		51, // window size
		3,  // polynom order
		20, // final frame number
	};

	seg_info.final_interframe_time = motion->getFrames().size() * motion->getFrameTime() / static_cast<double>(seg_info.final_frame_number - 1);

	Mla::MotionOperation::MotionSegmentation(motion, seg_info, motion_vector, JOINT_OF_INTEREST);

	std::vector<SpeedData> speed_data_set;

	Mla::MotionOperation::ComputeSpeedData(motion_vector, speed_data_set);

	// Name of the motion (-4, so that '.bvh' is erased)
	std::string folder_name = motion_name.std::string::substr(0, motion_name.size() - 4) + "_JSON_BATCH_TEST";
	// Name of the segmentation (NB_SEG_X) HARDCODED FOR THE MOMENT
	std::string subfolder_name = "NB_SEG_0";
	// Name of the lin_speed file (-4, so that '.bvh' is erased)
	std::string file_name = motion_name.std::string::substr(0, motion_name.size() - 4);

	Mla::JsonExport::ExportMotionInformations(motion->getMotionInformation(), folder_name, "motion_information");
	Mla::JsonExport::ExportMotionSegmentationInformations(seg_info, folder_name, "segmentation_information");

	Data data;
	data.insertNewData("Speed", speed_data_set[0].getAllValues());

	speed_data_set.clear();
	Mla::MotionOperation::ComputeSpeedAxis(motion_vector, speed_data_set, "x", true);
	data.insertNewData("Speedx", speed_data_set[0].getAllValues());
	speed_data_set.clear();
	Mla::MotionOperation::ComputeSpeedAxis(motion_vector, speed_data_set, "y", true);
	data.insertNewData("Speedy", speed_data_set[0].getAllValues());
	speed_data_set.clear();
	Mla::MotionOperation::ComputeSpeedAxis(motion_vector, speed_data_set, "z", true);
	data.insertNewData("Speedz", speed_data_set[0].getAllValues());
	
	std::vector<std::map<std::string, double>> values;
	Mla::MotionOperation::JESAISPASQUOI(motion, seg_info, JOINT_OF_INTEREST, values);
	data.insertNewData("DiffSpeed", values);

	Mla::JsonExport::ExportData(data, folder_name, subfolder_name, file_name);

	for (unsigned int i = 0; i < motion_vector.size(); i++)
		delete motion_vector[i];

	delete motion;

	return 0;
}

unsigned int FullDataTest() {
	std::vector<std::string> file_names;
	std::string folder_name = "C:\\Users\\quentin\\Documents\\Programmation\\C++\\MLA\\Data\\Bvh\\batch_test_Damien\\";
	Mla::Utility::readDirectory(folder_name, file_names);

	for (std::vector<std::string>::iterator it = file_names.begin(); it != file_names.end(); it++) {
		std::cout << "Processing " << *it << std::endl;
		DataClassTest(folder_name, *it);
	}

	return 0;
}

unsigned int JESAISPASQUOITEST() {
	
	Motion* motion = nullptr;

	std::string motion_name = "Guillaume_1_Char00.bvh";
	std::string motion_folder_name = "C:\\Users\\quentin\\Documents\\Programmation\\C++\\MLA\\Data\\Bvh\\batch_test_Guillaume\\";

	motion = Mla::BvhParser::parseBvh(motion_folder_name, motion_name);

	if (motion == nullptr) {
		std::cout << "Error reading motion (make sure the name and folder are correct)." << std::endl;
		std::cout << "File name: " << motion_name << std::endl;
		std::cout << "Folder name: " << motion_folder_name << std::endl;
		return 1;
	}

	Mla::MotionOperation::motionFiltering(motion);

	std::vector<Motion*> motion_vector;

	SegmentationInformation seg_info = {
		0,	// left cut
		0,  // right cut
		51, // window size
		3,  // polynom order
		20, // final frame number
	};

	seg_info.final_interframe_time = motion->getFrames().size() * motion->getFrameTime() / static_cast<double>(seg_info.final_frame_number - 1);

	std::vector<std::map<std::string, double>> values;

	Mla::MotionOperation::JESAISPASQUOI(motion, seg_info, JOINT_OF_INTEREST, values);
}

unsigned int GlmFunctionsTest() {
	glm::dvec3 vec(10, 5, 2);
	glm::dvec3 normalised = glm::normalize(vec);
	std::cout << "Distance: " << glm::distance(glm::dvec3(0, 0, 0), vec) << std::endl;
	std::cout << "Original vec\nx: " << vec.x << "\ny: " << vec.y << "\nz: " << vec.z << std::endl;
	std::cout << "Normalised vec\nx: " << normalised.x << "\ny: " << normalised.y << "\nz: " << normalised.z << std::endl;

	return 0;
}

unsigned int MemoryLeakChaser(){
	Motion* motion = nullptr;
	std::string name = "apur.bvh";

	std::vector<std::map<std::string, double>> lin_speed;

	for (unsigned int i=0 ; i<100 ; i++) {
		std::cout << std::endl << std::endl << "Processing file " << name << std::endl;
		motion = Mla::BvhParser::parseBvh(MLA_INPUT_BVH_PATH, name);

		if (motion == nullptr) {
			std::cout << "Failed to parse bvh file (is the path/namefile correct ?)" << std::endl;
			std::cout << "Press ENTER to quit...";
			std::cin.get();
			return 1;
		}

		lin_speed.clear();
		Mla::MotionOperation::MeanLinearSpeed(lin_speed, motion, 10);

		delete motion;

	}	

	return 0;
}