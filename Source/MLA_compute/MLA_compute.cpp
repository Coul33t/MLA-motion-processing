// WARNING : this file is a mess. I'm using it to test my functions,
// so old code in it (i.e. anything older than the last function created)
// probably won't work anymore.

#include "MLA.h"
#include <algorithm>

// TODO: automatically find the joint
//       with the highest speed
#define JOINT_OF_INTEREST "RightHand"

Data ExtractDarts(std::shared_ptr<Motion>,
				  SegmentationInformation&, 
				  const std::string&,
				  std::map<std::string, std::vector<std::string>>&,
				  const std::vector<std::pair<std::string, std::string>>&);

unsigned int DartsExtraction(const std::string&, const std::string&, 
							 const std::string&, const std::string&, 
							 std::vector<std::string>&,
							 std::map<std::string, std::vector<std::string>>&,
							 std::vector<std::pair<std::string, std::string>>&,
							 std::string&);

unsigned int DartsExtractionFromFolder(const std::string&, const std::string&, const std::string&);

unsigned int GlmFunctionsTest();
unsigned int MemoryLeakChaser();

int main(int argc, char *argv[]) {	
	std::string input_folder = "E:/Users/quentin/Documents/Programmation/C++/MLA-motion-processing/Data/bvh_test/input/";
	std::string output_folder = "E:/Users/quentin/Documents/Programmation/C++/MLA-motion-processing/Data/bvh_test/output/";

	std::string laterality = "Right";

	DartsExtractionFromFolder(input_folder, output_folder, laterality);

	std::cout << std::endl << "Press any key to quit...";
	
	std::cin.get();
	return 0;
}

Data ExtractDarts(std::shared_ptr<Motion> motion, SegmentationInformation& seg_info, const std::string& joint_to_segment,
	std::map<std::string, std::vector<std::string>>& KCs, const std::vector<std::pair<std::string, 
	std::string>>& distances_to_compute) {

	std::cout << "Motion filtering..." << std::endl;
	Mla::MotionOperation::motionFiltering(motion);

	std::shared_ptr<Motion> segmented_motion = std::make_shared<Motion>();

	seg_info.final_frame_number = motion->getFrames().size();
	seg_info.final_interframe_time = motion->getFrames().size() * motion->getFrameTime() / (static_cast<double>(seg_info.final_frame_number) - 1);

	// Computing speed
	std::cout << "Speed computing..." << std::endl;

	SpeedData speed_data(motion->getFrames().size() - 1,
		motion->getFrameTime(),
		motion->getFrames().size());

	Mla::MotionOperation::motionSpeedComputing(motion, speed_data);

	std::vector<std::map<std::string, double>> values_to_store = std::vector<std::map<std::string, double>>();

	// Instanciating the data class, which will hold all the data (wow rly?)
	Data data;

	// Experimental (setting the savgol window size)
	seg_info.savgol_window_size = seg_info.final_frame_number / 4;

	// Window size must be odd
	if (seg_info.savgol_window_size % 2 == 0)
		seg_info.savgol_window_size++;

	// Computing the savgol on the data
	std::cout << "Savgol computing..." << std::endl;
	Mla::MotionOperation::ComputeSavgol(speed_data, seg_info);

	// --------------------------------------------
	// From here, the speed_data is now savgoled,
	// which means that every subsequent operation
	// is done on a savgoled signal
	// --------------------------------------------

	// Extracting the norm of the speed
	std::cout << "Mean Speed extraction..." << std::endl;
	std::map<std::string, double> tmp_vec;
	speed_data.getMeanSpeed(tmp_vec);
	values_to_store.push_back(tmp_vec);
	data.insertNewData("MeanSpeed", values_to_store);

	values_to_store.clear();

	// Extracting the norm of the speed
	speed_data.getNorm(values_to_store);
	data.insertNewData("SpeedNorm", values_to_store);

	// Storing x speed and directions values (savgoled)
	speed_data.getAllValues(values_to_store, "x", false);
	data.insertNewData("Speedx", values_to_store);
	speed_data.getAllValues(values_to_store, "x", true);
	data.insertNewData("SpeedDirx", values_to_store);

	// Storing y speed and directions values (savgoled)
	speed_data.getAllValues(values_to_store, "y", false);
	data.insertNewData("Speedy", values_to_store);
	speed_data.getAllValues(values_to_store, "y", true);
	data.insertNewData("SpeedDiry", values_to_store);

	// Storing z speed and directions values (savgoled)
	speed_data.getAllValues(values_to_store, "z", false);
	data.insertNewData("Speedz", values_to_store);
	speed_data.getAllValues(values_to_store, "z", true);
	data.insertNewData("SpeedDirz", values_to_store);

	// BOUNDING BOX
	std::map<std::string, double> norm_map;

	std::cout << "Bounding Box computing..." << std::endl;
	std::vector<std::map<std::string, std::vector<double>>> bb;

	Mla::MotionOperation::computeFinalBoudingBox(motion, bb, KCs.at("arm_KC"), true);

	Mla::Utility::ExtractComponent(bb, values_to_store, 0);
	data.insertNewData("BoundingBoxMinusX", values_to_store);
	Mla::Utility::ExtractComponent(bb, values_to_store, 1);
	data.insertNewData("BoundingBoxPlusX", values_to_store);
	Mla::Utility::ExtractComponent(bb, values_to_store, 2);
	data.insertNewData("BoundingBoxMinusY", values_to_store);
	Mla::Utility::ExtractComponent(bb, values_to_store, 3);
	data.insertNewData("BoundingBoxPlusY", values_to_store);
	Mla::Utility::ExtractComponent(bb, values_to_store, 4);
	data.insertNewData("BoundingBoxMinusZ", values_to_store);
	Mla::Utility::ExtractComponent(bb, values_to_store, 5);
	data.insertNewData("BoundingBoxPlusZ", values_to_store);

	// POSITION
	std::cout << "Position extraction..." << std::endl;
	std::vector<std::map<std::string, glm::dvec3>> pos_values;
	std::string abs_or_rel = "abs";

	Mla::MotionOperation::computePositionBeforeThrow(motion, seg_info, joint_to_segment, pos_values, KCs.at("hand_pos_KC"));

	Mla::Utility::ExtractComponent(pos_values, values_to_store, 0);
	data.insertNewData("PosX", values_to_store);
	Mla::Utility::ExtractComponent(pos_values, values_to_store, 1);
	data.insertNewData("PosY", values_to_store);
	Mla::Utility::ExtractComponent(pos_values, values_to_store, 2);
	data.insertNewData("PosZ", values_to_store);

	std::cout << "Distances computation..." << std::endl;
	Mla::MotionOperation::computeDistancesBeforeThrow(motion, seg_info, joint_to_segment, values_to_store, distances_to_compute, KCs.at("new_arm_KC"), true);
	data.insertNewData("DistanceNorm", values_to_store);

	std::vector<std::map<std::string, std::vector<double>>> distance_values;
	Mla::MotionOperation::computeDistancesAxisBeforeThrow(motion, seg_info, joint_to_segment, distance_values, distances_to_compute, KCs.at("new_arm_KC"), true);
	Mla::Utility::ExtractComponent(distance_values, values_to_store, 0);
	data.insertNewData("DistanceX", values_to_store);
	Mla::Utility::ExtractComponent(distance_values, values_to_store, 1);
	data.insertNewData("DistanceY", values_to_store);
	Mla::Utility::ExtractComponent(distance_values, values_to_store, 2);
	data.insertNewData("DistanceZ", values_to_store);

	// TODO: REDO THE COMPUTEBOUNDINGBOXWIDTH
	std::map<std::string, double> bb_mean_and_std;

	std::string final_name = "";
	for (auto it = KCs.at("new_arm_KC").begin(); it != KCs.at("new_arm_KC").end(); it++) {
		final_name += (*it);
	}

	// TODO: change the way it works so that we won't have to do these extra steps to insert the datas
	Mla::MotionOperation::computeBoundingBoxWidth(motion, bb_mean_and_std, KCs.at("new_arm_KC"));
	values_to_store.clear();
	std::map<std::string, double> tmp;
	tmp[final_name] = bb_mean_and_std["mean"];
	values_to_store.push_back(tmp);
	data.insertNewData("BoundingBoxWidthMean", values_to_store);
	values_to_store.clear();
	tmp.clear();
	tmp[final_name] = bb_mean_and_std["std"];
	values_to_store.push_back(tmp);
	data.insertNewData("BoundingBoxWidthStd", values_to_store);

	std::cout << "Data export..." << std::endl;

	//delete segmented_motion;

	return data;
}

unsigned int DartsExtraction(const std::string& motion_folder_name, const std::string& motion_name,
							 const std::string& output_folder, const std::string& joint_to_segment,
							 std::map <std::string, std::vector<std::string>>& KCs,
							 std::vector<std::pair<std::string, std::string>> distances_to_compute,
							 const std::string& laterality) {
	
	std::shared_ptr<Motion> motion = nullptr;

	motion = Mla::BvhParser::parseBvh(motion_folder_name, motion_name);

	if (motion == nullptr) {
		std::cout << "Error reading motion (make sure the name and folder are correct)." << std::endl;
		std::cout << "File name: " << motion_name << std::endl;
		std::cout << "Folder name: " << motion_folder_name << std::endl;
		std::cout << "Final path: " << motion_folder_name << "(/)" << motion_name << std::endl;
		return 1;
	}
	
	SegmentationInformation seg_info = {
		0,	// left cut
		0,  // right cut
		51, // window size
		3,  // polynom order
		20, // final frame number
	};


	Data data = ExtractDarts(motion, seg_info, joint_to_segment, KCs, distances_to_compute);

	// Name of the motion -4, so that '.bvh' is erased
	std::string folder_name = output_folder + motion_name.std::string::substr(0, motion_name.size() - 4);

	std::string subfolder_name = "data/";
	// Name of the  file : motion name -4, so that '.bvh' is erased
	std::string file_name = motion_name.std::string::substr(0, motion_name.size() - 4);

	Mla::JsonExport::ExportMotionInformations(motion->getMotionInformation(), motion->getFrame(0)->getJointsName(), folder_name, "motion_information");
	Mla::JsonExport::ExportMotionSegmentationInformations(seg_info, folder_name, "segmentation_information");
	Mla::JsonExport::ExportData(data, folder_name + "/" + subfolder_name, file_name);

	//delete motion;
	return 0;
}

unsigned int DartsExtractionFromFolder(const std::string& input_folder, const std::string& output_folder,
									   const std::string& laterality) {
	std::vector<std::string> file_names;
	Mla::Utility::readDirectory(input_folder, file_names);

	std::vector<std::string> laterality_joints;
	laterality_joints.push_back("Shoulder");
	laterality_joints.push_back("Arm");
	laterality_joints.push_back("ForeArm");
	laterality_joints.push_back("Hand");

	// BE CAREFUL TO PUT THE JOINTS IN THE RIGHT ORDER (FROM ROOT TO EXTREMITY)
	// FOR BOUNDING BOX NORMALISATION (the algorithm takes the front and back
	// values of the vector as the root and the extremity respectively)
	std::map<std::string, std::vector<std::string>> algos_KC;
	std::vector<std::string> arm_KC;
	arm_KC.push_back("Shoulder");
	arm_KC.push_back("Arm");
	arm_KC.push_back("ForeArm");
	arm_KC.push_back("Hand");
	algos_KC.insert(std::pair<std::string, std::vector<std::string>>("arm_KC", arm_KC));

	std::vector<std::string> new_arm_KC;
	new_arm_KC.push_back("Head");
	new_arm_KC.push_back("Shoulder");
	new_arm_KC.push_back("Arm");
	new_arm_KC.push_back("ForeArm");
	new_arm_KC.push_back("Hand");
	algos_KC.insert(std::pair<std::string, std::vector<std::string>>("new_arm_KC", new_arm_KC));

	std::vector<std::string> hand_pos_KC;
	hand_pos_KC.push_back("Head");
	hand_pos_KC.push_back("Neck");
	hand_pos_KC.push_back("Shoulder");
	hand_pos_KC.push_back("Arm");
	hand_pos_KC.push_back("ForeArm");
	hand_pos_KC.push_back("Hand");
	algos_KC.insert(std::pair<std::string, std::vector<std::string>>("hand_pos_KC", hand_pos_KC));

	std::vector<std::pair<std::string, std::string>> distances_to_compute;
	distances_to_compute.push_back(std::pair<std::string, std::string>("Hand", "Head"));

	// Speed extraction and sub-motion cut
	std::string joint_to_segment = "Hand";

	joint_to_segment = laterality + joint_to_segment;

	for (auto it = distances_to_compute.begin(); it != distances_to_compute.end(); ++it) {
		if (std::find(laterality_joints.begin(), laterality_joints.end(), (*it).first) != laterality_joints.end())
			(*it).first = laterality + (*it).first;
		if (std::find(laterality_joints.begin(), laterality_joints.end(), (*it).second) != laterality_joints.end())
			(*it).second = laterality + (*it).second;
	}

	for (auto it = algos_KC.begin(); it != algos_KC.end(); ++it) {
		for (auto i = 0; i < algos_KC[(it)->first].size(); i++) {
			if (std::find(laterality_joints.begin(), laterality_joints.end(), algos_KC[it->first][i]) != laterality_joints.end())
				algos_KC[it->first][i] = laterality + algos_KC[it->first][i];
		}
	}

	for (auto it = file_names.begin(); it != file_names.end(); it++) {
		std::cout << "\n\n\nProcessing " << *it << std::endl;
		DartsExtraction(input_folder, *it, output_folder, joint_to_segment, algos_KC, distances_to_compute, laterality);
	}

	return 0;
}

unsigned int GlmFunctionsTest() {
	glm::dvec3 vec(10, 5, 2);
	glm::dvec3 normalised = glm::normalize(vec);
	std::cout << "Distance: " << glm::distance(glm::dvec3(0, 0, 0), vec) << std::endl;
	std::cout << "Original vec\nx: " << vec.x << "\ny: " << vec.y << "\nz: " << vec.z << std::endl;
	std::cout << "Normalised vec\nx: " << normalised.x << "\ny: " << normalised.y << "\nz: " << normalised.z << std::endl;
	std::cout << "Length: " << glm::length(vec) << std::endl;
	std::cout << "Normalized norm: " << glm::length(normalised) << std::endl;
	// It works (vec = (13,8,5))
	vec += 3;
	std::cout << "Test vec\nx: " << vec.x << "\ny: " << vec.y << "\nz: " << vec.z << std::endl;
	// It works (vec = (6.5,4,2.5))
	vec /= 2;
	std::cout << "Test vec\nx: " << vec.x << "\ny: " << vec.y << "\nz: " << vec.z << std::endl;
	return 0;
}

unsigned int MemoryLeakChaser(){
	std::shared_ptr<Motion> motion = nullptr;
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

		//delete motion;

	}	

	return 0;
}