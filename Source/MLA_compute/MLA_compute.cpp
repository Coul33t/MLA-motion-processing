#include "MLA.h"

int main(int argc, char *argv[]) {
	BvhParser parser;

	Motion* motion = parser.parseBvh(MLA_INPUT_BVH_PATH, "throw_10_gimbal_smooth_16.bvh");

	if(motion == false) {
		std::cout << "Failed to parse bvh file (is the path/namefile correct ?)" << std::endl;
		std::cout << "Press ENTER to quit...";
		std::cin.get();
		return 1;
	}

	MotionOperation motionOp;

	motionOp.motionFiltering(motion);

	for (unsigned int i = 1; i<motion->getFrames().size() - 1; i++) {
		std::string linName = "lin_full_" + std::to_string(i);
		std::string angName = "ang_full_" + std::to_string(i);

		std::string linFolder = "lin\\";
		std::string angFolder = "ang\\";

		CSVExport::ExportData(motionOp.jointsLinearSpeed(motion->getFrame(i), motion->getFrame(i + 1), motion->getFrameTime()), motion->getName(), linFolder, linName);
		CSVExport::ExportData(motionOp.jointsAngularSpeed(motion->getFrame(i), motion->getFrame(i + 1), motion->getFrameTime()), motion->getName(), angFolder, angName);
	}



	/*std::cout << "Press any key to quit...";
	std::cin.get();*/
	return 0;
}