#include "MLA.h"

int main(int argc, char *argv[]) {

	Motion* motion = Mla::BvhParser::parseBvh(MLA_INPUT_BVH_PATH "batch_test_Damien/", "Damien_1Char00.bvh");

	if(motion == false) {
		std::cout << "Failed to parse bvh file (is the path/namefile correct ?)" << std::endl;
		std::cout << "Press ENTER to quit...";
		std::cin.get();
		return 1;
	}
	
	MainWindow window;

	if (window.WindowInit() == false) {
		std::cout << "Failed to init window." << std::endl;
		std::cout << "Press ENTER to quit...";
		std::cin.get();
		return 1;
	}
		
	if (window.GLinit() == false) {
		std::cout << "Failed to init GL." << std::endl;
		std::cout << "Press ENTER to quit...";
		std::cin.get();
		return 1;
	}
		

	if (window.LoadShader(MLA_INPUT_SHADER_PATH "couleur3D.vert", MLA_INPUT_SHADER_PATH "couleur3D.frag") == false) {
		std::cout << "Failed to load shaders." << std::endl;
		std::cout << "Press ENTER to quit...";
		std::cin.get();
		return 1;

	}
		
	Mla::MotionOperation::motionFiltering(motion);

	/*std::vector<Motion*> seg_motion;
	SegmentationInformation seg_info = {
		2,	// left cut
		2,  // right cut
		51, // window size
		3,  // polynom order
		20, // final frame number
	};
	Mla::MotionOperation::MotionSegmentation(motion, seg_info, seg_motion);

	window.MainLoop(seg_motion[0]);*/
	window.MainLoop(motion);

	/*std::cout << "Press any key to quit...";
	std::cin.get();*/
	return 0;
}