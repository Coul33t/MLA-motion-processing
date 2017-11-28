#include "MLA.h"

int main(int argc, char *argv[]) {

	Motion* motion = Mla::BvhParser::parseBvh(MLA_INPUT_BVH_PATH, "Damien_2_Char00.bvh");

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
	Mla::MotionOperation::MotionSegmentation(motion, 2, 2, 51, 3, 20, seg_motion);

	window.MainLoop(seg_motion[2]);*/
	window.MainLoop(motion);

	/*std::cout << "Press any key to quit...";
	std::cin.get();*/
	return 0;
}