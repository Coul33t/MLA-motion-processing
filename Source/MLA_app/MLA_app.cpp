#include "MLA.h"

int main() {

	Motion* motion = bvhparser::parseBvh(MLA_INPUT_BVH_PATH, "throw_5_gimbal_smooth_16.bvh");

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
		

	window.MainLoop(motion);

	/*std::cout << "Press any key to quit...";
	std::cin.get();*/
	return 0;
}