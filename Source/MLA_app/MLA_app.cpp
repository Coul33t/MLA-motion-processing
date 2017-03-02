#include "MLA.h"

int main(int argc, char *argv[]) {
	
	BvhParser parser;
	Motion* motion = parser.parseBvh(MLA_INPUT_BVH_PATH "test.bvh");
	
	MainWindow window;

	if (window.WindowInit() == false)
		return 1;

	if (window.GLinit() == false)
		return 1;

	if (window.LoadShader(MLA_INPUT_SHADER_PATH "couleur3D.vert", MLA_INPUT_SHADER_PATH "couleur3D.frag") == false)
		return 1;

	window.MainLoop(motion);

	/*std::cout << "Success. Press any key to quit...";
	std::cin.get();*/
	return 0;
}