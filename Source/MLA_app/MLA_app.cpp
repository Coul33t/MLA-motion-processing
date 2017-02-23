#include "MLA.h"

int main(int argc, char *argv[]) {
	
	BvhParser parser;
	Motion* motion = parser.parseBvh(MLA_INPUT_BVH_PATH "SIG_S2_X04_hips_only_10_frames.bvh");
	

	std::cout << "Success. Press any key to quit...";
	std::cin.get();
	return 0;
}