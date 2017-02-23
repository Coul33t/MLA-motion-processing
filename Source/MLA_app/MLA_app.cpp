#include "MLA.h"

int main(int argc, char *argv[]) {
	Motion testMotion;
	BvhParser parser;
	parser.parseBvh(MLA_INPUT_BVH_PATH "SIG_S2_X04_hips_only.bvh");
	std::cout << "Success. Press any key to quit...";
	std::cin.get();
	return 0;
}