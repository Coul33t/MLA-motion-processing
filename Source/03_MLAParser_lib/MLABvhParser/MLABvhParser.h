/**
Class for a bvh parser.
MLABvhParser.h

@author Quentin Couland
@version 0.1

*/

#ifndef __MLA_BVH_PARSER__
#define __MLA_BVH_PARSER__

#include "MLAMotion_lib.h"

class BvhParser {

public:
	BvhParser();
	bool parseBvh(const std::string&);
	bool searchForward(std::ifstream&, const std::string);
	bool isDouble(const char* str);

private:

};

#endif //__MLA_BVH_PARSER__