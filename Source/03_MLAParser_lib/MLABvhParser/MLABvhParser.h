/**
Class for a bvh parser.
MLABvhParser.h

@author Quentin Couland
@version 0.1

*/

#ifndef __MLA_BVH_PARSER__
#define __MLA_BVH_PARSER__

#include "MLAMotion_lib.h"

namespace Mla {
	namespace BvhParser {

		std::unique_ptr<Motion> parseBvh(const std::string&, const std::string&);

	};
};

#endif //__MLA_BVH_PARSER__