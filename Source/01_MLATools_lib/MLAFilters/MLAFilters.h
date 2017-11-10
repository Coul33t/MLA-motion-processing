/**
MotionOperation.h

@author Quentin Couland
@version 0.1

*/

#ifndef __MLA_FILTERS_H__
#define __MLA_FILTERS_H__

#include <iostream>
#include <vector>
#include <algorithm> // std::min

namespace Mla {
	namespace Filters {
		void MeanShift(std::vector<double>&, const std::vector<double>&, unsigned int);
	};
};

#endif //__MLA_FILTERS_H__
