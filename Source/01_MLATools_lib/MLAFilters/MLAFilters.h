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

namespace filters {
	std::vector<double> MeanShift(std::vector<double>, unsigned int);

};


#endif //__MLA_FILTERS_H__
