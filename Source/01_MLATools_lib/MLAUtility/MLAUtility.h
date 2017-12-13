/**
Utility.h

@author Quentin Couland
@version 0.1

*/

#ifndef __MLA_UTILITY_H__
#define __MLA_UTILITY_H__

#include "MLAMotion\MLAMotion.h"
#include <numeric> // std::accumulate

#include <Windows.h> //readDirectory

namespace Mla {
	namespace Utility {

		double vectorLength(glm::dvec3, glm::dvec3);

		double roundToDecimal(double, unsigned int);

		template <typename T>
		std::pair<T, int> getMaxValue(std::vector<T>&);

		template <typename T>
		std::pair<T, int> getMinValue(std::vector<T>&);

		void readDirectory(const std::string&, std::vector<std::string>&);

	};
};

#include "MLAutility_impl.tcc"
#endif //__MLA_UTILITY_H__