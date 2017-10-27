#include "MLAUtility.h"

namespace mla {
	namespace utility {

		double vectorLength(glm::dvec3 p1, glm::dvec3 p2) {
			return sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2) + pow(p2.z - p1.z, 2));
		}

		double roundToDecimal(double number, unsigned int decimal) {
			return (number * pow(10, decimal)) / pow(10, decimal);
		}

	}
}