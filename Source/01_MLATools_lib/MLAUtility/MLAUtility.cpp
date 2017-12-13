#include "MLAUtility.h"

namespace Mla {
	namespace Utility {

		double vectorLength(glm::dvec3 p1, glm::dvec3 p2) {
			return sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2) + pow(p2.z - p1.z, 2));
		}

		double roundToDecimal(double number, unsigned int decimal) {
			return (number * pow(10, decimal)) / pow(10, decimal);
		}

		// See http://www.martinbroadhurst.com/list-the-files-in-a-directory-in-c.html
		void readDirectory(const std::string& folder, std::vector<std::string>& file_names_vector) {
			std::string pattern(folder);
			pattern.append("\\*");
			WIN32_FIND_DATA data;
			HANDLE hFind;

			if ((hFind = FindFirstFile(pattern.c_str(), &data)) != INVALID_HANDLE_VALUE) {
				do {
					file_names_vector.push_back(data.cFileName);
				} while (FindNextFile(hFind, &data) != 0);
				FindClose(hFind);
			}
		}

	}
}