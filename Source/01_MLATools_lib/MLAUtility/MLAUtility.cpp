#include "MLAUtility.h"

namespace Mla {
	namespace Utility {

		double vectorLength(glm::dvec3 p1, glm::dvec3 p2) {
			return sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2) + pow(p2.z - p1.z, 2));
		}

		double roundToDecimal(double number, unsigned int decimal) {
			return (number * pow(10, decimal)) / pow(10, decimal);
		}

		void ExtractComponent(std::vector<std::map<std::string, glm::dvec3>>& vec_3d, std::vector<std::map<std::string, double>>& vec_extracted, unsigned int idx) {
			vec_extracted.clear();

			std::map<std::string, double> copy_map;
			
			// For each map
			for (auto it = vec_3d.begin(); it != vec_3d.end(); ++it) {
				
				copy_map.clear();

				// For each joint
				for (auto& kv: *it) {
					copy_map[kv.first] = kv.second[idx];
				}

				vec_extracted.push_back(copy_map);
			}
		}

		void ExtractComponent(std::vector<glm::dvec3>& vec_3d, std::vector<double>& extracted_component, unsigned int idx) {
			extracted_component.clear();

			for (auto it = vec_3d.begin(); it != vec_3d.end(); ++it) {
				extracted_component.push_back((*it)[idx]);
			}
		}

		void ExtractComponent(std::vector<std::map<std::string, std::vector<double>>>& vec_input, std::vector<std::map<std::string, double>>& vec_extracted, unsigned int idx) {
			vec_extracted.clear();

			std::map<std::string, double> copy_map;
			// For each map
			for (auto it = vec_input.begin(); it != vec_input.end(); ++it) {

				copy_map.clear();

				// For each joint
				for (auto& kv : *it) {
					copy_map[kv.first] = kv.second[idx];
				}

				vec_extracted.push_back(copy_map);
			}
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

		/** Creates a folder recursively.
		* @param path The path to create
		* @return 0 if all went well, 1 if the creation failed		
		*/
		bool createDirectoryRecursively(std::string path) {
			std::vector<std::string> sub_path;
			std::string token;
			size_t pos = 0;
			const std::string delimiter = "/";
			std::string path_to_create = "";

			while (path.length() > 0) {
				token = path.substr(0, path.find(delimiter));
				sub_path.push_back(token);
				pos = token.length();
				path.erase(0, pos + delimiter.length());
				path_to_create = "";
				for (auto it = sub_path.begin(); it != sub_path.end(); it++) {
					path_to_create += (*it) + "/";
				}
				if (!dirExists(path_to_create)) {
					if (!CreateDirectory(path_to_create.c_str(), NULL)) {
						std::cout << "ERROR creating folder " << path_to_create << std::endl;
						if (GetLastError() == ERROR_ALREADY_EXISTS)
							std::cout << "WARNING: ALREADY EXISTS" << std::endl;
						else if (GetLastError() == ERROR_PATH_NOT_FOUND)
							std::cout << "ERROR: PATH NOT FOUND" << std::endl;
						else if (GetLastError() == ERROR_ACCESS_DENIED)
							std::cout << "ERROR: ACCESS DENIED" << std::endl;
						else
							std::cout << "ERROR: UNKNOWN ERROR" << std::endl;
						return false;
					}
				}
			}

			return true;
		}

		// See https://stackoverflow.com/questions/8233842/how-to-check-if-directory-exist-using-c-and-winapi
		bool dirExists(const std::string& dirName_in) {
			DWORD ftyp = GetFileAttributesA(dirName_in.c_str());
			if (ftyp == INVALID_FILE_ATTRIBUTES)
				return false;  //something is wrong with your path!

			if (ftyp & FILE_ATTRIBUTE_DIRECTORY)
				return true;   // this is a directory!

			return false;    // this is not a directory!
		}
	}
}