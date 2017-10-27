#include "CSVExport.h"

//TODO: use pipe for communication
namespace mla {
	namespace csvexport {

		/** Export a map to a csv file

		@param data the data
		@param name the name of the output file (default : "default")

		@return bool success of file opening

		*/
		bool ExportData(std::map<std::string, double> data, std::string motionName, std::string folder, std::string name) {
			// 2 times, because the function can only create a directory, not the potential subdirectories.
			if (CreateDirectory(("..\\..\\..\\Data\\Speed\\" + motionName).c_str(), NULL) || GetLastError() == ERROR_ALREADY_EXISTS) {

				if (CreateDirectory(("..\\..\\..\\Data\\Speed\\" + motionName + "\\" + folder).c_str(), NULL) || GetLastError() == ERROR_ALREADY_EXISTS) {
					std::ofstream outfile;

					// Wipe any pre-existing file.
					outfile.open("../../../Data/Speed/" + motionName + "/" + folder + "/" + name + ".csv", std::ios::out);
					outfile.close();

					outfile.open("../../../Data/Speed/" + motionName + "/" + folder + "/" + name + ".csv", std::ios::out | std::ios::app);

					if (!outfile.fail()) {

						for (std::map<std::string, double>::iterator it = data.begin(); it != data.end(); ++it) {
							outfile << it->first << "," << it->second << '\n';
						}

						outfile.close();
						return true;
					}

					else {
						std::cout << "Failed to open file " + name + ".csv" << std::endl;
						return false;
					}
				}

				else {
					std::cout << "Failed to create the folder " + folder << std::endl;
					return false;
				}
			}

			else {
				std::cout << "Failed to create the folder " + motionName << std::endl;
				return false;
			}


		}

		/** Export a vector to a csv file

		@param data the data
		@param name the name of the output file (default : "default")

		@return bool success of file opening

		*/
		bool ExportData(std::vector<double> data, std::string motionName, std::string folder, std::string name) {
			// 2 times, because the function can only create a directory, not the potential subdirectories.
			if (CreateDirectory(("..\\..\\..\\Data\\Speed\\" + motionName).c_str(), NULL) || GetLastError() == ERROR_ALREADY_EXISTS) {

				if (CreateDirectory(("..\\..\\..\\Data\\Speed\\" + motionName + "\\" + folder).c_str(), NULL) || GetLastError() == ERROR_ALREADY_EXISTS) {
					std::ofstream outfile;

					// Wipe any pre-existing file.
					outfile.open("../../../Data/Speed/" + motionName + "/" + folder + "/" + name + ".csv", std::ios::out);
					outfile.close();

					outfile.open("../../../Data/Speed/" + motionName + "/" + folder + "/" + name + ".csv", std::ios::out | std::ios::app);

					if (!outfile.fail()) {

						for (std::vector<double>::iterator it = data.begin(); it != data.end(); ++it) {
							outfile << *it << '\n';
						}

						outfile.close();
						return true;
					}

					else {
						std::cout << "Failed to open file " + name + ".csv" << std::endl;
						return false;
					}
				}

				else {
					std::cout << "Failed to create the folder " + folder << std::endl;
					return false;
				}
			}

			else {
				std::cout << "Failed to create the folder " + motionName << std::endl;
				return false;
			}


		}

		void EraseFolderContent(std::string pathname) {

			struct stat info;

			if (stat(pathname.c_str(), &info) != 0)
				printf("cannot access %s\n", pathname);
			else if (info.st_mode & S_IFDIR)  // S_ISDIR() doesn't exist on my windows 
				printf("%s is a directory\n", pathname);
			else
				printf("%s is no directory\n", pathname);
		}

	}
}