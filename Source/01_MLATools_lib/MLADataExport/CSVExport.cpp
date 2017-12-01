#include "CSVExport.h"

//TODO: use pipe for communication
namespace Mla {
	namespace CsvExport {

		/** Export a map to a csv file

		@param data the data
		@param name the name of the output file (default : "default")

		@return bool success of file opening

		*/
		bool ExportData(const std::map<std::string, double>& data, const std::string& motion_name, const std::string& folder, const std::string& name) {
			// 2 times, because the function can only create a directory, not the potential subdirectories.
			if (CreateDirectory(("..\\..\\..\\Data\\Speed\\" + motion_name).c_str(), NULL) || GetLastError() == ERROR_ALREADY_EXISTS) {

				if (CreateDirectory(("..\\..\\..\\Data\\Speed\\" + motion_name + "\\" + folder).c_str(), NULL) || GetLastError() == ERROR_ALREADY_EXISTS) {
					std::ofstream outfile;

					// Wipe any pre-existing file.
					outfile.open("../../../Data/Speed/" + motion_name + "/" + folder + "/" + name + ".csv", std::ios::out);
					outfile.close();

					outfile.open("../../../Data/Speed/" + motion_name + "/" + folder + "/" + name + ".csv", std::ios::out | std::ios::app);

					if (!outfile.fail()) {

						for (std::map<std::string, double>::const_iterator it = data.begin(); it != data.end(); ++it) {
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
				std::cout << "Failed to create the folder " + motion_name << std::endl;
				return false;
			}


		}

		bool ExportData(const std::vector<double>& data, const std::string& motion_name, const std::string& folder, const std::string& name) {
			// 2 times, because the function can only create a directory, not the potential subdirectories.
			if (CreateDirectory(("..\\..\\..\\Data\\Speed\\" + motion_name).c_str(), NULL) || GetLastError() == ERROR_ALREADY_EXISTS) {

				if (CreateDirectory(("..\\..\\..\\Data\\Speed\\" + motion_name + "\\" + folder).c_str(), NULL) || GetLastError() == ERROR_ALREADY_EXISTS) {
					std::ofstream outfile;

					// Wipe any pre-existing file.
					outfile.open("../../../Data/Speed/" + motion_name + "/" + folder + "/" + name + ".csv", std::ios::out);
					outfile.close();

					outfile.open("../../../Data/Speed/" + motion_name + "/" + folder + "/" + name + ".csv", std::ios::out | std::ios::app);

					if (!outfile.fail()) {

						for (std::vector<double>::const_iterator it = data.begin(); it != data.end(); ++it) {
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
				std::cout << "Failed to create the folder " + motion_name << std::endl;
				return false;
			}


		}

		bool ExportData(const std::vector<std::pair<int, int>>& data, const std::string& motion_name, const std::string& folder, const std::string& name) {
			// 2 times, because the function can only create a directory, not the potential subdirectories.
			if (CreateDirectory(("..\\..\\..\\Data\\Speed\\" + motion_name).c_str(), NULL) || GetLastError() == ERROR_ALREADY_EXISTS) {

				if (CreateDirectory(("..\\..\\..\\Data\\Speed\\" + motion_name + "\\" + folder).c_str(), NULL) || GetLastError() == ERROR_ALREADY_EXISTS) {
					std::ofstream outfile;

					// Wipe any pre-existing file.
					outfile.open("../../../Data/Speed/" + motion_name + "/" + folder + "/" + name + ".csv", std::ios::out);
					outfile.close();

					outfile.open("../../../Data/Speed/" + motion_name + "/" + folder + "/" + name + ".csv", std::ios::out | std::ios::app);

					if (!outfile.fail()) {

						for (std::vector<std::pair<int, int>>::const_iterator it = data.begin(); it != data.end(); ++it) {
							outfile << (*it).first << '\n' << (*it).second << '\n';
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
				std::cout << "Failed to create the folder " + motion_name << std::endl;
				return false;
			}
		}

		bool ExportData(Frame* const frame, const std::string& motion_name, const std::string& folder, const std::string& name) {
			// 2 times, because the function can only create a directory, not the potential subdirectories.
			if (CreateDirectory(("..\\..\\..\\Data\\Speed\\" + motion_name).c_str(), NULL) || GetLastError() == ERROR_ALREADY_EXISTS) {

				if (CreateDirectory(("..\\..\\..\\Data\\Speed\\" + motion_name + "\\" + folder).c_str(), NULL) || GetLastError() == ERROR_ALREADY_EXISTS) {
					std::ofstream outfile;

					// Wipe any pre-existing file.
					outfile.open("../../../Data/Speed/" + motion_name + "/" + folder + "/" + name + ".csv", std::ios::out);
					outfile.close();

					outfile.open("../../../Data/Speed/" + motion_name + "/" + folder + "/" + name + ".csv", std::ios::out | std::ios::app);

					if (!outfile.fail()) {

						for (unsigned int i = 0; i < frame->getJoints().size(); i++) {
							outfile << frame->getJoint(i)->getName() << ","
									<< frame->getJoint(i)->getPositions().x << ","
									<< frame->getJoint(i)->getPositions().y << ","
									<< frame->getJoint(i)->getPositions().z << ","
									<< frame->getJoint(i)->getOrientations().x << ","
									<< frame->getJoint(i)->getOrientations().y << ","
									<< frame->getJoint(i)->getOrientations().z << "\n";
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
				std::cout << "Failed to create the folder " + motion_name << std::endl;
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