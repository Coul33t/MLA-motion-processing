#include "CSVExport.h"

//TODO: use pipe for communication
namespace Mla {
	namespace CsvExport {

		/** Export a map to a csv file

		@param data the data
		@param name the name of the output file (default : "default")

		@return bool success of file opening

		*/
		bool ExportData(const std::map<std::string, double>& data, const std::string& folder_name, const std::string& subfolder_name, const std::string& file_name) {
			// 2 times, because the function can only create a directory, not the potential subdirectories.
			if (CreateDirectory(("..\\..\\..\\Data\\Speed\\" + folder_name).c_str(), NULL) || GetLastError() == ERROR_ALREADY_EXISTS) {

				if (CreateDirectory(("..\\..\\..\\Data\\Speed\\" + folder_name + "\\" + subfolder_name).c_str(), NULL) || GetLastError() == ERROR_ALREADY_EXISTS) {
					std::ofstream outfile;

					// Wipe any pre-existing file.
					outfile.open("../../../Data/Speed/" + folder_name + "/" + subfolder_name + "/" + file_name + ".csv", std::ios::out);
					outfile.close();

					outfile.open("../../../Data/Speed/" + folder_name + "/" + subfolder_name + "/" + file_name + ".csv", std::ios::out | std::ios::app);

					if (!outfile.fail()) {

						for (std::map<std::string, double>::const_iterator it = data.begin(); it != data.end(); ++it) {
							outfile << it->first << "," << it->second << '\n';
						}

						outfile.close();
						return true;
					}

					else {
						std::cout << "Failed to open file " + file_name + ".csv" << std::endl;
						return false;
					}
				}

				else {
					std::cout << "Failed to create the subfolder_name " + subfolder_name << std::endl;
					return false;
				}
			}

			else {
				std::cout << "Failed to create the subfolder_name " + folder_name << std::endl;
				return false;
			}


		}

		bool ExportData(const std::vector<double>& data, const std::string& folder_name, const std::string& subfolder_name, const std::string& file_name) {
			// 2 times, because the function can only create a directory, not the potential subdirectories.
			if (CreateDirectory(("..\\..\\..\\Data\\Speed\\" + folder_name).c_str(), NULL) || GetLastError() == ERROR_ALREADY_EXISTS) {

				if (CreateDirectory(("..\\..\\..\\Data\\Speed\\" + folder_name + "\\" + subfolder_name).c_str(), NULL) || GetLastError() == ERROR_ALREADY_EXISTS) {
					std::ofstream outfile;

					// Wipe any pre-existing file.
					outfile.open("../../../Data/Speed/" + folder_name + "/" + subfolder_name + "/" + file_name + ".csv", std::ios::out);
					outfile.close();

					outfile.open("../../../Data/Speed/" + folder_name + "/" + subfolder_name + "/" + file_name + ".csv", std::ios::out | std::ios::app);

					if (!outfile.fail()) {

						for (std::vector<double>::const_iterator it = data.begin(); it != data.end(); ++it) {
							outfile << *it << '\n';
						}

						outfile.close();
						return true;
					}

					else {
						std::cout << "Failed to open file " + file_name + ".csv" << std::endl;
						return false;
					}
				}

				else {
					std::cout << "Failed to create the subfolder_name " + subfolder_name << std::endl;
					return false;
				}
			}

			else {
				std::cout << "Failed to create the subfolder_name " + folder_name << std::endl;
				return false;
			}


		}

		bool ExportData(const std::vector<std::pair<int, int>>& data, const std::string& folder_name, const std::string& subfolder_name, const std::string& file_name) {
			// 2 times, because the function can only create a directory, not the potential subdirectories.
			if (CreateDirectory(("..\\..\\..\\Data\\Speed\\" + folder_name).c_str(), NULL) || GetLastError() == ERROR_ALREADY_EXISTS) {

				if (CreateDirectory(("..\\..\\..\\Data\\Speed\\" + folder_name + "\\" + subfolder_name).c_str(), NULL) || GetLastError() == ERROR_ALREADY_EXISTS) {
					std::ofstream outfile;

					// Wipe any pre-existing file.
					outfile.open("../../../Data/Speed/" + folder_name + "/" + subfolder_name + "/" + file_name + ".csv", std::ios::out);
					outfile.close();

					outfile.open("../../../Data/Speed/" + folder_name + "/" + subfolder_name + "/" + file_name + ".csv", std::ios::out | std::ios::app);

					if (!outfile.fail()) {

						for (std::vector<std::pair<int, int>>::const_iterator it = data.begin(); it != data.end(); ++it) {
							outfile << (*it).first << '\n' << (*it).second << '\n';
						}

						outfile.close();
						return true;
					}

					else {
						std::cout << "Failed to open file " + file_name + ".csv" << std::endl;
						return false;
					}
				}

				else {
					std::cout << "Failed to create the subfolder_name " + subfolder_name << std::endl;
					return false;
				}
			}

			else {
				std::cout << "Failed to create the folder name " + folder_name << std::endl;
				return false;
			}
		}

		bool ExportData(Frame* const frame, const std::string& folder_name, const std::string& subfolder_name, const std::string& file_name) {
			// 2 times, because the function can only create a directory, not the potential subdirectories.
			if (CreateDirectory(("..\\..\\..\\Data\\Speed\\" + folder_name).c_str(), NULL) || GetLastError() == ERROR_ALREADY_EXISTS) {

				if (CreateDirectory(("..\\..\\..\\Data\\Speed\\" + folder_name + "\\" + subfolder_name).c_str(), NULL) || GetLastError() == ERROR_ALREADY_EXISTS) {
					std::ofstream outfile;

					// Wipe any pre-existing file.
					outfile.open("../../../Data/Speed/" + folder_name + "/" + subfolder_name + "/" + file_name + ".csv", std::ios::out);
					outfile.close();

					outfile.open("../../../Data/Speed/" + folder_name + "/" + subfolder_name + "/" + file_name + ".csv", std::ios::out | std::ios::app);

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
						std::cout << "Failed to open file " + file_name + ".csv" << std::endl;
						return false;
					}
				}

				else {
					std::cout << "Failed to create the subfolder_name " + subfolder_name << std::endl;
					return false;
				}
			}

			else {
				std::cout << "Failed to create the subfolder_name " + folder_name << std::endl;
				return false;
			}
		}

		bool ExportData(const SpeedData& data, const std::string& folder_name, const std::string& subfolder_name, const std::string& file_name) {
			
			std::vector<std::map<std::string, double>> speed_set;
			
			data.getSpeedSetVector(speed_set);
			
			for (unsigned int i = 0; i < speed_set.size(); i++) {
				ExportData(speed_set[i], folder_name, subfolder_name, file_name + std::to_string(i));
			}

			return true;
		}

		bool ExportData(const std::vector<SpeedData>& data, const MotionInformation& motion_info, const SegmentationInformation& seg_info, const std::string& folder_name, const std::string& subfolder_name, const std::string& file_name) {
			
			ExportMotionInformations(motion_info, folder_name, "motion_information");
			ExportMotionSegmentationInformations(seg_info, folder_name, "segmentation_information");

			for (unsigned int i = 0; i < data.size(); i++) {
				ExportData(data[i], folder_name, subfolder_name + std::to_string(i), file_name);
			}

			return true;
		}

		bool ExportMotionSegmentationInformations(const SegmentationInformation& seg_motion_info, const std::string& folder_name, const std::string& file_name) {
			if (CreateDirectory(("..\\..\\..\\Data\\Speed\\" + folder_name).c_str(), NULL) || GetLastError() == ERROR_ALREADY_EXISTS) {

				std::ofstream outfile;

				// Wipe any pre-existing file.
				outfile.open("../../../Data/Speed/" + folder_name + "/" + file_name + ".json", std::ios::out);
				outfile.close();

				outfile.open("../../../Data/Speed/" + folder_name + "/" + file_name + ".json", std::ios::out | std::ios::app);

				if (!outfile.fail()) {
					outfile << "{\n";
					outfile << "\t\"desired left cut number\": " << seg_motion_info.left_cut << ",\n";
					outfile << "\t\"desired right cut number\": " << seg_motion_info.right_cut << ",\n";
					outfile << "\t\"final frame number\": " << seg_motion_info.final_frame_number << ",\n";
					outfile << "\t\"savgol\": {\n";
					outfile << "\t\t\"window size\": " << seg_motion_info.savgol_window_size << ",\n";
					outfile << "\t\t\"polynom order\": " << seg_motion_info.savgol_polynom_order << "\n";
					outfile << "\t}\n";
					outfile << "}";
					outfile.close();
					return true;
				}

				else {
					std::cout << "Failed to open file " + file_name + ".json" << std::endl;
					return false;
				}
			}

			else {
				std::cout << "Failed to create the folder " + folder_name << std::endl;
				return false;
			}
		}

		bool ExportMotionInformations(const MotionInformation& motion_info, const std::string& folder_name, const std::string& file_name) {
			if (CreateDirectory(("..\\..\\..\\Data\\Speed\\" + folder_name).c_str(), NULL) || GetLastError() == ERROR_ALREADY_EXISTS) {
				std::ofstream outfile;

				// Wipe any pre-existing file.
				outfile.open("../../../Data/Speed/" + folder_name + "/" + file_name + ".json", std::ios::out);
				outfile.close();

				outfile.open("../../../Data/Speed/" + folder_name + "/" + file_name + ".json", std::ios::out | std::ios::app);

				if (!outfile.fail()) {
					outfile << "{\n";
					outfile << "\t\"motion name\": \"" << motion_info.motion_name << "\",\n";
					outfile << "\t\"frame number\": " << motion_info.frame_number << ",\n";
					outfile << "\t\"interframe time\": " << motion_info.frame_time << ",\n";
					outfile << "\t\"root name\": \"" << motion_info.root_name << "\"\n";
					outfile << "}";
					outfile.close();
					return true;
				}

				else {
					std::cout << "Failed to open file " + file_name + ".json" << std::endl;
					return false;
				}
			}

			else {
				std::cout << "Failed to create the folder " + folder_name << std::endl;
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