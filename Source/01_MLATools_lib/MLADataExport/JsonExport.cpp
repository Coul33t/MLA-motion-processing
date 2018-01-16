#include "JsonExport.h"

namespace Mla {
	namespace JsonExport {

		bool ExportData(Data& data, const std::string& folder_name, const std::string& subfolder_name, const std::string& file_name) {
			// 2 times, because the function can only create a directory, not the potential subdirectories.
			if (!(CreateDirectory(("..\\..\\..\\Data\\Speed\\" + folder_name).c_str(), NULL) || GetLastError() == ERROR_ALREADY_EXISTS)) {
				std::cout << "Failed to create the subfolder_name " + folder_name << std::endl;
				return false;
			}

			if (!(CreateDirectory(("..\\..\\..\\Data\\Speed\\" + folder_name + "\\" + subfolder_name).c_str(), NULL) || GetLastError() == ERROR_ALREADY_EXISTS)) {
				std::cout << "Failed to create the subfolder_name " + subfolder_name << std::endl;
				return false;
			}

			json json_file;
			std::vector<DataType> data_contents = data.getData();

			// iterates over every data type
			for (std::vector<DataType>::const_iterator data_contents_it = data_contents.begin(); data_contents_it != data_contents.end(); ++data_contents_it) {
				
				std::map<std::string, std::vector<double>> values_rearranged;
				data.convertToMap(data_contents_it->name, values_rearranged);

				// iterates over every joint
				for (std::map<std::string, std::vector<double>>::const_iterator data_it = values_rearranged.begin(); data_it != values_rearranged.end(); ++data_it) {
					json j_vec(data_it->second);
					json_file[data_contents_it->name][data_it->first] = j_vec;
				} // end iteration over joint

			} // end iterate over data type

			std::ofstream outfile;

			// Wipe any pre-existing file.
			outfile.open("../../../Data/Speed/" + folder_name + "/" + subfolder_name + "/" + file_name + ".json", std::ios::out);
			outfile.close();

			outfile.open("../../../Data/Speed/" + folder_name + "/" + subfolder_name + "/" + file_name + ".json", std::ios::out | std::ios::app);

			if (outfile.fail()) {
				std::cout << "Failed to open file " + file_name + ".csv" << std::endl;
				return false;
			}
			outfile << json_file;
			outfile.close();
			return true;	
		}


	}
}