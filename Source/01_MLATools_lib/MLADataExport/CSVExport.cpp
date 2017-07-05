#include "CSVExport.h"

bool CSVExport::ExportData(std::map<std::string, double> data, std::string name) {

	std::ofstream outfile;
	outfile.open("../../../Data/Speed/" + name + ".csv", std::ios::out | std::ios::app);

	if (!outfile.fail()) {

		for (std::map<std::string, double>::iterator it = data.begin(); it != data.end(); ++it) {
			outfile << it->first << "," << it->second << '\n';
		}

		outfile.close();
		return true;
	}

	else {
		std::cout << "Failed to open file tmp.bvh" << std::endl;
		return false;
	}

	
}