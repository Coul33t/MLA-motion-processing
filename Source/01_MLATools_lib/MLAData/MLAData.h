/**
Class for the data.
MLAData.h

@author Quentin Couland
@version 0.1

*/

#ifndef __MLA_DATA_H__
#define __MLA_DATA_H__

#include "MLACommonInclude.h"

struct DataType {
	// Name of the data (e.g. speed, acceleration, etc.)
	std::string name;
	// Values (one map for each frame)
	std::vector<std::map<std::string, double>> values;
};

class Data {
	public:
		Data();
		Data(const std::string&);
		Data(const std::string&, std::vector<std::map<std::string, double>>&);
		void insertNewData(const std::string&, std::vector<std::map<std::string, double>>&);

		std::vector<DataType>& getData();

		bool convertToMap(const std::string&, std::map<std::string, std::vector<double>>&);

	private:
		std::vector<DataType> m_data;
		
};

#endif