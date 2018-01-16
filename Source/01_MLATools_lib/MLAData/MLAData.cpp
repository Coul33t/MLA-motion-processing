#include "MLAData.h"

Data::Data() {

}

Data::Data(const std::string& name) {
	DataType dataT;
	dataT.name = name;
	dataT.values = std::vector<std::map<std::string, double>>();
	m_data.push_back(dataT);
}

Data::Data(const std::string& name, std::vector<std::map<std::string, double>>& values) {
	insertNewData(name, values);
}

void Data::insertNewData(const std::string& name, std::vector<std::map<std::string, double>>& values) {
	DataType dataT;
	dataT.name = name;
	dataT.values = values;
	m_data.push_back(dataT);
}

std::vector<DataType>& Data::getData() {
	return m_data;
}

bool Data::convertToMap(const std::string& name, std::map<std::string, std::vector<double>>& returned_values) {
	
	returned_values.clear();

	auto it_data = find_if(m_data.begin(), m_data.end(), [&name](const DataType& obj) {return obj.name == name; });

	// If the name exists (if the data is in the vector)
	// it_data = DataType
	if (it_data != m_data.end()) {

		// for each frame (= map in the vector)
		// it_found = iterator on vector<map>
		for (std::vector<std::map<std::string, double>>::const_iterator it_found = it_data->values.begin(); it_found != it_data->values.end(); ++it_found) {
			
			// if the map is empty (i.e. first iteration), we fill it with the joints names
			if (returned_values.empty()) {
				// it_names = iterator over a map (inside the vector of map)
				for (std::map<std::string, double>::const_iterator it_names = it_found->begin(); it_names != it_found->end(); ++it_names) {
					returned_values[it_names->first] = std::vector<double>();
				}
			}

			// Whenever it was empty or not, it is not now, so we insert the values
			for (std::map<std::string, double>::const_iterator it_map = it_found->begin(); it_map != it_found->end(); ++it_map) {
				returned_values[it_map->first].push_back(it_map->second);
			}
			
		}

	}

	else
		return false;

	return true;
}