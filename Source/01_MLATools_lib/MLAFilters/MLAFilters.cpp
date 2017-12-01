#include "MLAFilters.h"

namespace Mla {
	namespace Filters {

		// Inspired from
		// https://stackoverflow.com/questions/30338671/i-tried-coding-my-own-simple-moving-average-in-c
		void MeanShift(std::vector<double>& filtered_data, const std::vector<double>& data, unsigned int window_size) {

			double running_total = 0.0;

			for (unsigned int i = 0; i < data.size(); i++) {

				running_total += data[i];   // add

				if (i >= window_size)
					running_total -= data[i - window_size];   // subtract

				if (i >= (window_size - 1))  // output moving average
					filtered_data.push_back(running_total / (double)window_size);
				else
					filtered_data.push_back(data[i]);
			}
		}

	}
}