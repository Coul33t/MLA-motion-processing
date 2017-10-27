#include "MLAFilters.h"

namespace mla {
	namespace filters {

		// Inspired from
		// https://stackoverflow.com/questions/30338671/i-tried-coding-my-own-simple-moving-average-in-c
		void MeanShift(std::vector<double>& filteredData, const std::vector<double>& data, unsigned int windowSize) {

			double runningTotal = 0.0;

			for (unsigned int i = 0; i < data.size(); i++) {

				runningTotal += data.at(i);   // add

				if (i >= windowSize)
					runningTotal -= data.at(i - windowSize);   // subtract

				if (i >= (windowSize - 1))  // output moving average
					filteredData.push_back(runningTotal / (double)windowSize);
				else
					filteredData.push_back(data.at(i));
			}
		}

	}
}