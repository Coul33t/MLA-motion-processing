namespace utility {

	template <typename T>
	std::pair<T, int> getMaxValue(std::vector<T>& data) {
		T max_value = data.at(0);
		int idx = -1;

		for (std::vector<T>::iterator it = std::begin(data); it < std::end(data); ++it) {
			if (*it > max_value) {
				max_value = *it;
				idx = it - data.begin();
			}
		}

		return std::pair<T, int>(max_value, idx);
	}

	template <typename T>
	std::pair<T, int> getMinValue(std::vector<T>& data) {
		T min_value = data.at(0);
		int idx = -1;

		for (std::vector<T>::iterator it = std::begin(data); it < std::end(data); ++it) {
			if (*it < min_value) {
				min_value = *it;
				idx = it - data.begin();
			}
		}

		return std::pair<T, int>(min_value, idx);
	}
}