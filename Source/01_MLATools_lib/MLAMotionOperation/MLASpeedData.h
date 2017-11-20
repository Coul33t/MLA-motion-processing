#include <iostream>
#include <map>
#include <vector>
#include <string>

class SpeedData {

	public:
		SpeedData(const double& intervalNumber, const double& timeStepInterFrame, const double& nbframe);
		~SpeedData();

		bool getJointSpeedAndTime(const std::string& jointName, const unsigned int index, std::pair<double, double>& speedTime) const;
		bool getJointSpeedAndTimeVector(const std::string& jointName, std::vector<std::pair<double,double>>& speedAndTimeVector) const;
		unsigned int getNbInterval() const;
		const double getIntervalTime() const;
		double getDuration() const;

		void addFrameSpeed(const std::map<std::string, double>& bodySpeed);

	private:
		struct MLABodySpeed {
			// Time at which the values are computed
			double m_time;
			// Set of speed values for each joint at time m_time
			std::map<std::string, double> m_speedSet;
		};

		// Set of speed and corresponding time, for each joint for the whole motion
		std::vector<MLABodySpeed> m_speedData; 
		// Total animation time
		double m_intervalTime; 


};