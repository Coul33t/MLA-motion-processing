#ifndef __MLA_CSV_EXPORT_H__
#define __MLA_CSV_EXPORT_H__

#include "MLACommonInclude.h"
#include "MLAMotion_lib.h"
#include "MLAMotionOperation/MLASpeedData.h"
#include "MLAMotionOperation/MLAMotionOperation.h"
#include "windows.h"
#include <sys/types.h>
#include <sys/stat.h>

namespace Mla {
	namespace CsvExport {

		bool ExportData(const std::map<std::string, double>&, const std::string& = "DEFAULT_MOTION_NAME", const std::string& = "", const std::string& = "default");
		bool ExportData(const std::vector<double>&, const std::string& = "DEFAULT_MOTION_NAME", const std::string& = "", const std::string& = "default");
		bool ExportData(const std::vector<std::pair<int, int>>&, const std::string& = "DEFAULT_MOTION_NAME", const std::string& = "", const std::string& = "default");
		
		// lol unused
		bool ExportData(Frame* const, const std::string& = "DEFAULT_MOTION_NAME", const std::string& = "", const std::string& = "default");
		bool ExportData(const SpeedData&, const std::string& = "DEFAULT_MOTION_NAME", const std::string& = "", const std::string& = "default");
		bool ExportData(const std::vector<SpeedData>&, const MotionInformation&, const SegmentationInformation&, const std::string& = "DEFAULT_MOTION_NAME", const std::string& = "", const std::string& = "default");
		
		bool ExportMotionSegmentationInformations(const SegmentationInformation&, const std::string& = "DEFAULT_MOTION_NAME", const std::string& = "default");
		bool ExportMotionInformations(const MotionInformation&, const std::string& = "DEFAULT_MOTION_NAME", const std::string& = "default");
		
		void EraseFolderContent(const std::string&);
	};
};

#endif //__MLA_CSV_EXPORT_H__