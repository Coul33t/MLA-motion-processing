#ifndef __MLA_JSON_EXPORT_H__
#define __MLA_JSON_EXPORT_H__

#include "MLACommonInclude.h"
#include "MLAMotion_lib.h"

#include "MLAData/MLAData.h"
#include "MLAMotionOperation/MLAMotionOperation.h"

#include "json.hpp"

#include "windows.h"
#include <sys/types.h>
#include <sys/stat.h>

// for convenience
using json = nlohmann::json;

namespace Mla {
	namespace JsonExport {
		bool ExportData(Data& data, const std::string& = "default_folder_name", const std::string& = "default_subfolder_name", const std::string& = "default_file_name");
		bool ExportMotionSegmentationInformations(const SegmentationInformation& seg_motion_info, const std::string& folder_name, const std::string& file_name);
		bool ExportMotionInformations(const MotionInformation& motion_info, std::vector<std::string>, const std::string& folder_name, const std::string& file_name);
	}
}
#endif //__MLA_JSON_EXPORT_H__