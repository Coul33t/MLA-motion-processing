#ifndef __MLA_JSON_EXPORT_H__
#define __MLA_JSON_EXPORT_H__

#include "MLACommonInclude.h"
#include "MLAMotion_lib.h"

#include "MLAData/MLAData.h"

#include "json.hpp"

#include "windows.h"
#include <sys/types.h>
#include <sys/stat.h>

// for convenience
using json = nlohmann::json;

namespace Mla {
	namespace JsonExport {
		bool ExportData(Data& data, const std::string& = "default_folder_name", const std::string& = "default_subfolder_name", const std::string& = "default_file_name");
	}
}
#endif //__MLA_JSON_EXPORT_H__