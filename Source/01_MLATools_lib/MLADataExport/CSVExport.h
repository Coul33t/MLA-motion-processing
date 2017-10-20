#ifndef __MLA_CSV_EXPORT__
#define __MLA_CSV_EXPORT__

#include "MLACommonInclude.h"
#include "windows.h"
#include <sys/types.h>
#include <sys/stat.h>


namespace csvexport {
	 bool ExportData(std::map<std::string, double>, std::string = "DEFAULT_MOTION_NAME", std::string = "", std::string = "default");
	 bool ExportData(std::vector<double>, std::string = "DEFAULT_MOTION_NAME", std::string = "", std::string = "default");
	 void EraseFolderContent(std::string);
};

#endif //__MLA_CSV_EXPORT__