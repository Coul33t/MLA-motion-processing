#ifndef __MLA_CSV_EXPORT__
#define __MLA_CSV_EXPORT__

#include "MLACommonInclude.h"
#include "windows.h"

class CSVExport {
public:
	CSVExport();
	~CSVExport();

	static bool ExportData(std::map<std::string, double>, std::string = "DEFAULT_MOTION_NAME", std::string = "", std::string = "default");
};

#endif //__MLA_CSV_EXPORT__