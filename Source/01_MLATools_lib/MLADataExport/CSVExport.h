#ifndef __MLA_CSV_EXPORT__
#define __MLA_CSV_EXPORT__

#include "MLACommonInclude.h"
class CSVExport {
public:
	CSVExport();
	~CSVExport();

	static bool ExportData(std::map<std::string, double>, std::string = "default");
};

#endif //__MLA_CSV_EXPORT__