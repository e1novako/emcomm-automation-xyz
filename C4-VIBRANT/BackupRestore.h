#pragma once
#include <LittleFS.h>
namespace vibrant {
extern File importFile;
extern bool importFailed;
void handleConfigExport();
void handleConfigImportUpload();
void handleConfigImportDone();
void handleFactoryReset();
} // namespace vibrant
