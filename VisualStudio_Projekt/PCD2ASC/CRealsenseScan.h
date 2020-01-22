#ifndef _CREALSENSESCAN_H_
#define _CREALSENSESCAN_H_

#include <string>

class CRealsenseScan
{

public:
	CRealsenseScan();

	~CRealsenseScan();


	/**
	* @brief sucht nach einer aktiven Realsense-Kamera; prüft und korrigiert die Aufnahmeparameter; macht eine Aufnahme und speichert sie unter dem übergebenen Dateinamen(sFileName) ab.
	* @param sFileName           [in] Dateiname zum speichern der Aufnahme (PLY-Datei)
    * @return true wenn das Scan erfolgreich gespeichert wurde, sonst false
    **/
	bool performScanAndSave(std::string& sFileName);

private:

};


#endif _CREALSENSESCAN_H_