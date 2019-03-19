﻿#ifndef __NMEAPRES_H__
#define __NMEAPRES_H__

#include "nmeaparser.h"

#define SIMU_DIR "../"
#define SIMU_FILE "NmeaFrames.nmeap"

#define LOG_NDEBUG 1

typedef struct {
    double fLatitude;//from gga,rmc
    double fLongitude;//from gga,rmc
    double fAltitude;//from gga
    unsigned long ulTime;//from gga
    unsigned long ulDate;//from rmc
    double fSpeed;//from rmc
    double fCourse;//from rmc
	int CeGpsFixMode;
	double fGpsSNR;
	int quality;
	struct {
		double pdop;
		double hdop;
		double vdop;
	}dilution;
}GlobalGpsStruct;

class NmeaPres : public NmeaParser
{
	public:
		NmeaPres(std::string NmeaPort, const unsigned long culBaudrate);
        NmeaPres();
		~NmeaPres();
	    bool NmeaInitParsers();
		void getGpsGlobalStruct(GlobalGpsStruct &GlobalGps);

	private:
		unsigned long ulBaudrate;
		std::string NmeaSerialPort;
		GlobalGpsStruct mGlobalGps;
		static void gga_callback(std::string NmeaType, void * ggaStruct, void * context);
		static void rmc_callback(std::string NmeaType, void * rmcStruct, void * context);
		static void gsa_callback(std::string NmeaType, void * gsaStruct, void * context);
		static void gsv_callback(std::string NmeaType, void * gsvStruct, void * context);
};

#endif
