/*
* Description :   Interface to BQ76952 BMS IC (by Texas Instruments) for Arduino platform.
* Author      :   James Fotherby forked from pranjal-joshi/BQ76952Lib
* Date        :   23/11/2024
* License     :   MIT
* This code is published as open source software. Feel free to share/modify.
*/

#ifndef BQ76952_h
#define BQ76952_h

#include "Arduino.h"

// BQ76952 - Address Map
#define CMD_DIR_SUBCMD_LOW          	0x3E
#define CMD_DIR_SUBCMD_HI           	0x3F
#define CMD_DIR_RESP_LEN            	0x61
#define CMD_DIR_RESP_START          	0x40
#define CMD_DIR_RESP_CHKSUM         	0x60

// BQ76952 - Direct Commands with data
#define DIR_CMD_SPROTEC           		0x02    // Safety Alert A Reg
#define DIR_CMD_FPROTEC           		0x03	// Safety Fault A Reg
#define DIR_CMD_STEMP             		0x04	// Safety Alert B Reg
#define DIR_CMD_FTEMP             		0x05	// Safety Fault B Reg
#define DIR_CMD_SFET              		0x06	// Safety Alert C Reg
#define DIR_CMD_FFET              		0x07	// Safety Fault C Reg
#define DIR_CMD_BAT_STATUS          	0x12	// Battery Status	
#define DIR_CMD_VCELL_1           		0x14
//									 	 |
// 									 	 |									  
#define DIR_CMD_VCELL_16           		0x32
#define DIR_CMD_VSTACK           		0x34
#define DIR_CMD_VPACK           		0x36
#define DIR_CMD_CC2_CUR           		0x3A
#define DIR_CMD_INT_TEMP          		0x68
#define DIR_CMD_FET_STAT          		0x7F	// FET Status

// BQ76952 - Command-Only Subcommands
#define COSCMD_RESET        			0x0012	// Resets device
#define COSCMD_SET_CFGUPDATE        	0x0090	// Enters config mode
#define COSCMD_EXIT_CFGUPDATE       	0x0092	// Exits config mode
#define COSCMD_ALL_FETS_ON       		0x0096	// Enables all FETS
#define COSCMD_MANUFAC_TOGGLE_FE   		0x0022	// Toggles FET manufacture mode

// Subcommands with data
#define DASTATUS_5						0x0075	// 32 bytes (min/max cell voltages, CC1, CC3
#define DASTATUS_6						0x0076  // Accumulated charge and timer

// Data memory locations:
#define Enabled_Protections_A			0x9261
#define CUV_Threshold					0x9275
#define	COV_Threshold					0x9278
#define	OCC_Threshold					0x9280

#define OCD1_Threshold					0x9282	// [8 mV]
#define OCD1_Delay						0x9283	// [13.3 us]
#define OCD2_Threshold					0x9284	// [6 mV]
#define OCD2_Delay						0x9285	// [33.3 us]
#define SCD_Threshold					0x9286	
#define SCD_Delay						0x9287	// [30 us]

#define FET_Predischarge_Timeout		0x930E  // [50 ms]
#define FET_Predischarge_Stop_Delta		0x930F	// [500 mV]

#define FET_Options						0x9308	// [0x0D]
#define Mfg_Status_Init					0x9343	// [0x0040]
#define DA_Configuration				0x9303	// [0x05]

// Fault Bits in BQ76952 registers
#define BIT_SA_SC_DCHG            7
#define BIT_SA_OC2_DCHG           6
#define BIT_SA_OC1_DCHG           5
#define BIT_SA_OC_CHG             4
#define BIT_SA_CELL_OV            3
#define BIT_SA_CELL_UV            2

#define BIT_SB_OTF                7
#define BIT_SB_OTINT              6
#define BIT_SB_OTD                5
#define BIT_SB_OTC                4
#define BIT_SB_UTINT              2
#define BIT_SB_UTD                1
#define BIT_SB_UTC                0

enum bq76952_thermistor {
	TS1,
	TS2,
	TS3,
	HDQ,
	DCHG,
	DDSG
};

enum bq76952_fet {
	CHG,
	DCH,
	ALL
};

enum bq76952_fet_state {
	OFF,
	ON
};

enum bq76952_scd_thresh {
	SCD_10,
	SCD_20,
	SCD_40,
	SCD_60,
	SCD_80,
	SCD_100,
	SCD_125,
	SCD_150,
	SCD_175,
	SCD_200,
	SCD_250,
	SCD_300,
	SCD_350,
	SCD_400,
	SCD_450,
	SCD_500
};

typedef union protection {
	struct {
		uint8_t SC_DCHG            :1;
		uint8_t OC2_DCHG           :1;
		uint8_t OC1_DCHG           :1;
		uint8_t OC_CHG             :1;
		uint8_t CELL_OV            :1;
		uint8_t CELL_UV            :1;
	} bits;
} bq_protection_t;

typedef union temperatureProtection {
	struct {
		uint8_t OVERTEMP_FET		:1;
		uint8_t OVERTEMP_INTERNAL	:1;
		uint8_t OVERTEMP_DCHG		:1;
		uint8_t OVERTEMP_CHG		:1;
		uint8_t UNDERTEMP_INTERNAL	:1;
		uint8_t UNDERTEMP_DCHG		:1;
		uint8_t UNDERTEMP_CHG		:1;
	} bits;
} bq_temp_t;

class BQ76952 
{
	public:
		BQ76952();
		void 			begin(void);
		void 			begin(int SDA_Pin, int SCK_Pin);
		void 			setDebug(byte debug);
		void 			reset(void);
		
		unsigned int 	directCommandRead(byte command);
		void 			CommandOnlysubCommand(unsigned int command);
		byte* 			subCommandwithdata(unsigned int command, int bytes_to_read);
		
		byte*			readDataMemory(unsigned int addr);
		void			writeDataMemory(unsigned int addr, byte* data_buffer, byte noOfBytes);
		void			writeByteToMemory(unsigned int addr, byte data);
		void			writeIntToMemory(unsigned int addr, unsigned int data);
		
		// API functions:
		unsigned int 	getCellVoltage(byte cellNumber);
		int 			getCurrent(void);
		void			setConnectedCells(unsigned int Cells);
		void 			setFET(bq76952_fet fet, bq76952_fet_state state);
		bool 			isCharging(void);
		bool 			isDischarging(void);

		float 			getInternalTemp(void);
		float 			getThermistorTemp(bq76952_thermistor thermistor);
		bq_protection_t getProtectionStatus(void);
		bq_temp_t 		getTemperatureStatus(void);		
		
		void 			setDebug(bool);
		void 			debugPrint(const char*);
		void 			debugPrintln(const char*);
		void 			debugPrint(const __FlashStringHelper*);
		void 			debugPrintln(const __FlashStringHelper*);
		void 			debugPrintlnCmd(unsigned int cmd);	

		byte			_DataBuffer[36];
	private:
		void 			enterConfigUpdate(void);
		void 			exitConfigUpdate(void);
		byte			computeChecksum(byte oldChecksum, byte data);
};

#endif