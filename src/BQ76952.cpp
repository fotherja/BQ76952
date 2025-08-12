/*
* Description :   Interface to BQ76952 BMS IC (by Texas Instruments) for Arduino platform.
* Author      :   James Fotherby forked from pranjal-joshi/BQ76952Lib
* Date        :   23/11/2024
* License     :   MIT
* This code is published as open source software. Feel free to share/modify.
*/

#include "BQ76952.h"
#include <Wire.h>

// Defines:
#define	DBG_BAUD					115200
#define BQ_I2C_ADDR   				0x08

#define CELL_NO_TO_ADDR(cellNo) (DIR_CMD_VCELL_1 + ((cellNo-1)*2))
#define LOW_BYTE(addr) (byte)(addr & 0x00FF)
#define HIGH_BYTE(addr) (byte)((addr >> 8) & 0x00FF)

bool BQ_DEBUG = false;

BQ76952::BQ76952()
{
}

void BQ76952::begin(void)
{
	Wire.begin();
	if(BQ_DEBUG) {
	Serial.begin(DBG_BAUD);
	debugPrintln(F("[+] Initializing BQ76952..."));
	}
}

void BQ76952::begin(int SDA_Pin, int SCK_Pin)
{
	Wire.begin(SDA_Pin,SCK_Pin,400000);
	if(BQ_DEBUG) {
	Serial.begin(DBG_BAUD);
	debugPrintln(F("[+] Initializing BQ76952..."));
	}
}

void BQ76952::setDebug(byte debug)
{
	BQ_DEBUG = debug;
}

void BQ76952::reset(void)
{
	CommandOnlysubCommand(COSCMD_RESET);
}

void BQ76952::enterConfigUpdate(void) {
  CommandOnlysubCommand(COSCMD_SET_CFGUPDATE);
  delayMicroseconds(2000);
}

void BQ76952::exitConfigUpdate(void) {
  CommandOnlysubCommand(COSCMD_EXIT_CFGUPDATE);
  delayMicroseconds(2000);
}

// Send Direct command
// Direct commands have a single byte address followed by reading or writing 1 or 2 subsequent bytes
// However there are very few direct commands that involve writing data so we have only implemented reading
unsigned int BQ76952::directCommandRead(byte command) {
	Wire.beginTransmission(BQ_I2C_ADDR);				// Begin with the device I2C address
	Wire.write(command);								// Write the command
	Wire.endTransmission();
	delayMicroseconds(1000);

	Wire.requestFrom(BQ_I2C_ADDR, 2);					// Read 2 bytes (the particular command may only return 1 byte though				
	delayMicroseconds(1000);

    unsigned long startMillis = millis();
    while (!Wire.available()) {  						// Wait for all bytes
        if (millis() - startMillis > 10) {  			// Timeout after 10ms
            debugPrint(F("[!] Timeout waiting for I2C response.\n"));
            return(0);
        }
    }
	byte lsb = Wire.read();
	byte msb = Wire.read();

	debugPrint(F("[+] Direct Cmd SENT -> "));
	debugPrintlnCmd((uint16_t)command);
	debugPrint(F("[+] Direct Cmd RESP <- "));
	debugPrintlnCmd((uint16_t)(msb << 8 | lsb));

	return (unsigned int)(msb << 8 | lsb);
}


// Command-Only subcommand
// These subcommands on their own force simple actions eg. device reset, enter config mode, turn fets off etc
void BQ76952::CommandOnlysubCommand(unsigned int command) {
	Wire.beginTransmission(BQ_I2C_ADDR);				// Begin with the device I2C address
	Wire.write(CMD_DIR_SUBCMD_LOW);						// This sets the device's address pointer (ie 3E)
	Wire.write((byte)command & 0x00FF);					// This writes data into the address we just pointed to
	Wire.write((byte)(command >> 8) & 0x00FF);			// This writes the next data byte (pointer address autoincrements)
	Wire.endTransmission();

	delayMicroseconds(1000);

	debugPrint(F("[+] Sub Cmd SENT to 0x3E -> "));
	debugPrintlnCmd((uint16_t)command);
}


// Subcommand with data
// These subcommands return data (some return nearly 32 bytes)
byte* BQ76952::subCommandwithdata(unsigned int command, int bytes_to_read) {
    if (bytes_to_read > sizeof(_DataBuffer)) {
        debugPrint(F("[!] Error: bytes_to_read exceeds buffer size.\n"));
        return nullptr;  // Prevent overflow
    }

    CommandOnlysubCommand(command);  					// Enact the subcommand

    Wire.beginTransmission(BQ_I2C_ADDR);
    Wire.write(CMD_DIR_RESP_START);  					// Set address to start of data buffer
    Wire.endTransmission();
    delayMicroseconds(1000);  							// Allow time for the device to respond

    Wire.requestFrom(BQ_I2C_ADDR, bytes_to_read);
	delayMicroseconds(1000);
    
    unsigned long startMillis = millis();
    while (Wire.available() < bytes_to_read) {  		// Wait for all bytes
        if (millis() - startMillis > 10) {  			// Timeout after 10ms
            debugPrint(F("[!] Timeout waiting for I2C response.\n"));
            return nullptr;
        }
    }

    debugPrint(F("[+] Sub Cmd RESP at 0x40 -> "));
	for (int i = 0; i < bytes_to_read; i++) {
		_DataBuffer[i] = Wire.read();

		// Format the current byte as a hexadecimal string (e.g., "0x1A")
		char hexString[5];  // 4 characters for "0xXX" + null terminator
		snprintf(hexString, sizeof(hexString), "0x%02X", _DataBuffer[i]);
		debugPrint(hexString);  // Print the formatted hex string

		// Add a comma and space after each byte, except the last one
		if (i < bytes_to_read - 1) {
			debugPrint(F(", "));
		}
		else	{
			debugPrintln(F(" "));
		}
			
	}

    return _DataBuffer;  // Return pointer to buffer
}

// Write a subcommand with payload (no response data).
// Returns true on success.
// subcmd: 16-bit subcommand code
// data: pointer to payload (may be nullptr if len==0)
// len:  number of payload bytes (0..32 typically)
bool BQ76952::subCommandWriteData(uint16_t subcmd, const uint8_t* data, uint8_t len) {
  // Build [LSB, MSB, payload...]
  const uint8_t maxBlock = sizeof(_DataBuffer);            
  if ((uint16_t)len + 2 > maxBlock) {
    debugPrintln(F("[!] subCommandWriteData: payload too large"));
    return false;
  }

  _DataBuffer[0] = (uint8_t)(subcmd & 0xFF);
  _DataBuffer[1] = (uint8_t)((subcmd >> 8) & 0xFF);
  for (uint8_t i = 0; i < len; i++) _DataBuffer[2 + i] = data ? data[i] : 0;

  // Compute checksum = ~sum(block) & 0xFF ; length = blockLen + 2
  uint16_t sum = 0;
  for (uint8_t i = 0; i < (uint8_t)(len + 2); i++) sum += _DataBuffer[i];
  uint8_t checksum = (uint8_t)(~sum);                // same as (0xFF - (sum & 0xFF))
  uint8_t writelen = (uint8_t)(len + 2 + 2);         // "+2" per TI (include subcmd bytes)

  // Write block to 0x3E
  Wire.beginTransmission(BQ_I2C_ADDR);
  Wire.write(CMD_DIR_SUBCMD_LOW);                    // 0x3E
  for (uint8_t i = 0; i < (uint8_t)(len + 2); i++) Wire.write(_DataBuffer[i]);
  if (Wire.endTransmission() != 0) {
    debugPrintln(F("[!] I2C error writing subcmd block"));
    return false;
  }

  delayMicroseconds(1000); // brief settle; most cmds ~0.5–0.6 ms

  // Write checksum & length to 0x60
  Wire.beginTransmission(BQ_I2C_ADDR);
  Wire.write(0x60);
  Wire.write(checksum);
  Wire.write(writelen);
  if (Wire.endTransmission() != 0) {
    debugPrintln(F("[!] I2C error writing checksum/length"));
    return false;
  }

  // Optional: poll 0x3E/0x3F until device echoes the subcommand (completion)
  unsigned long t0 = millis();
  for (;;) {
    Wire.beginTransmission(BQ_I2C_ADDR);
    Wire.write(CMD_DIR_SUBCMD_LOW);                  // 0x3E
    if (Wire.endTransmission(false) != 0) break;     // repeated start
    if (Wire.requestFrom(BQ_I2C_ADDR, 2) == 2) {
      uint8_t lo = Wire.read();
      uint8_t hi = Wire.read();
      if (lo == (uint8_t)(subcmd & 0xFF) && hi == (uint8_t)(subcmd >> 8)) break;
    }
    if (millis() - t0 > 10) {                        // ~10 ms guard
      debugPrintln(F("[!] subcmd completion timeout"));
      return false;
    }
  }

  debugPrint(F("[+] Sub Cmd WRITE 0x"));
  debugPrintlnCmd(subcmd);
  debugPrint(F(" len="));
  debugPrintln(len);
  return true;
}

// Read Bytes from Data memory of BQ76952
// Provide an address. 32
byte* BQ76952::readDataMemory(unsigned int addr) {
	byte chksum = 0;
	chksum = computeChecksum(chksum, LOW_BYTE(addr));
	chksum = computeChecksum(chksum, HIGH_BYTE(addr));

	CommandOnlysubCommand(addr);  						// This causes a read from a register into the transfer buffer	
	delayMicroseconds(2000);
	
    Wire.beginTransmission(BQ_I2C_ADDR);
    Wire.write(CMD_DIR_RESP_START);  					// Set address to start of data buffer
    Wire.endTransmission();
    delayMicroseconds(1000);  							// Allow time for the device to respond

    Wire.requestFrom(BQ_I2C_ADDR, 32);					
	delayMicroseconds(1000);
    
    unsigned long startMillis = millis();
    while (Wire.available() < 32) {  					// Wait for all bytes
        if (millis() - startMillis > 10) {  			// Timeout after 10ms
            debugPrint(F("[!] Timeout waiting for I2C response.\n"));
            return nullptr;
        }
    }

    debugPrint(F("[+] Data RESCEIVED -> "));
	for (int i = 0; i < 32; i++) {
		_DataBuffer[i] = Wire.read();
		
		chksum = computeChecksum(chksum, _DataBuffer[i]);

		// Format the current byte as a hexadecimal string (e.g., "0x1A")
		char hexString[5];  // 4 characters for "0xXX" + null terminator
		snprintf(hexString, sizeof(hexString), "0x%02X", _DataBuffer[i]);
		debugPrint(hexString);  // Print the formatted hex string

		// Add a comma and space after each byte, except the last one
		if (i < 31) {
			debugPrint(F(", "));
		}
		else	{
			debugPrintln(F(" "));
		}
			
	}

	// We've just received the 32 byte transfer buffer. Now receive the checksum and check it
    Wire.beginTransmission(BQ_I2C_ADDR);
    Wire.write(CMD_DIR_RESP_CHKSUM);  					// Set address to start of data buffer
    Wire.endTransmission();
    delayMicroseconds(1000);  							// Allow time for the device to respond

    Wire.requestFrom(BQ_I2C_ADDR, 1);					
	delayMicroseconds(1000);
    
    startMillis = millis();
    while (Wire.available() < 1) {  					// Wait for all bytes
        if (millis() - startMillis > 10) {  			// Timeout after 10ms
            debugPrint(F("[!] Timeout waiting for I2C response.\n"));
            return nullptr;
        }
    }
	
	byte Received_chksum = Wire.read();;
	
	debugPrint(F("[+] Calculated Checksum: "));
	debugPrintlnCmd(chksum);
	
	debugPrint(F("[+] Received Checksum: "));
	debugPrintlnCmd(Received_chksum);	

    return _DataBuffer;  // Return pointer to buffer
}

void BQ76952::writeDataMemory(unsigned int addr, byte* data_buffer, byte noOfBytes) {
	byte chksum = 0;
	chksum = computeChecksum(chksum, LOW_BYTE(addr));
	chksum = computeChecksum(chksum, HIGH_BYTE(addr));

	enterConfigUpdate();
	delayMicroseconds(1000);  
	
	debugPrint(F("[+] address SENT to 0x3E -> "));
	debugPrintlnCmd((uint16_t)addr);
  
	Wire.beginTransmission(BQ_I2C_ADDR);				// Begin with the device I2C address
	Wire.write(CMD_DIR_SUBCMD_LOW);						// This sets the device's address pointer (ie 3E)
	Wire.write(LOW_BYTE(addr));							// This writes data into the address we just pointed to
	Wire.write(HIGH_BYTE(addr));						// This writes the next data byte (pointer address autoincrements)
  
	debugPrint(F("[+] Data SENT -> "));
	for (int i = 0; i < noOfBytes; i++) {
		Wire.write(data_buffer[i]);						// Write from our passed buffer calculating checksum as we go
		
		chksum = computeChecksum(chksum, data_buffer[i]);

		// Format the current byte as a hexadecimal string (e.g., "0x1A")
		char hexString[5];  // 4 characters for "0xXX" + null terminator
		snprintf(hexString, sizeof(hexString), "0x%02X", data_buffer[i]);
		debugPrint(hexString);  // Print the formatted hex string

		// Add a comma and space after each byte, except the last one
		if (i < noOfBytes - 1) {
			debugPrint(F(", "));
		}
		else	{
			debugPrintln(F(" "));
		}			
	}	
	
	for (int i = noOfBytes; i < 32; i++) {
		Wire.write(0);									// Pad with zeros
	}  
  
	Wire.write(chksum);									// write checksum
	Wire.write(noOfBytes + 4);							// write bytes
	Wire.endTransmission();

	exitConfigUpdate();
}

// Set user-defined number of cells connected
void BQ76952::writeByteToMemory(unsigned int addr, byte data) {
	writeDataMemory(addr, &data, 1);
}

// Set user-defined number of cells connected
void BQ76952::writeIntToMemory(unsigned int addr, unsigned int data) {
	writeDataMemory(addr, reinterpret_cast<byte*>(&data), 2);
}

void BQ76952::writeFloatToMemory(unsigned int addr, float data)	{
	writeDataMemory(addr, reinterpret_cast<byte*>(&data), 4);	
}


// Compute checksome = ~(sum of all bytes)
byte BQ76952::computeChecksum(byte oldChecksum, byte data) {
  if(!oldChecksum)
    oldChecksum = data;
  else
    oldChecksum = ~(oldChecksum) + data;
  return ~(oldChecksum);
}

// -------------------------------------------------------------------
// --------------------------- API Functions -------------------------
// -------------------------------------------------------------------

// Set user-defined number of cells connected
void BQ76952::setConnectedCells(unsigned int Cells) {
  if(Cells < 3 || Cells > 16)
    Cells = 16;
  else {
	uint16_t Cell_Flags = (1 << Cells) - 1;  
	  
    debugPrint(F("[+] Vcell Mode => "));
    debugPrintlnCmd(Cell_Flags);
    writeDataMemory(0x9304, reinterpret_cast<byte*>(&Cell_Flags), 2);
  }
}

// Read single cell voltage
unsigned int BQ76952::getCellVoltage(byte cellNumber) {
  return directCommandRead(CELL_NO_TO_ADDR(cellNumber));
}

// Measure CC2 current
int BQ76952::getCurrent(void) {
  return (int16_t)directCommandRead(DIR_CMD_CC2_CUR);
}

// Returns the accumulated charge and updates the time since this was reset
float BQ76952::getAccumulatedCharge(void)	{
  byte* buffer = subCommandwithdata(DASTATUS_6, 12);
  
  // Extract the integer portion (I4 - 32-bit signed integer)
  int32_t accumChargeInteger = 
      ((int32_t)buffer[3] << 24) |
      ((int32_t)buffer[2] << 16) |
      ((int32_t)buffer[1] << 8) |
      buffer[0];

  // Extract the fractional portion (U4 - 32-bit unsigned integer)
  uint32_t accumChargeFraction = 
      ((uint32_t)buffer[7] << 24) |
      ((uint32_t)buffer[6] << 16) |
      ((uint32_t)buffer[5] << 8) |
      buffer[4];

  // Extract the integer time in Seconds
  this->AccumulatedChargeTime = 
      ((uint32_t)buffer[11] << 24) |
      ((uint32_t)buffer[10] << 16) |
      ((uint32_t)buffer[9] << 8) |
      buffer[8];
	  
  // Convert the fractional portion to userAh by dividing by 2^32
  float fractionalCharge = (float)accumChargeFraction / (float)(1ULL << 32);

  // Combine the integer and fractional portions
  float totalAccumulatedCharge = accumChargeInteger + fractionalCharge;

  return totalAccumulatedCharge;  
}

// Return the time in seconds since the accumulated charge was reset
uint32_t BQ76952::getAccumulatedChargeTime(void)	{
  getAccumulatedCharge();
  return this->AccumulatedChargeTime;
}

// Resets accumulated charge and timer
void BQ76952::ResetAccumulatedCharge(void)	{
  CommandOnlysubCommand(COSCMD_RESET_PASSQ);
}

// Returns a bit mask of cells being balanced
uint16_t BQ76952::GetCellBalancingBitmask(void) {
    byte* buffer = subCommandwithdata(CB_ACTIVE_CELLS, 2);
    return ((uint16_t)buffer[0]) | ((uint16_t)buffer[1] << 8);
}

// Read cumulative time spent balancing for each cell into a passed buffer
void BQ76952::GetCellBalancingTimes(uint32_t* Cell_Balance_Times)	{
  byte* buffer = subCommandwithdata(CBSTATUS2, 32);
  
  for(byte i = 0;i < 8;i++)  {
    uint8_t offset = 4 * i;

    Cell_Balance_Times[i] =
      ((uint32_t)buffer[3 + offset] << 24) |
      ((uint32_t)buffer[2 + offset] << 16) |
      ((uint32_t)buffer[1 + offset] << 8) |
      buffer[0 + offset];
  }

  buffer = subCommandwithdata(CBSTATUS3, 32);
  for(byte i = 0;i < 8;i++)  {
    uint8_t offset = 4 * i;

    Cell_Balance_Times[i+8] =
      ((uint32_t)buffer[3 + offset] << 24) |
      ((uint32_t)buffer[2 + offset] << 16) |
      ((uint32_t)buffer[1 + offset] << 8) |
      buffer[0 + offset];
  }	
}

void BQ76952::setFET(bq76952_fet fet, bq76952_fet_state state) {
  unsigned int subcmd;
  switch(state) {
    case OFF:
      switch(fet) {
        case DCHG:
          subcmd = COSCMD_DSG_PDSG_OFF;
          break;
        case CHG:
          subcmd = COSCMD_CHG_PCHG_OFF;
          break;
        default:
          subcmd = COSCMD_ALL_FETS_OFF;
          break;
      }
      break;
    case ON:
      subcmd = COSCMD_ALL_FETS_ON;
      break;
  }
  CommandOnlysubCommand(subcmd);
}

// is Charging FET ON?
bool BQ76952::isCharging(void) {
  byte regData = (byte)directCommandRead(DIR_CMD_FET_STAT);
  if(regData & 0x01) {
    debugPrintln(F("[+] Charging FET -> ON"));
    return true;
  }
  debugPrintln(F("[+] Charging FET -> OFF"));
  return false;
}

// is Discharging FET ON?
bool BQ76952::isDischarging(void) {
  byte regData = (byte)directCommandRead(DIR_CMD_FET_STAT);
  if(regData & 0x04) {
    debugPrintln(F("[+] Discharging FET -> ON"));
    return true;
  }
  debugPrintln(F("[+] Discharging FET -> OFF"));
  return false;
}

// are cells being balanced?
uint16_t BQ76952::isBalancing(void)	{
	return GetCellBalancingBitmask();
}

// Measure chip temperature in °C
float BQ76952::getInternalTemp(void) {
  float raw = directCommandRead(DIR_CMD_INT_TEMP)/10.0;
  return (raw - 273.15);
}

// Measure thermistor temperature in °C
float BQ76952::getThermistorTemp(bq76952_thermistor thermistor) {
  byte cmd;
  switch(thermistor) {
    case TS1:
      cmd = 0x70;
      break;
    case TS2:
      cmd = 0x72;
      break;
    case TS3:
      cmd = 0x74;
      break;
    case HDQ:
      cmd = 0x76;
      break;
    case DCHG:
      cmd = 0x78;
      break;
    case DDSG:
      cmd = 0x7A;
      break;
  }
  float raw = directCommandRead(cmd)/10.0;
  return (raw - 273.15);
}

// Check Primary Protection status
bq_protection_t BQ76952::getProtectionStatus(void) {
  bq_protection_t prot;
  byte regData = (byte)directCommandRead(DIR_CMD_FPROTEC);
  prot.bits.SC_DCHG = bitRead(regData, BIT_SA_SC_DCHG);
  prot.bits.OC2_DCHG = bitRead(regData, BIT_SA_OC2_DCHG);
  prot.bits.OC1_DCHG = bitRead(regData, BIT_SA_OC1_DCHG);
  prot.bits.OC_CHG = bitRead(regData, BIT_SA_OC_CHG);
  prot.bits.CELL_OV = bitRead(regData, BIT_SA_CELL_OV);
  prot.bits.CELL_UV = bitRead(regData, BIT_SA_CELL_UV);
  return prot;
}

// Check Temperature Protection status
bq_temp_t BQ76952::getTemperatureStatus(void) {
  bq_temp_t prot;
  byte regData = (byte)directCommandRead(DIR_CMD_FTEMP);
  prot.bits.OVERTEMP_FET = bitRead(regData, BIT_SB_OTC);
  prot.bits.OVERTEMP_INTERNAL = bitRead(regData, BIT_SB_OTINT);
  prot.bits.OVERTEMP_DCHG = bitRead(regData, BIT_SB_OTD);
  prot.bits.OVERTEMP_CHG = bitRead(regData, BIT_SB_OTC);
  prot.bits.UNDERTEMP_INTERNAL = bitRead(regData, BIT_SB_UTINT);
  prot.bits.UNDERTEMP_DCHG = bitRead(regData, BIT_SB_UTD);
  prot.bits.UNDERTEMP_CHG = bitRead(regData, BIT_SB_UTC);
  return prot;
}

///// UTILITY FUNCTIONS /////

void BQ76952::setDebug(bool d) {
  BQ_DEBUG = d;
}

// Debug printing utilites
void BQ76952::debugPrint(const char* msg) {
  if(BQ_DEBUG)
    Serial.print(msg);
}

void BQ76952::debugPrintln(const char* msg) {
  if(BQ_DEBUG)
    Serial.println(msg);
}

void BQ76952::debugPrint(const __FlashStringHelper* msg) {
  if(BQ_DEBUG)
    Serial.print(msg);
}

void BQ76952::debugPrintln(const __FlashStringHelper* msg) {
  if(BQ_DEBUG)
    Serial.println(msg);
}

void BQ76952::debugPrintlnCmd(unsigned int cmd) {
  if(BQ_DEBUG) {
    Serial.print(F("0x"));
    Serial.println(cmd, HEX);
  }
}








