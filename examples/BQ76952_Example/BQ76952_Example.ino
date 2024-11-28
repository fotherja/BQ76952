/*
* Description :   Custom software for the OpenBMS project by Martin Jäger, Lead Developer & Founder | Libre Solar
* Author      :   James Fotherby
* Date        :   27/11/2024
* License     :   MIT
* This code is published as open source software. Feel free to share/modify.
*
*
*  - This example runs on the ESP32-C3 on the OpenBMS hardware. Current sense resistor = 300uR 
*  - Configures the BQ76952 and all of its parameters to be suitable for a 16S LiFePO4 315Ah home battery system powering a 5000W Vitron inverter 
*/

#include <BQ76952.h>

#define LED_GREEN_PIN       0
#define LED_RED_PIN         1
#define ALERT_PIN           2
#define BUTTON_LOW_PIN      3

#define I2C_SDA_PIN         8
#define I2C_SCL_PIN         9                                       // Pulling to ground and reseting device enters it into BOOT mode

BQ76952 bms;

void setup() {
  Serial.begin(115200);
  bms.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  bms.reset();
  delay(100); 

  // Configure the BQ76952 just the way we like it:
  bms.setConnectedCells(4);                                         // Set to 4 Cells attached
  bms.writeByteToMemory(FET_Options, 0x1D);                         // This enable the Pre-discharge function
  bms.writeByteToMemory(DA_Configuration, 0x06);                    // Set current to report in centiamps (10mA)

  bms.writeByteToMemory(Enabled_Protections_A, 0b10101100);         // Enable CUV, COV, Short circuit and OCD1 protection
  bms.writeByteToMemory(CUV_Threshold, 60);                         // Set Cell undervoltage protection to 3.00 volts
  bms.writeByteToMemory(COV_Threshold, 67);                         // Set Cell overvoltage protection to 3.40 volts
  bms.writeByteToMemory(OCD1_Threshold, 25);                        // Set OCD to 50mV across shunt (167 A)
  bms.writeByteToMemory(OCD1_Delay, 127);                           // Set OCD to 425 ms (max)
  bms.writeByteToMemory(SCD_Threshold, SCD_80);                     // Set SCD to 80 mV (265 A)
  bms.writeByteToMemory(SCD_Delay, 2);                              // Keep default SCD detection time of 15 us

  bms.writeIntToMemory(Mfg_Status_Init, 0x0050);                    // This takes the BQ out of manufacture mode and allows BQ to enable the FETs subject to protections

  // Commands:                  
  bms.setFET(ALL, ON);                                              // Enable FETS
}

void loop() {
  delay(200);

  Serial.print(bms.getCellVoltage(1)); Serial.print(", ");
  Serial.print(bms.getCellVoltage(2)); Serial.print(", ");
  Serial.print(bms.getCellVoltage(3)); Serial.print(", ");
  Serial.print(bms.getCellVoltage(4)); Serial.print(", ");
  Serial.print(bms.getCellVoltage(17)); Serial.print(", ");         // Stack voltage
  Serial.print(bms.getCellVoltage(18)); Serial.print(", ");         // Pack voltage
  Serial.println(bms.getCurrent());  




  // byte* charge_data = bms.subCommandwithdata(DASTATUS_6, 32);

  //   // Extract 32-bit signed integer values
  //   int32_t AccumulatedChargeInteger = 
  //       ((int32_t)charge_data[3] << 24) |
  //       ((int32_t)charge_data[2] << 16) |
  //       ((int32_t)charge_data[1] << 8) |
  //       charge_data[0];

  //   int32_t AccumulatedChargeFractional = 
  //       ((int32_t)charge_data[7] << 24) |
  //       ((int32_t)charge_data[6] << 16) |
  //       ((int32_t)charge_data[5] << 8) |
  //       charge_data[4];

  //   // Extract 32-bit unsigned integer values
  //   uint32_t AccumulatedTime = 
  //       ((uint32_t)charge_data[11] << 24) |
  //       ((uint32_t)charge_data[10] << 16) |
  //       ((uint32_t)charge_data[9] << 8) |
  //       charge_data[8];

  //   uint32_t CFETOFFCounts = 
  //       ((uint32_t)charge_data[15] << 24) |
  //       ((uint32_t)charge_data[14] << 16) |
  //       ((uint32_t)charge_data[13] << 8) |
  //       charge_data[12];

  //   uint32_t DFETOFFCounts = 
  //       ((uint32_t)charge_data[19] << 24) |
  //       ((uint32_t)charge_data[18] << 16) |
  //       ((uint32_t)charge_data[17] << 8) |
  //       charge_data[16];

  //   uint32_t ALERTCounts = 
  //       ((uint32_t)charge_data[23] << 24) |
  //       ((uint32_t)charge_data[22] << 16) |
  //       ((uint32_t)charge_data[21] << 8) |
  //       charge_data[20];

  //   uint32_t TS1Counts = 
  //       ((uint32_t)charge_data[27] << 24) |
  //       ((uint32_t)charge_data[26] << 16) |
  //       ((uint32_t)charge_data[25] << 8) |
  //       charge_data[24];

  //   uint32_t TS2Counts = 
  //       ((uint32_t)charge_data[31] << 24) |
  //       ((uint32_t)charge_data[30] << 16) |
  //       ((uint32_t)charge_data[29] << 8) |
  //       charge_data[28];

  //   // Print the extracted data for verification
  //   Serial.println(F("Extracted Charge Data:"));
  //   Serial.print(F("Accumulated Charge (Integer): "));
  //   Serial.println(AccumulatedChargeInteger);

  //   Serial.print(F("Accumulated Charge (Fractional): "));
  //   Serial.println(AccumulatedChargeFractional);

  //   Serial.print(F("Accumulated Time: "));
  //   Serial.println(AccumulatedTime);

  //   Serial.print(F("CFETOFF Counts: "));
  //   Serial.println(CFETOFFCounts);

  //   Serial.print(F("DFETOFF Counts: "));
  //   Serial.println(DFETOFFCounts);

  //   Serial.print(F("ALERT Counts: "));
  //   Serial.println(ALERTCounts);

  //   Serial.print(F("TS1 Counts: "));
  //   Serial.println(TS1Counts);

  //   Serial.print(F("TS2 Counts: "));
  //   Serial.println(TS2Counts);


  //-----------------------------------------------------------------------------------------
    //byte* voltage_data = bms.subCommandwithdata(DASTATUS_5, 32);            

    // // Define variables for each data item
    // uint16_t VREG18 = (voltage_data[1] << 8) | voltage_data[0]; // 16-bit
    // uint16_t VSS = (voltage_data[3] << 8) | voltage_data[2];    // 16-bit
    // uint16_t MaxCellVoltage = (voltage_data[5] << 8) | voltage_data[4]; // 16-bit
    // uint16_t MinCellVoltage = (voltage_data[7] << 8) | voltage_data[6]; // 16-bit
    // uint16_t BatteryVoltageSum = (voltage_data[9] << 8) | voltage_data[8]; // 16-bit
    // uint16_t AvgCellTemperature = (voltage_data[11] << 8) | voltage_data[10]; // 16-bit
    // uint16_t FETTemperature = (voltage_data[13] << 8) | voltage_data[12]; // 16-bit
    // uint16_t MaxCellTemperature = (voltage_data[15] << 8) | voltage_data[14]; // 16-bit
    // uint16_t MinCellTemperature = (voltage_data[17] << 8) | voltage_data[16]; // 16-bit
    // uint16_t AvgCellTemperatureAgain = (voltage_data[19] << 8) | voltage_data[18]; // 16-bit
    // uint16_t CC3Current = (voltage_data[21] << 8) | voltage_data[20]; // 16-bit
    // uint16_t CC1Current = (voltage_data[23] << 8) | voltage_data[22]; // 16-bit
    // uint32_t CC2Counts = ((uint32_t)voltage_data[26] << 24) |
    //                      ((uint32_t)voltage_data[25] << 16) |
    //                      ((uint32_t)voltage_data[24] << 8) |
    //                      voltage_data[27]; // 32-bit
    // uint32_t CC3Counts = ((uint32_t)voltage_data[31] << 24) |
    //                      ((uint32_t)voltage_data[30] << 16) |
    //                      ((uint32_t)voltage_data[29] << 8) |
    //                      voltage_data[28]; // 32-bit

    // // Print the extracted data for verification
    // Serial.println(F("Extracted Voltage Data:"));
    // Serial.print(F("VREG18: ")); Serial.println(VREG18);
    // Serial.print(F("VSS: ")); Serial.println(VSS);
    // Serial.print(F("Max Cell Voltage: ")); Serial.println(MaxCellVoltage);
    // Serial.print(F("Min Cell Voltage: ")); Serial.println(MinCellVoltage);
    // Serial.print(F("Battery Voltage Sum: ")); Serial.println(BatteryVoltageSum);
    // Serial.print(F("Avg Cell Temperature: ")); Serial.println(AvgCellTemperature);
    // Serial.print(F("FET Temperature: ")); Serial.println(FETTemperature);
    // Serial.print(F("Max Cell Temperature: ")); Serial.println(MaxCellTemperature);
    // Serial.print(F("Min Cell Temperature: ")); Serial.println(MinCellTemperature);
    // Serial.print(F("Avg Cell Temperature Again: ")); Serial.println(AvgCellTemperatureAgain);
    // Serial.print(F("CC3 Current: ")); Serial.println(CC3Current);
    // Serial.print(F("CC1 Current: ")); Serial.println(CC1Current);
    // Serial.print(F("CC2 Counts: ")); Serial.println(CC2Counts);
    // Serial.print(F("CC3 Counts: ")); Serial.println(CC3Counts);
}





