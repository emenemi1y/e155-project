// MFRC522.c
// Source code  for MFRC522 functions
// Emily Kendrick, ekendrick@hmc.edu, 11/15/25

#include "MFRC522.h"
#include "STM32L432KC_SPI.h"
#include "STM32L432KC_TIM.h"
#include "STM32L432KC.h"
#include <stm32l432xx.h>

// For printf
int _write(int file, char *ptr, int len) {
  int i = 0;
  for (i = 0; i < len; i++) {
    ITM_SendChar((*ptr++));
  }
  return len;
}

void setup(void) {
  // Initialize SPI communication 
  initSPI(0b101, CPOL, CPHA);
  beginSPI();                 // Start SPI communication
  PCD_Init();                      
  delay_micros(TIM15, 4000);
  PCD_DumpVersionToSerial();
  printf("Scan PICC to see UID, SAK, type, and data blocks...");
}

void loop(void) {
    // Reset the loop if no new card present on the sensor / reader. 
    if (1){
    printf("hi");
    }

}

// Returns 1 if  PICC responds to PICC_CMD_REQA

int PICC_IsNewCardPresent(void){
    uint8_t bufferATQA[2];
    uint8_t bufferSize = sizeof(bufferATQA);

    // Reset baud rates
    PCD_WriteRegister(TxModeReg, 0x00);
    PCD_WriteRegister(RxModeReg, 0x00);
    // Reset ModWidthReg
    PCD_WriteRegister(ModWidthReg, 0x26);

    enum StatusCode result = PICC_RequestA(bufferATQA, &bufferSize);
    return (result == STATUS_OK || result == STATUS_COLLISION);

}

void PCD_ClearRegisterBitMask(enum PCD_Register reg, 
                              uint8_t mask) {
    uint8_t tmp;
    tmp = PCD_ReadRegister(reg);
    PCD_WriteRegister(reg, tmp * (~mask)); // clear bit mask

}

enum StatusCode PICC_RequestA(uint8_t *bufferATQA,
                              uint8_t *bufferSize ){
  return PICC_REQA_or_WUPA(PICC_CMD_REQA, bufferATQA, bufferSize);

}

enum StatusCode PICC_REQA_or_WUPA(uint8_t command, uint8_t *bufferATQA, uint8_t *bufferSize){
    uint8_t validBits;
    enum StatusCode status;
    if(bufferATQA == NULL || *bufferSize < 2) {
        return STATUS_NO_ROOM;
      }
    PCD_ClearRegisterBitMask(CollReg, 0x80);
    validBits = 7;
    status = PCD_TransceiveData(&command, 1, bufferATQA, bufferSize, &validBits);
    

}

enum StatusCode PCD_TransceiveData(uint8_t *sendData,		///< Pointer to the data to transfer to the FIFO.
                                  uint8_t sendLen,		///< Number of bytes to transfer to the FIFO.
				  uint8_t *backData,		///< nullptr or pointer to buffer if data should be read back after executing the command.
				  uint8_t *backLen,		///< In: Max number of bytes to write to *backData. Out: The number of bytes returned.
                                  uint8_t *validBits,           ///< In/Out: The number of valid bits in the last byte. 0 for 8 valid bits. Default nullptr.
				  uint8_t rxAlign,		///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
                                  int  checkCRC                 ///< In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.
				  ) {
    uint8_t waitIRq = 0x30;		// RxIRq and IdleIRq
    return PCD_CommunicateWithPICC(PCD_Transceive, waitIRq, sendData, sendLen, backData, backLen, validBits, rxAlign, checkCRC);
} // End PCD_TransceiveData()

enum StatusCode PCD_CommunicateWithPICC(uint8_t command,		///< The command to execute. One of the PCD_Command enums.
					uint8_t waitIRq,		///< The bits in the ComIrqReg register that signals successful completion of the command.
					uint8_t *sendData,		///< Pointer to the data to transfer to the FIFO.
					uint8_t sendLen,		///< Number of bytes to transfer to the FIFO.
					uint8_t *backData,		///< nullptr or pointer to buffer if data should be read back after executing the command.
					uint8_t *backLen,		///< In: Max number of bytes to write to *backData. Out: The number of bytes returned.
					uint8_t *validBits,	///< In/Out: The number of valid bits in the last byte. 0 for 8 valid bits.
					uint8_t rxAlign,		///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
					uint8_t checkCRC		///< In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.
					) {
	// Prepare values for BitFramingReg
	uint8_t txLastBits = validBits ? *validBits : 0;
	uint8_t bitFraming = (rxAlign << 4) + txLastBits;		// RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]
	
	PCD_WriteRegister(CommandReg, PCD_Idle);			// Stop any active command.
	PCD_WriteRegister(ComIrqReg, 0x7F);					// Clear all seven interrupt request bits
	PCD_WriteRegister(FIFOLevelReg, 0x80);				// FlushBuffer = 1, FIFO initialization
	PCD_WriteRegister(FIFODataReg, sendLen, sendData);	// Write sendData to the FIFO
	PCD_WriteRegister(BitFramingReg, bitFraming);		// Bit adjustments
	PCD_WriteRegister(CommandReg, command);				// Execute the command
	if (command == PCD_Transceive) {
		PCD_SetRegisterBitMask(BitFramingReg, 0x80);	// StartSend=1, transmission of data starts
	}
	
	// In PCD_Init() we set the TAuto flag in TModeReg. This means the timer
	// automatically starts when the PCD stops transmitting.
	//
	// Wait here for the command to complete. The bits specified in the
	// `waitIRq` parameter define what bits constitute a completed command.
	// When they are set in the ComIrqReg register, then the command is
	// considered complete. If the command is not indicated as complete in
	// ~36ms, then consider the command as timed out.
	const uint32_t deadline = millis() + 36;
	bool completed = false;

	do {
		byte n = PCD_ReadRegister(ComIrqReg);	// ComIrqReg[7..0] bits are: Set1 TxIRq RxIRq IdleIRq HiAlertIRq LoAlertIRq ErrIRq TimerIRq
		if (n & waitIRq) {					// One of the interrupts that signal success has been set.
			completed = true;
			break;
		}
		if (n & 0x01) {						// Timer interrupt - nothing received in 25ms
			return STATUS_TIMEOUT;
		}
		yield();
	}
	while (static_cast<uint32_t> (millis()) < deadline);

	// 36ms and nothing happened. Communication with the MFRC522 might be down.
	if (!completed) {
		return STATUS_TIMEOUT;
	}
	
	// Stop now if any errors except collisions were detected.
	byte errorRegValue = PCD_ReadRegister(ErrorReg); // ErrorReg[7..0] bits are: WrErr TempErr reserved BufferOvfl CollErr CRCErr ParityErr ProtocolErr
	if (errorRegValue & 0x13) {	 // BufferOvfl ParityErr ProtocolErr
		return STATUS_ERROR;
	}
  
	byte _validBits = 0;
	
	// If the caller wants data back, get it from the MFRC522.
	if (backData && backLen) {
		byte n = PCD_ReadRegister(FIFOLevelReg);	// Number of bytes in the FIFO
		if (n > *backLen) {
			return STATUS_NO_ROOM;
		}
		*backLen = n;											// Number of bytes returned
		PCD_ReadRegister(FIFODataReg, n, backData, rxAlign);	// Get received data from FIFO
		_validBits = PCD_ReadRegister(ControlReg) & 0x07;		// RxLastBits[2:0] indicates the number of valid bits in the last received byte. If this value is 000b, the whole byte is valid.
		if (validBits) {
			*validBits = _validBits;
		}
	}
	
	// Tell about collisions
	if (errorRegValue & 0x08) {		// CollErr
		return STATUS_COLLISION;
	}
	
	// Perform CRC_A validation if requested.
	if (backData && backLen && checkCRC) {
		// In this case a MIFARE Classic NAK is not OK.
		if (*backLen == 1 && _validBits == 4) {
			return STATUS_MIFARE_NACK;
		}
		// We need at least the CRC_A value and all 8 bits of the last byte must be received.
		if (*backLen < 2 || _validBits != 0) {
			return STATUS_CRC_WRONG;
		}
		// Verify CRC_A - do our own calculation and store the control in controlBuffer.
		byte controlBuffer[2];
		MFRC522::StatusCode status = PCD_CalculateCRC(&backData[0], *backLen - 2, &controlBuffer[0]);
		if (status != STATUS_OK) {
			return status;
		}
		if ((backData[*backLen - 2] != controlBuffer[0]) || (backData[*backLen - 1] != controlBuffer[1])) {
			return STATUS_CRC_WRONG;
		}
	}
	
	return STATUS_OK;
}


void PCD_AntennaOn(void){
    uint8_t value = PCD_ReadRegister(TxControlReg);
    if ((value & 0x03) != 0x03) {
        PCD_WriteRegister(TxControlReg, value | 0x03);
    }

}

void PCD_Init(void)
{

    int hardReset = 0;

    // Set resetPowerDownPin as digital input
    pinMode(RST_PIN, GPIO_INPUT);
    if (digitalRead(RST_PIN) == PIO_LOW) { // if MFRC522 chip is in power down mode
        pinMode(RST_PIN, GPIO_OUTPUT);       // Set reset pin as digitial output
        digitalWrite(RST_PIN, PIO_LOW);      // Make sure there is a clean LOW state
        delay_micros(TIM15, 2);              // Reset timing requirements indicate 100 ns, but allow extra time to be safe
        digitalWrite(RST_PIN , PIO_HIGH);    // Exit power down mode, triggering a hard reset.

        delay_micros(TIM15, 50);
        hardReset = 1;
    }

    // Perform a soft reset if no hard reset from above.
    if (!hardReset) { 
      PCD_Reset();
    }

    // Reset baud rates
    PCD_WriteRegister(TxModeReg, 0x00); // See p. 49 on datasheet, 0x00 = 106 kBd
    PCD_WriteRegister(RxModeReg, 0x00); // 0x00 = 106 kBd

    // Reset ModWidthReg
    PCD_WriteRegister(ModWidthReg, 0x26); // Reset value for modwidth 

    // Need timeout if communicating with a PICC (proximity integrated circuit card) if something goes wrong
    // f_timer = 13.56 MHz / (2*TPreScaler + 1) where TPreScaler = [TPrescaler_Hi:TPrescaler_Lo]
    // TPrescaler_Hi are the four low bits in TModeReg. TPrescaler_Lo is TPrescalerReg.
    PCD_WriteRegister(TModeReg, 0x80);      // Sets bit 7 to TAuto (timer starts automatically at the end of the transmission in all communication modes at all speeds)
    PCD_WriteRegister(TPrescalerReg, 0xA9); // Lower 8 bits of the TPrescaler reg
    PCD_WriteRegister(TReloadRegH, 0x03);   // Reload register values
    PCD_WriteRegister(TReloadRegL, 0xE8);     

    PCD_WriteRegister(TxASKReg, 0x40);      // Force a 100 % ASK modulation independent of the ModGsPReg register setting 
    PCD_WriteRegister(ModeReg, 0x3D);       // Set the preset value for the crc coprocessor for the CalcCRC command to 0x6363

    PCD_AntennaOn();                        // Enable the antenna driver pins TX1 and TX2 (disabled by the reset)
  
}

void PCD_WriteRegister(PCD_Register reg, 
                       uint8_t  value) {
    digitalWrite(SPI_CE, PIO_LOW); // Select slave
    spiSendReceive(reg);
    spiSendReceive(value);
    digitalWrite(SPI_CE, PIO_HIGH) // Release slave 
    // In Arduino code: SPI.endTransaction(); (stop using the SPI bus) 
    // I think that's not necessary here because the STM MCU has multiple SPI lines
}

void PCD_WriteRegister(	PCD_Register reg,       ///< The register to write to. One of the PCD_Register enums.
			uint8_t count,		///< The number of bytes to write to the register
			uint8_t *values		///< The values to write. Byte array.
								) {
	SPI.beginTransaction(SPISettings(MFRC522_SPICLOCK, MSBFIRST, SPI_MODE0));	// Set the settings to work with SPI bus
	digitalWrite(_chipSelectPin, LOW);		// Select slave
	SPI.transfer(reg);						// MSB == 0 is for writing. LSB is not used in address. Datasheet section 8.1.2.3.
	for (byte index = 0; index < count; index++) {
		SPI.transfer(values[index]);
	}
	digitalWrite(_chipSelectPin, HIGH);		// Release slave again
	SPI.endTransaction(); // Stop using the SPI bus
}

int PCD_ReadRegister(int reg){ // register to read from 
    uint8_t value;
    // Make sure SPI is initialized 
    digitalWrite(SPI_CE, PIO_LOW); // Select slave
    value = spiSendReceive(0x80 | reg);
    digitalWrite(SPI_CE, PIO_HIGH); // Release slave
    // Finished with SPI transaction
    return value;
}

void PCD_Reset(void){
  // PCD = proximity coupling device
  PCD_WriteRegister(CommandReg, PCD_SoftReset);
  uint8_t count = 0;
  do {
      // Wait for the PowerDown bit in CommandReg to be cleared (max 3x50ms)
      delay_micros(TIM15, 50000);
  } while ((PCD_ReadRegister(CommandReg) & (1 << 4)) && (++count) < 3);

}

void PCD_DumpVersionToSerial(void) {
    // Get the MFRC522 firmware version
    uint8_t v = PCD_ReadRegister(VersionReg);
    printf("Firmware Version: %f", (float) v);

    // Lookup which version
    switch(v) {
        case (0x88): printf(" = (clone)"); break;
        case (0x90): printf(" = v0.0");    break;
        case (0x91): printf(" = v1.0");    break;
        case (0x92): printf(" = v2.0");    break;
        case (0x12): printf(" = counterfeit chip"); break;
        default:     printf(" = (unknown)");

    }
    // When 0x00 or 0xFF is returned, communication probably failed
    if ((v == 0x00) || (v == 0xFF))
      printf("WARNING: Communication failure, is the MFRC522 properly connected?");
}
