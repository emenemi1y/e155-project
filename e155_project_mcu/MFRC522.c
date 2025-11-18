// MFRC522.c
// Source code  for MFRC522 functions
// Emily Kendrick, ekendrick@hmc.edu, 11/15/25

#include "MFRC522.h"
#include "STM32L432KC_SPI.h"
#include "STM32L432KC_TIM.h"
#include "STM32L432KC.h"
#include <stm32l432xx.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>



// For printf
int _write(int file, char *ptr, int len) {
  int i = 0;
  for (i = 0; i < len; i++) {
    ITM_SendChar((*ptr++));
  }
  return len;
}


/**
 * Writes a byte to the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.2.
 */
 
void PCD_WriteRegisterMulti(enum PCD_Register reg,	///< The register to write to. One of the PCD_Register enums.
                            uint8_t count,              ///< The number of bytes to write to the register
                            uint8_t *values		///< The value to write.
								) {
    digitalWrite(SPI_CE, PIO_LOW); // Select slave
    spiSendReceive(reg);
    for (uint8_t index = 0; index < count; index++) {
      spiSendReceive(values[index]);
    }
    digitalWrite(SPI_CE, PIO_HIGH) // Release slave 
    // In Arduino code: SPI.endTransaction(); (stop using the SPI bus) 
    // I think that's not necessary here because the STM MCU has multiple SPI lines
} // End PCD_WriteRegister()


void PCD_WriteRegister(enum PCD_Register reg,  ///< The register to write to. One of the PCD_Register enums.
                       uint8_t value) {        ///< The value to write.
    digitalWrite(SPI_CE, PIO_LOW); // Select slave
    spiSendReceive(reg);
    spiSendReceive(value);
    digitalWrite(SPI_CE, PIO_HIGH); // Release slave
}

/**
 * Reads a number of bytes from the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.2.
 */

void PCD_ReadRegisterMulti(enum PCD_Register reg,   ///< The register to read from.
                     uint8_t count,           ///< The number of bytes to read
                     uint8_t *values,         ///< Byte array to store the values in.
                     uint8_t rxAlign          ///< Only bit positions rxAlign...7 in values[0] are updated
                     ){ 

    if (count == 0) return;

    uint8_t address = 0x80 | reg;
    uint8_t index = 0;
    digitalWrite(SPI_CE, PIO_LOW);
    count --;
    spiSendReceive(address);

    if (rxAlign) { // Only update bit positions rxAlign..7 in values[0]
        // Create bit mask for bit positions rxAlign..7
        uint8_t mask = (0xFF << rxAlign) & 0xFF;
        // Read value and tell that we want to read the same address again.
        uint8_t value = spiSendReceive(address);
        // Apply mask to both current value of values[0] and the new data in value.
        values[0] = (values[0] & ~mask) | (value & mask);
        index++;   
    }
    while (index < count) {
        values[index] = spiSendReceive(address); // Read value and tell that we want to read the same address again
        index++;
    }
    values[index] = spiSendReceive(0); // Send 0 to stop reading
    digitalWrite(SPI_CE, PIO_HIGH); // Release slave
}

uint8_t PCD_ReadRegister(enum PCD_Register reg) { ///< The register to read from.
    uint8_t value;
    digitalWrite(SPI_CE, PIO_LOW);  // Select slave
    spiSendReceive(0x80 | reg);
    value = spiSendReceive(0);
    digitalWrite(SPI_CE, PIO_HIGH); // Release slave
    return value;
}

// Sets the bits given in mask in register reg
void PCD_SetRegisterBitMask(enum PCD_Register reg,  ///< The register to update. One of the PCD_Register enums.
                            uint8_t mask ){         ///< The bits to set
  uint8_t tmp;
  tmp = PCD_ReadRegister(reg);
  PCD_WriteRegister(reg, tmp | mask); // Set bit mask
}

// Clears the bits given in mask in register reg
void PCD_ClearRegisterBitMask(enum PCD_Register reg,  ///< The register to update. 
                              uint8_t mask){          ///< The bits to clear 
    uint8_t tmp;
    tmp = PCD_ReadRegister(reg);
    PCD_WriteRegister(reg, tmp & (~mask));
}


/////////////////////////////////////////////////////////////////////////////////////
// Functions for manipulating the MFRC522
/////////////////////////////////////////////////////////////////////////////////////


void PCD_Init(void) {
	int hardReset = 0;

	// Set the chipSelectPin as digital output, do not select the slave yet
	pinMode(SPI_CE, GPIO_OUTPUT);
	digitalWrite(SPI_CE, PIO_HIGH);
	
	// If a valid pin number has been set, pull device out of power down / reset state.
	if (RST_PIN != UNUSED_PIN) {
		// First set the resetPowerDownPin as digital input, to check the MFRC522 power down mode.
		pinMode(RST_PIN, GPIO_INPUT);
	
		if (digitalRead(RST_PIN) == PIO_LOW) {          // The MFRC522 chip is in power down mode.
			pinMode(RST_PIN, GPIO_OUTPUT);		// Now set the resetPowerDownPin as digital output.
			digitalWrite(RST_PIN, PIO_LOW);		// Make sure we have a clean LOW state.
			delay_micros(TIM15, 2);				// 8.8.1 Reset timing requirements says about 100ns. Let us be generous: 2μsl
			digitalWrite(RST_PIN, PIO_HIGH);		// Exit power down mode. This triggers a hard reset.
			// Section 8.8.2 in the datasheet says the oscillator start-up time is the start up time of the crystal + 37,74μs. Let us be generous: 50ms.
			delay_micros(100);
			hardReset = 1;
		}
	}

	if (!hardReset) { // Perform a soft reset if we haven't triggered a hard reset above.
	   PCD_Reset();
	}
	
	// Reset baud rates
	PCD_WriteRegister(TxModeReg, 0x00);
	PCD_WriteRegister(RxModeReg, 0x00);
	// Reset ModWidthReg
	PCD_WriteRegister(ModWidthReg, 0x26);

	// When communicating with a PICC we need a timeout if something goes wrong.
	// f_timer = 13.56 MHz / (2*TPreScaler+1) where TPreScaler = [TPrescaler_Hi:TPrescaler_Lo].
	// TPrescaler_Hi are the four low bits in TModeReg. TPrescaler_Lo is TPrescalerReg.
	PCD_WriteRegister(TModeReg, 0x80);			// TAuto=1; timer starts automatically at the end of the transmission in all communication modes at all speeds
	PCD_WriteRegister(TPrescalerReg, 0xA9);		// TPreScaler = TModeReg[3..0]:TPrescalerReg, ie 0x0A9 = 169 => f_timer=40kHz, ie a timer period of 25μs.
	PCD_WriteRegister(TReloadRegH, 0x03);		// Reload timer with 0x3E8 = 1000, ie 25ms before timeout.
	PCD_WriteRegister(TReloadRegL, 0xE8);
	
	PCD_WriteRegister(TxASKReg, 0x40);		// Default 0x00. Force a 100 % ASK modulation independent of the ModGsPReg register setting
	PCD_WriteRegister(ModeReg, 0x3D);		// Default 0x3F. Set the preset value for the CRC coprocessor for the CalcCRC command to 0x6363 (ISO 14443-3 part 6.2.4)
	PCD_AntennaOn();						// Enable the antenna driver pins TX1 and TX2 (they were disabled by the reset)
} // End PCD_Init()


// Turns the antenna on by enabling pins TX1 and TX2
void PCD_AntennaOn(void){
  uint8_t value = PCD_ReadRegister(TxControlReg);
  if ((value & 0x03) != (0x03)) {
      PCD_WriteRegister(TxControlReg, value | 0x03);
  }
}

// Turns the antenna off by disabling pins TX1 and TX2
void PCD_AntennaOff(void){
  PCD_ClearRegisterBitMask(TxControlReg, 0x03);
}

// Get the current Receiver Gain (RxGain[2:0] value
// Returns value of RxGain, scrubbed to the 3 bits used
uint8_t PCD_GetAntennaGain(void) {
  return PCD_ReadRegister(RFCfgReg) & (0x07<<4);
}

// Set the Receiver Gain (RxGain) to value specified by given mask
// Note: Given mask is scrubbed with (0x07<<4)=0111000b as RCFfgReg may use reserved bits
void PCD_SetAntennaGain(uint8_t mask) {
  if(PCD_GetAntennaGain() != mask){                      // Only update if there is a change
      PCD_ClearRegisterBitMask(RFCfgReg, (0x07<<4));     // clear needed to allow 000 pattern
      PCD_SetRegisterBitMask(RFCfgReg, mask & (0x07<<4)); // Only set RxGain[2:0] bits
  }
}

// Skipping PCD_PerformSelfTest

/////////////////////////////////////////////////////////////////////////////////////
// Power control
/////////////////////////////////////////////////////////////////////////////////////

//IMPORTANT NOTE!!!!
//Calling any other function that uses CommandReg will disable soft power down mode !!!

void PCD_SoftPowerDown(void) {                  //Note : Only soft power down mode is available throught software
    uint8_t val = PCD_ReadRegister(CommandReg); // Read state of the command register 
    val |= (1<<4);                              // set PowerDown bit ( bit 4 ) to 1 
    PCD_WriteRegister(CommandReg, val);         //write new value to the command register
}


void PCD_SoftPowerUp(void){
    uint8_t val = PCD_ReadRegister(CommandReg); // Read state of the command register 
    val &= ~(1<<4);                             // set PowerDown bit ( bit 4 ) to 0 
    PCD_WriteRegister(CommandReg, val);         //write new value to the command register

    // wait until PowerDown bit is cleared (this indicates end of wake up procedure) 
    // const uint32_t timeout = (uint32_t) millis() + 500;// create timer for timeout (just in case) 
    
    while(1 == 1){ // set timeout to 500 ms 
        val = PCD_ReadRegister(CommandReg);// Read state of the command register
        if(!(val & (1<<4))){ // if powerdown bit is 0 
           break;// wake up procedure is finished 
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////
// Functions for communicating with PICCs
/////////////////////////////////////////////////////////////////////////////////////

// Executes the Transceive command.
// Returns STATUS_OK on success, STATUS_??? otherwise 

enum StatusCode PCD_TransceiveData(uint8_t *sendData,		///< Pointer to the data to transfer to the FIFO.
				   uint8_t sendLen,		///< Number of bytes to transfer to the FIFO.
                                   uint8_t *backData,		///< nullptr or pointer to buffer if data should be read back after executing the command.
                                   uint8_t *backLen,		///< In: Max number of bytes to write to *backData. Out: The number of bytes returned.
                                   uint8_t *validBits,          ///< In/Out: The number of valid bits in the last byte. 0 for 8 valid bits. Default nullptr.
                                   uint8_t rxAlign,		///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
                                   uint8_t checkCRC		///< In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.
								 ) {
  uint8_t waitIRq = 0x30;		// RxIRq and IdleIRq
  return PCD_CommunicateWithPICC(PCD_Transceive, waitIRq, sendData, sendLen, backData, backLen, validBits, rxAlign, checkCRC);
} // End PCD_TransceiveData()

// Transfers data to the FIFO, executes a command, waits for completion and transfers data back from the FIFO
// Returns STATUS_OK on success, STATUS_??? otherwise

enum StatusCode PCD_CommunicateWithPICC(uint8_t command,	///< The command to execute. One of the PCD_Command enums.
					uint8_t waitIRq,	///< The bits in the ComIrqReg register that signals successful completion of the command.
					uint8_t *sendData,	///< Pointer to the data to transfer to the FIFO.
					uint8_t sendLen,	///< Number of bytes to transfer to the FIFO.
					uint8_t *backData,	///< nullptr or pointer to buffer if data should be read back after executing the command.
					uint8_t *backLen,	///< In: Max number of bytes to write to *backData. Out: The number of bytes returned.
					uint8_t *validBits,	///< In/Out: The number of valid bits in the last byte. 0 for 8 valid bits.
					uint8_t rxAlign,	///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
					uint8_t checkCRC	///< In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.
									 ) {
	// Prepare values for BitFramingReg
	uint8_t txLastBits = validBits ? *validBits : 0;
	uint8_t bitFraming = (rxAlign << 4) + txLastBits;	// RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]
	
	PCD_WriteRegister(CommandReg, PCD_Idle);		// Stop any active command.
	PCD_WriteRegister(ComIrqReg, 0x7F);			// Clear all seven interrupt request bits
	PCD_WriteRegister(FIFOLevelReg, 0x80);			// FlushBuffer = 1, FIFO initialization
	PCD_WriteRegisterMulti(FIFODataReg, sendLen, sendData);	// Write sendData to the FIFO
	PCD_WriteRegister(BitFramingReg, bitFraming);		// Bit adjustments
	PCD_WriteRegister(CommandReg, command);			// Execute the command
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
	const uint32_t deadline = 36;
	uint8_t completed = 0;
        start_timer(TIM15, deadline * 1000); // Start a timer for the deadline (deadline is in ms)

	do {
		uint8_t n = PCD_ReadRegister(ComIrqReg);	// ComIrqReg[7..0] bits are: Set1 TxIRq RxIRq IdleIRq HiAlertIRq LoAlertIRq ErrIRq TimerIRq
		if (n & waitIRq) {					// One of the interrupts that signal success has been set.
                  completed = 1;
                  break;
		}
		if (n & 0x01) {						// Timer interrupt - nothing received in 25ms
			return STATUS_TIMEOUT;
		}
	}
	while (!check_timer(TIM15));

	// 36ms and nothing happened. Communication with the MFRC522 might be down.
	if (!completed) {
		return STATUS_TIMEOUT;
	}
	
	// Stop now if any errors except collisions were detected.
	uint8_t errorRegValue = PCD_ReadRegister(ErrorReg); // ErrorReg[7..0] bits are: WrErr TempErr reserved BufferOvfl CollErr CRCErr ParityErr ProtocolErr
	if (errorRegValue & 0x13) {                         // BufferOvfl ParityErr ProtocolErr
		return STATUS_ERROR;
	}
  
	uint8_t _validBits = 0;
	
	// If the caller wants data back, get it from the MFRC522.
	if (backData && backLen) {
		uint8_t n = PCD_ReadRegister(FIFOLevelReg);	// Number of bytes in the FIFO
		if (n > *backLen) {
			return STATUS_NO_ROOM;
		}
		*backLen = n;											// Number of bytes returned
		PCD_ReadRegisterMulti(FIFODataReg, n, backData, rxAlign);	// Get received data from FIFO
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
		uint8_t controlBuffer[2];
		enum StatusCode status = PCD_CalculateCRC(&backData[0], *backLen - 2, &controlBuffer[0]);
		if (status != STATUS_OK) {
			return status;
		}
		if ((backData[*backLen - 2] != controlBuffer[0]) || (backData[*backLen - 1] != controlBuffer[1])) {
			return STATUS_CRC_WRONG;
		}
	}
	
	return STATUS_OK;
} // End PCD_CommunicateWithPICC()

// Transmists a REQuest command, Type A. Invites PICCs in state IDLE to go to READY and prepare for anticollision or selection. 7 bit frame.

enum StatusCode PICC_RequestA(uint8_t *bufferATQA,	///< The buffer to store the ATQA (Answer to request) in
                              uint8_t *bufferSize	///< Buffer size, at least two bytes. Also number of bytes returned if STATUS_OK.
										) {
	return PICC_REQA_or_WUPA(PICC_CMD_REQA, bufferATQA, bufferSize);
} // End PICC_RequestA()


// Transmits a Wake-UP command, Type A. Invites PICCs in state IDLE and HALT to go to READY(*) and prepare for anticollision or selection. 7 bit frame.
enum StatusCode PICC_WakeupA(uint8_t *bufferATQA,	///< The buffer to store the ATQA (Answer to request) in
                             uint8_t *bufferSize	///< Buffer size, at least two bytes. Also number of bytes returned if STATUS_OK.
										) {
	return PICC_REQA_or_WUPA(PICC_CMD_WUPA, bufferATQA, bufferSize);
} // End PICC_WakeupA()

// Transmits REQA or WUPA commands
enum StatusCode PICC_REQA_or_WUPA(uint8_t command, 	///< The command to send - PICC_CMD_REQA or PICC_CMD_WUPA
				  uint8_t *bufferATQA,	///< The buffer to store the ATQA (Answer to request) in
                                  uint8_t *bufferSize	///< Buffer size, at least two bytes. Also number of bytes returned if STATUS_OK.
				  ) {
	uint8_t validBits;
	enum StatusCode status;
	
	if (bufferATQA == NULL || *bufferSize < 2) {	// The ATQA response is 2 bytes long.
		return STATUS_NO_ROOM;
	}
	PCD_ClearRegisterBitMask(CollReg, 0x80);		// ValuesAfterColl=1 => Bits received after collision are cleared.
	validBits = 7;									// For REQA and WUPA we need the short frame format - transmit only 7 bits of the last (and only) byte. TxLastBits = BitFramingReg[2..0]
	status = PCD_TransceiveData(&command, 1, bufferATQA, bufferSize, &validBits);
	if (status != STATUS_OK) {
		return status;
	}
	if (*bufferSize != 2 || validBits != 0) {		// ATQA must be exactly 16 bits.
		return STATUS_ERROR;
	}
	return STATUS_OK;
} // End PICC_REQA_or_WUPA()


/**
 * Transmits SELECT/ANTICOLLISION commands to select a single PICC.
 * Before calling this function the PICCs must be placed in the READY(*) state by calling PICC_RequestA() or PICC_WakeupA().
 * On success:
 * 		- The chosen PICC is in state ACTIVE(*) and all other PICCs have returned to state IDLE/HALT. (Figure 7 of the ISO/IEC 14443-3 draft.)
 * 		- The UID size and value of the chosen PICC is returned in *uid along with the SAK.
 * 
 * A PICC UID consists of 4, 7 or 10 bytes.
 * Only 4 bytes can be specified in a SELECT command, so for the longer UIDs two or three iterations are used:
 * 		UID size	Number of UID bytes		Cascade levels		Example of PICC
 * 		========	===================		==============		===============
 * 		single				 4						1				MIFARE Classic
 * 		double				 7						2				MIFARE Ultralight
 * 		triple				10						3				Not currently in use?
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */


// Transmits SELECT/ANTICOLLISION commands to select a single PICC
enum StatusCode PICC_Select(Uid *uid,			///< Pointer to Uid struct. Normally output, but can also be used to supply a known UID.
			    uint8_t validBits		///< The number of known UID bits supplied in *uid. Normally 0. If set you must also supply uid->size.
										 ) {
	uint8_t uidComplete;
	uint8_t selectDone;
	uint8_t useCascadeTag;
	uint8_t cascadeLevel = 1;
	enum StatusCode result;
	uint8_t count;
	uint8_t checkBit;
	uint8_t index;
	uint8_t uidIndex;			// The first index in uid->uidByte[] that is used in the current Cascade Level.
	int8_t currentLevelKnownBits;		// The number of known UID bits in the current Cascade Level.
	uint8_t buffer[9];			// The SELECT/ANTICOLLISION commands uses a 7 byte standard frame + 2 bytes CRC_A
	uint8_t bufferUsed;			// The number of bytes used in the buffer, ie the number of bytes to transfer to the FIFO.
	uint8_t rxAlign;			// Used in BitFramingReg. Defines the bit position for the first bit received.
	uint8_t txLastBits;			// Used in BitFramingReg. The number of valid bits in the last transmitted byte. 
	uint8_t *responseBuffer;
	uint8_t responseLength;
	
	// Description of buffer structure:
	//		Byte 0: SEL 				Indicates the Cascade Level: PICC_CMD_SEL_CL1, PICC_CMD_SEL_CL2 or PICC_CMD_SEL_CL3
	//		Byte 1: NVB					Number of Valid Bits (in complete command, not just the UID): High nibble: complete bytes, Low nibble: Extra bits. 
	//		Byte 2: UID-data or CT		See explanation below. CT means Cascade Tag.
	//		Byte 3: UID-data
	//		Byte 4: UID-data
	//		Byte 5: UID-data
	//		Byte 6: BCC					Block Check Character - XOR of bytes 2-5
	//		Byte 7: CRC_A
	//		Byte 8: CRC_A
	// The BCC and CRC_A are only transmitted if we know all the UID bits of the current Cascade Level.
	//
	// Description of bytes 2-5: (Section 6.5.4 of the ISO/IEC 14443-3 draft: UID contents and cascade levels)
	//		UID size	Cascade level	Byte2	Byte3	Byte4	Byte5
	//		========	=============	=====	=====	=====	=====
	//		 4 bytes		1			uid0	uid1	uid2	uid3
	//		 7 bytes		1			CT		uid0	uid1	uid2
	//						2			uid3	uid4	uid5	uid6
	//		10 bytes		1			CT		uid0	uid1	uid2
	//						2			CT		uid3	uid4	uid5
	//						3			uid6	uid7	uid8	uid9
	
	// Sanity checks
	if (validBits > 80) {
		return STATUS_INVALID;
	}
	
	// Prepare MFRC522
	PCD_ClearRegisterBitMask(CollReg, 0x80);		// ValuesAfterColl=1 => Bits received after collision are cleared.
	
	// Repeat Cascade Level loop until we have a complete UID.
	uidComplete = 0;
	while (!uidComplete) {
		// Set the Cascade Level in the SEL byte, find out if we need to use the Cascade Tag in byte 2.
		switch (cascadeLevel) {
			case 1:
				buffer[0] = PICC_CMD_SEL_CL1;
				uidIndex = 0;
				useCascadeTag = validBits && uid->size > 4;	// When we know that the UID has more than 4 bytes
				break;
			
			case 2:
				buffer[0] = PICC_CMD_SEL_CL2;
				uidIndex = 3;
				useCascadeTag = validBits && uid->size > 7;	// When we know that the UID has more than 7 bytes
				break;
			
			case 3:
				buffer[0] = PICC_CMD_SEL_CL3;
				uidIndex = 6;
				useCascadeTag = 0;						// Never used in CL3.
				break;
			
			default:
				return STATUS_INTERNAL_ERROR;
				break;
		}
		
		// How many UID bits are known in this Cascade Level?
		currentLevelKnownBits = validBits - (8 * uidIndex);
		if (currentLevelKnownBits < 0) {
			currentLevelKnownBits = 0;
		}
		// Copy the known bits from uid->uidByte[] to buffer[]
		index = 2; // destination index in buffer[]
		if (useCascadeTag) {
			buffer[index++] = PICC_CMD_CT;
		}
		uint8_t bytesToCopy = currentLevelKnownBits / 8 + (currentLevelKnownBits % 8 ? 1 : 0); // The number of bytes needed to represent the known bits for this level.
		if (bytesToCopy) {
			uint8_t maxBytes = useCascadeTag ? 3 : 4; // Max 4 bytes in each Cascade Level. Only 3 left if we use the Cascade Tag
			if (bytesToCopy > maxBytes) {
				bytesToCopy = maxBytes;
			}
			for (count = 0; count < bytesToCopy; count++) {
				buffer[index++] = uid->uidByte[uidIndex + count];
			}
		}
		// Now that the data has been copied we need to include the 8 bits in CT in currentLevelKnownBits
		if (useCascadeTag) {
			currentLevelKnownBits += 8;
		}
		
		// Repeat anti collision loop until we can transmit all UID bits + BCC and receive a SAK - max 32 iterations.
		selectDone = 0;
		while (!selectDone) {
			// Find out how many bits and bytes to send and receive.
			if (currentLevelKnownBits >= 32) { // All UID bits in this Cascade Level are known. This is a SELECT.
				//Serial.print(F("SELECT: currentLevelKnownBits=")); Serial.println(currentLevelKnownBits, DEC);
				buffer[1] = 0x70; // NVB - Number of Valid Bits: Seven whole bytes
				// Calculate BCC - Block Check Character
				buffer[6] = buffer[2] ^ buffer[3] ^ buffer[4] ^ buffer[5];
				// Calculate CRC_A
				result = PCD_CalculateCRC(buffer, 7, &buffer[7]);
				if (result != STATUS_OK) {
					return result;
				}
				txLastBits		= 0; // 0 => All 8 bits are valid.
				bufferUsed		= 9;
				// Store response in the last 3 bytes of buffer (BCC and CRC_A - not needed after tx)
				responseBuffer	= &buffer[6];
				responseLength	= 3;
			}
			else { // This is an ANTICOLLISION.
				//Serial.print(F("ANTICOLLISION: currentLevelKnownBits=")); Serial.println(currentLevelKnownBits, DEC);
				txLastBits		= currentLevelKnownBits % 8;
				count			= currentLevelKnownBits / 8;	// Number of whole bytes in the UID part.
				index			= 2 + count;					// Number of whole bytes: SEL + NVB + UIDs
				buffer[1]		= (index << 4) + txLastBits;	// NVB - Number of Valid Bits
				bufferUsed		= index + (txLastBits ? 1 : 0);
				// Store response in the unused part of buffer
				responseBuffer	= &buffer[index];
				responseLength	= sizeof(buffer) - index;
			}
			
			// Set bit adjustments
			rxAlign = txLastBits;											// Having a separate variable is overkill. But it makes the next line easier to read.
			PCD_WriteRegister(BitFramingReg, (rxAlign << 4) + txLastBits);	// RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]
			
			// Transmit the buffer and receive the response.
			result = PCD_TransceiveData(buffer, bufferUsed, responseBuffer, &responseLength, &txLastBits, rxAlign);
			if (result == STATUS_COLLISION) { // More than one PICC in the field => collision.
				uint8_t valueOfCollReg = PCD_ReadRegister(CollReg); // CollReg[7..0] bits are: ValuesAfterColl reserved CollPosNotValid CollPos[4:0]
				if (valueOfCollReg & 0x20) { // CollPosNotValid
					return STATUS_COLLISION; // Without a valid collision position we cannot continue
				}
				uint8_t collisionPos = valueOfCollReg & 0x1F; // Values 0-31, 0 means bit 32.
				if (collisionPos == 0) {
					collisionPos = 32;
				}
				if (collisionPos <= currentLevelKnownBits) { // No progress - should not happen 
					return STATUS_INTERNAL_ERROR;
				}
				// Choose the PICC with the bit set.
				currentLevelKnownBits	= collisionPos;
				count			= currentLevelKnownBits % 8; // The bit to modify
				checkBit		= (currentLevelKnownBits - 1) % 8;
				index			= 1 + (currentLevelKnownBits / 8) + (count ? 1 : 0); // First byte is index 0.
				buffer[index]	|= (1 << checkBit);
			}
			else if (result != STATUS_OK) {
				return result;
			}
			else { // STATUS_OK
				if (currentLevelKnownBits >= 32) { // This was a SELECT.
					selectDone = 1; // No more anticollision 
					// We continue below outside the while.
				}
				else { // This was an ANTICOLLISION.
					// We now have all 32 bits of the UID in this Cascade Level
					currentLevelKnownBits = 32;
					// Run loop again to do the SELECT.
				}
			}
		} // End of while (!selectDone)
		
		// We do not check the CBB - it was constructed by us above.
		
		// Copy the found UID bytes from buffer[] to uid->uidByte[]
		index			= (buffer[2] == PICC_CMD_CT) ? 3 : 2; // source index in buffer[]
		bytesToCopy		= (buffer[2] == PICC_CMD_CT) ? 3 : 4;
		for (count = 0; count < bytesToCopy; count++) {
			uid->uidByte[uidIndex + count] = buffer[index++];
		}
		
		// Check response SAK (Select Acknowledge)
		if (responseLength != 3 || txLastBits != 0) { // SAK must be exactly 24 bits (1 byte + CRC_A).
			return STATUS_ERROR;
		}
		// Verify CRC_A - do our own calculation and store the control in buffer[2..3] - those bytes are not needed anymore.
		result = PCD_CalculateCRC(responseBuffer, 1, &buffer[2]);
		if (result != STATUS_OK) {
			return result;
		}
		if ((buffer[2] != responseBuffer[1]) || (buffer[3] != responseBuffer[2])) {
			return STATUS_CRC_WRONG;
		}
		if (responseBuffer[0] & 0x04) { // Cascade bit set - UID not complete yes
			cascadeLevel++;
		}
		else {
			uidComplete = 1;
			uid->sak = responseBuffer[0];
		}
	} // End of while (!uidComplete)
	
	// Set correct uid->size
	uid->size = 3 * cascadeLevel + 1;

	return STATUS_OK;
} // End PICC_Select()



// Instructs a PICC in state ACTIVE(*) to go to state HALT.

enum StatusCode PICC_HaltA(void) {
	enum StatusCode result;
	uint8_t buffer[4];
	
	// Build command buffer
	buffer[0] = PICC_CMD_HLTA;
	buffer[1] = 0;
	// Calculate CRC_A
	result = PCD_CalculateCRC(buffer, 2, &buffer[2]);
	if (result != STATUS_OK) {
		return result;
	}
	
	// Send the command.
	// The standard says:
	//		If the PICC responds with any modulation during a period of 1 ms after the end of the frame containing the
	//		HLTA command, this response shall be interpreted as 'not acknowledge'.
	// We interpret that this way: Only STATUS_TIMEOUT is a success.
	result = PCD_TransceiveData(buffer, sizeof(buffer), NULL, 0);
	if (result == STATUS_TIMEOUT) {
		return STATUS_OK;
	}
	if (result == STATUS_OK) { // That is ironically NOT ok in this case ;-)
		return STATUS_ERROR;
	}
	return result;
} // End PICC_HaltA()


/////////////////////////////////////////////////////////////////////////////////////
// Functions for communicating with MIFARE PICCs
/////////////////////////////////////////////////////////////////////////////////////

/**
 * Executes the MFRC522 MFAuthent command.
 * This command manages MIFARE authentication to enable a secure communication to any MIFARE Mini, MIFARE 1K and MIFARE 4K card.
 * The authentication is described in the MFRC522 datasheet section 10.3.1.9 and http://www.nxp.com/documents/data_sheet/MF1S503x.pdf section 10.1.
 * For use with MIFARE Classic PICCs.
 * The PICC must be selected - ie in state ACTIVE(*) - before calling this function.
 * Remember to call PCD_StopCrypto1() after communicating with the authenticated PICC - otherwise no new communications can start.
 * 
 * All keys are set to FFFFFFFFFFFFh at chip delivery.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise. Probably STATUS_TIMEOUT if you supply the wrong key.
 */

enum StatusCode PCD_Authenticate(uint8_t command,	///< PICC_CMD_MF_AUTH_KEY_A or PICC_CMD_MF_AUTH_KEY_B
				 uint8_t blockAddr, 	///< The block number. See numbering in the comments in the .h file.
				 MIFARE_Key *key,	///< Pointer to the Crypteo1 key to use (6 bytes)
                                 Uid *uid		///< Pointer to Uid struct. The first 4 bytes of the UID is used.
											) {
	uint8_t waitIRq = 0x10;		// IdleIRq
	
	// Build command buffer
	uint8_t sendData[12];
	sendData[0] = command;
	sendData[1] = blockAddr;
	for (uint8_t i = 0; i < MF_KEY_SIZE; i++) {	// 6 key bytes
		sendData[2+i] = key->keyByte[i];
	}
	// Use the last uid bytes as specified in http://cache.nxp.com/documents/application_note/AN10927.pdf
	// section 3.2.5 "MIFARE Classic Authentication".
	// The only missed case is the MF1Sxxxx shortcut activation,
	// but it requires cascade tag (CT) byte, that is not part of uid.
	for (uint8_t i = 0; i < 4; i++) {				// The last 4 bytes of the UID
		sendData[8+i] = uid->uidByte[i+uid->size-4];
	}
	
	// Start the authentication.
	return PCD_CommunicateWithPICC(PCD_MFAuthent, waitIRq, &sendData[0], sizeof(sendData));
} // End PCD_Authenticate()

// Used to exit the PCD from its authenticated state.

void PCD_StopCrypto1() {
	// Clear MFCrypto1On bit
	PCD_ClearRegisterBitMask(Status2Reg, 0x08); // Status2Reg[7..0] bits are: TempSensClear I2CForceHS reserved reserved MFCrypto1On ModemState[2:0]
} // End PCD_StopCrypto1()


// Reads 16 bytes (+ 2 bytes CRC_A) from the active PICC.

enum StatusCode MIFARE_Read(uint8_t blockAddr, 	///< MIFARE Classic: The block (0-0xff) number. MIFARE Ultralight: The first page to return data from.
			    uint8_t *buffer,	///< The buffer to store the data in
			    uint8_t *bufferSize	///< Buffer size, at least 18 bytes. Also number of bytes returned if STATUS_OK.
										) {
	enum StatusCode result;
	
	// Sanity check
	if (buffer == NULL || *bufferSize < 18) {
		return STATUS_NO_ROOM;
	}
	
	// Build command buffer
	buffer[0] = PICC_CMD_MF_READ;
	buffer[1] = blockAddr;
	// Calculate CRC_A
	result = PCD_CalculateCRC(buffer, 2, &buffer[2]);
	if (result != STATUS_OK) {
		return result;
	}
	
	// Transmit the buffer and receive the response, validate CRC_A.
	return PCD_TransceiveData(buffer, 4, buffer, bufferSize, NULL, 0, 1);
} // End MIFARE_Read()

// Writes 16 bytes to the active PICC.
enum StatusCode MIFARE_Write(uint8_t blockAddr, ///< MIFARE Classic: The block (0-0xff) number. MIFARE Ultralight: The page (2-15) to write to.
                             uint8_t *buffer,	///< The 16 bytes to write to the PICC
			     uint8_t bufferSize	///< Buffer size, must be at least 16 bytes. Exactly 16 bytes are written.
                             ) {
	enum StatusCode result;
	
	// Sanity check
	if (buffer == NULL || bufferSize < 16) {
		return STATUS_INVALID;
	}
	
	// Mifare Classic protocol requires two communications to perform a write.
	// Step 1: Tell the PICC we want to write to block blockAddr.
	uint8_t cmdBuffer[2];
	cmdBuffer[0] = PICC_CMD_MF_WRITE;
	cmdBuffer[1] = blockAddr;
	result = PCD_MIFARE_Transceive(cmdBuffer, 2); // Adds CRC_A and checks that the response is MF_ACK.
	if (result != STATUS_OK) {
		return result;
	}
	
	// Step 2: Transfer the data
	result = PCD_MIFARE_Transceive(buffer, bufferSize); // Adds CRC_A and checks that the response is MF_ACK.
	if (result != STATUS_OK) {
		return result;
	}
	
	return STATUS_OK;
} // End MIFARE_Write()

// Writes a 4 byte page to the active MIFARE Ultralight PICC.
enum StatusCode MIFARE_Ultralight_Write(uint8_t page, 		///< The page (2-15) to write to.
					uint8_t *buffer,	///< The 4 bytes to write to the PICC
					uint8_t bufferSize	///< Buffer size, must be at least 4 bytes. Exactly 4 bytes are written.
					) {
	enum StatusCode result;
	
	// Sanity check
	if (buffer == NULL || bufferSize < 4) {
		return STATUS_INVALID;
	}
	
	// Build commmand buffer
	uint8_t cmdBuffer[6];
	cmdBuffer[0] = PICC_CMD_UL_WRITE;
	cmdBuffer[1] = page;
	memcpy(&cmdBuffer[2], buffer, 4);
	
	// Perform the write
	result = PCD_MIFARE_Transceive(cmdBuffer, 6); // Adds CRC_A and checks that the response is MF_ACK.
	if (result != STATUS_OK) {
		return result;
	}
	return STATUS_OK;
} // End MIFARE_Ultralight_Write()

// MIFARE Decrement subtracts the delta from the value of the addressed block and stores the result in memory.

enum StatusCode MIFARE_Decrement(uint8_t blockAddr, ///< The block (0-0xff) number.
				 int32_t delta      ///< This number is subtracted from the value of block blockAddr.
											) {
	return MIFARE_TwoStepHelper(PICC_CMD_MF_DECREMENT, blockAddr, delta);
} // End MIFARE_Decrement()

// MIFARE Increment adds the delta to the value of the addressed block and sotres the result in a volatile memory
enum StatusCode MIFARE_Increment(uint8_t blockAddr, ///< The block (0-0xff) number.
				 uint32_t delta		///< This number is added to the value of block blockAddr.
				 ) {
	return MIFARE_TwoStepHelper(PICC_CMD_MF_INCREMENT, blockAddr, delta);
} // End MIFARE_Increment()

// MIFARE Restore copies the value of the addressed block into a volatile memory 
enum StatusCode MIFARE_Restore(uint8_t blockAddr ///< The block (0-0xff) number.
                              ) {
	// The datasheet describes Restore as a two step operation, but does not explain what data to transfer in step 2.
	// Doing only a single step does not work, so I chose to transfer 0L in step two.
	return MIFARE_TwoStepHelper(PICC_CMD_MF_RESTORE, blockAddr, 0L);
} // End MIFARE_Restore()

// Helper function for the two-step MIFARE Classic protocol operations Decrement, Increment, and Restore

enum StatusCode MIFARE_TwoStepHelper(uint8_t command,	///< The command to use
				     uint8_t blockAddr,	///< The block (0-0xff) number.
                                     uint32_t data	///< The data to transfer in step 2
                                    ) {
	enum StatusCode result;
	uint8_t cmdBuffer[2]; // We only need room for 2 bytes.
	
	// Step 1: Tell the PICC the command and block address
	cmdBuffer[0] = command;
	cmdBuffer[1] = blockAddr;
	result = PCD_MIFARE_Transceive(	cmdBuffer, 2); // Adds CRC_A and checks that the response is MF_ACK.
	if (result != STATUS_OK) {
		return result;
	}
	
	// Step 2: Transfer the data
	result = PCD_MIFARE_Transceive(	(uint8_t *)&data, 4, 1); // Adds CRC_A and accept timeout as success.
	if (result != STATUS_OK) {
		return result;
	}
	
	return STATUS_OK;
} // End MIFARE_TwoStepHelper()


// MIFARE Transfer writes the value stored in the volatile memory into one MIFARE Classic block.

enum StatusCode MIFARE_Transfer(uint8_t blockAddr ///< The block (0-0xff) number.
				) {
	enum StatusCode result;
	uint8_t cmdBuffer[2]; // We only need room for 2 bytes.
	
	// Tell the PICC we want to transfer the result into block blockAddr.
	cmdBuffer[0] = PICC_CMD_MF_TRANSFER;
	cmdBuffer[1] = blockAddr;
	result = PCD_MIFARE_Transceive(	cmdBuffer, 2); // Adds CRC_A and checks that the response is MF_ACK.
	if (result != STATUS_OK) {
		return result;
	}
	return STATUS_OK;
} // End MIFARE_Transfer()


// Dumps debug info about the connected PCD to Serial. 

void PCD_DumpVersionToSerial() {
	// Get the MFRC522 firmware version
	uint8_t v = PCD_ReadRegister(VersionReg);
	printf("Firmware Version: %x", v);
	// Lookup which version
	switch(v) {
		case 0x88: printf(" = (clone)");  break;
		case 0x90: printf(" = v0.0");     break;
		case 0x91: printf(" = v1.0");     break;
		case 0x92: printf(" = v2.0");     break;
		case 0x12: printf(" = counterfeit chip");     break;
		default:   printf(" = (unknown)");
	}
	// When 0x00 or 0xFF is returned, communication probably failed
	if ((v == 0x00) || (v == 0xFF))
		printf("\nWARNING: Communication failure, is the MFRC522 properly connected?");
} // End PCD_DumpVersionToSerial()



// Dumps debug info about the selected PICC to Serial
// On success the PICC is halted after dumping the data 
void PICC_DumpToSerial(Uid *uid	///< Pointer to Uid struct returned from a successful PICC_Select().
                      ) {
	MIFARE_Key key;
	
	// Dump UID, SAK and Type
	PICC_DumpDetailsToSerial(uid);
	
	// Dump contents
	enum PICC_Type piccType = PICC_GetType(uid->sak);
	switch (piccType) {
		case PICC_TYPE_MIFARE_MINI:
		case PICC_TYPE_MIFARE_1K:
		case PICC_TYPE_MIFARE_4K:
			// All keys are set to FFFFFFFFFFFFh at chip delivery from the factory.
			for (uint8_t i = 0; i < 6; i++) {
				key.keyByte[i] = 0xFF;
			}
			PICC_DumpMifareClassicToSerial(uid, piccType, &key);
			break;
			
		case PICC_TYPE_MIFARE_UL:
			PICC_DumpMifareUltralightToSerial();
			break;
			
		case PICC_TYPE_ISO_14443_4:
		case PICC_TYPE_MIFARE_DESFIRE:
		case PICC_TYPE_ISO_18092:
		case PICC_TYPE_MIFARE_PLUS:
		case PICC_TYPE_TNP3XXX:
			printf("Dumping memory contents not implemented for that PICC type.");
			break;
			
		case PICC_TYPE_UNKNOWN:
		case PICC_TYPE_NOT_COMPLETE:
		default:
			break; // No memory dump here
	}
	
	printf("");
	PICC_HaltA(); // Already done if it was a MIFARE Classic PICC.
} // End PICC_DumpToSerial()


// Dumps card info *UID, SAK, Type) about the selected picc to Serial
void PICC_DumpDetailsToSerial(Uid *uid	///< Pointer to Uid struct returned from a successful PICC_Select().
									) {
	// UID
	printf("Card UID:");
	for (uint8_t i = 0; i < uid->size; i++) {
		if(uid->uidByte[i] < 0x10)
                    printf(" 0");
		else
                    printf(" ");
		printf("%x", uid->uidByte[i]);
	} 
	printf("\n");
	
	// SAK
	printf("Card SAK: ");
	if(uid->sak < 0x10)
		printf("0");
	printf("%x", uid->sak);
	
	// (suggested) PICC type
	enum PICC_Type piccType = PICC_GetType(uid->sak);
	printf("PICC type: ");
	printf(PICC_GetTypeName(piccType));
} // End PICC_DumpDetailsToSerial()


// Dumps details of a MIFARE Classic toPICC
void PICC_DumpMifareClassicToSerial(Uid *uid,			///< Pointer to Uid struct returned from a successful PICC_Select().
                                    enum PICC_Type piccType,	///< One of the PICC_Type enums.
                                    MIFARE_Key *key		///< Key A used for all sectors.
											) {
	uint8_t no_of_sectors = 0;
	switch (piccType) {
		case PICC_TYPE_MIFARE_MINI:
			// Has 5 sectors * 4 blocks/sector * 16 bytes/block = 320 bytes.
			no_of_sectors = 5;
			break;
					
		case PICC_TYPE_MIFARE_1K:
			// Has 16 sectors * 4 blocks/sector * 16 bytes/block = 1024 bytes.
			no_of_sectors = 16;
			break;
			
		case PICC_TYPE_MIFARE_4K:
			// Has (32 sectors * 4 blocks/sector + 8 sectors * 16 blocks/sector) * 16 bytes/block = 4096 bytes.
			no_of_sectors = 40;
			break;
			
		default: // Should not happen. Ignore.
			break;
	}
	
	// Dump sectors, highest address first.
	if (no_of_sectors) {
		printf("Sector Block   0  1  2  3   4  5  6  7   8  9 10 11  12 13 14 15  AccessBits");
		for (int8_t i = no_of_sectors - 1; i >= 0; i--) {
			PICC_DumpMifareClassicSectorToSerial(uid, key, i);
		}
	}
	PICC_HaltA(); // Halt the PICC before stopping the encrypted session.
	PCD_StopCrypto1();
} // End PICC_DumpMifareClassicToSerial()



/////////////////////////////////////////////////////////////////////////////////////
// Convenience functions - does not add extra functionality
/////////////////////////////////////////////////////////////////////////////////////

// Returns true if a PICC responds to PICC_CMD_REQA
// Only "new" cards in state IDLE are invited. Sleeping cards in state HALT are ignored


uint8_t PICC_IsNewCardPresent(void) {
	uint8_t bufferATQA[2];
	uint8_t bufferSize = sizeof(bufferATQA);

	// Reset baud rates
	PCD_WriteRegister(TxModeReg, 0x00);
	PCD_WriteRegister(RxModeReg, 0x00);
	// Reset ModWidthReg
	PCD_WriteRegister(ModWidthReg, 0x26);

	enum StatusCode result = PICC_RequestA(bufferATQA, &bufferSize);
	return (result == STATUS_OK || result == STATUS_COLLISION);
} // End PICC_IsNewCardPresent()

// Wrapper around PICC_Select
// Returns true if a UID could be read 

uint8_t PICC_ReadCardSerial(void) {
	enum StatusCode result = PICC_Select(&uid);
	return (result == STATUS_OK);
} // End 



















