
// --------------------------------- VNX_LSW.h -------------------------------------------
//
//	Include file for LabBrick LSW RF Switch API
//
// (c) 2012-15 by Vaunix Corporation, all rights reserved
//
//	HME Version 1.0 based on RD Version 1.0 for Windows
//-----------------------------------------------------------------------------

#define VNX_MIN_DWELLTIME 1			// minimum dwell time at each step in a pattern
#define STATUS_RF_ON 0x8			// MASK: The RF HW is on

// Bit masks and equates for the switch pattern command byte (stored in Sweep_mode, and reported also in Status)									
#define SWPAT_REPEAT		0x02	// MASK: bit = 1 for continuous stepping though the pattern
#define SWPAT_ONCE			0x01	// MASK: bit = 1 for single pass through the pattern
#define SWPAT_END			0x00
 
// HID report equates
#define HR_BLOCKSIZE 6				// size of the block of bytes buffer in our HID report
#define HID_REPORT_LENGTH 8 		// use an 8 byte report..

/* not actually used, but they map out the data blocks */
typedef struct 
{
  char reportid;
  char status;
  char count;
  char byteblock[HR_BLOCKSIZE];
} HID_REPORT1;

typedef struct 
{
  char reportid;
  char command;
  char count;
  char byteblock[HR_BLOCKSIZE];
} HID_REPORT_OUT;

// Misc commands to send to the device
// For the input reports the first byte is the status, for the output it is the command. The high bit sets the 
// direction.
//
//	count is the number of valid bytes in the byteblock array
// 	byteblock is an array of bytes which make up the value of the command's argument or arguments.
//
// For result reports the command portion of the status byte is equal to the command sent if the command was successful.
// status byte layout:

// Bit0 - Bit5 = command result, equal to command if everything went well
// Bit6 = --reserved--
// Bit7 = --reserved--

// Pulse times use an Unsigned DWORD with the uppermost bits used for range selection
// Dwell time is a DWORD (unsigned) stored in normal Microsoft byte order.


// Misc commands to send to the device

#define VNX_SET				0x80
#define VNX_GET				0x00	// the set and get bits are or'd into the msb of the command byte


// ---------------------- Switch commands ------------------------
#define VNX_SW_SWITCH		0x54	// byte value with switch select value (0 to n)

#define VNX_SW_SWPAT		0x55	// switching pattern entry for the switch pattern array

#define VNX_SW_START		0x56	// set/get switching pattern start index

#define VNX_SW_PATSTART		0x57	// command to activate the pattern, may contain and set a start index in future versions

#define VNX_SW_EXTMOD		0x58	// command to select external control inputs
#define VNX_SWEEP			0x09	// used to get the pattern type

// ----------------- Pulse mode commands ------------------------
#define VNX_OFFTIME			0x49	// length of pulse mode RF off time in microseconds

#define VNX_ONTIME			0x4A	// length of pulse mode RF on time in microseconds

#define VNX_PULSE_MODE		0x4B	// start/stop pulse mode

#define VNX_RFMUTE			0x0A	// enable or disable RF output, byte = 01 to enable, 00 to disable
									// for the switches this really just enables and disables pulse mode
									// and is not particularly useful


#define VNX_SAVEPAR			0x0C	// command to save user parameters to flash, data bytes must be
									// set to 0x42, 0x55, 0x31 as a key to enable the flash update
									// all of the above settings are saved (RF Mute State, Attenuation, 
									// sweep parameters, etc.

#define VNX_GETSERNUM		0x1F	// get the serial number, value is a DWORD

#define VNX_MODELNAME		0x22	// get (no set allowed) the device's model name string -- last 6 chars only

//------------------------- Status Report ID Byte -------------------------------------
#define VNX_SW_STATUS		0x5E	// Not really a command, but the status byte value for periodic status reports.	

// ----------- Global Equates ------------
#define MAXDEVICES 64
#define MAX_MODELNAME 32
#define MAX_PATTERN_LENGTH 4
#define MAX_HOLD_TIME 100000000

// ----------- Data Types ----------------

#define DEVID unsigned int

typedef struct
{
	int	DevStatus;
	int ActiveSwitch;
	int	SwitchSetting;
	bool Use_External_SWControl;
	int NumSwitches;
	int	PatternType;
	int PatternLength;
	bool PulseMode;
	bool PulseEnabled;
	bool Use_External_Pulse;
	int PulseOnTime;
	int PulseOffTime;
	float PulseOnTimeSeconds;
	float PulseOffTimeSeconds;

	int SwitchPatternSelection[MAX_PATTERN_LENGTH];        //  the pattern action values for the switch pattern  
    int SwitchPatternHoldTime[MAX_PATTERN_LENGTH];         //  the pattern hold times
	int PatternIndex;									   //  the current entry in the pattern
	int Modebits;
	int SerialNumber;
	char ModelName[MAX_MODELNAME];
  
  /* so we can find this device again later */
	unsigned int idVendor;
	unsigned int idProduct;
	unsigned int idType;
	char Serialstr[16];
	char thread_command;
	char sndbuff[8];
	char rcvbuff[24];
	char decodewatch;
	int MyDevID;

} LSWPARAMS;

// ----------- Mode Bit Masks ------------

#define MODE_RFON 0x00000010 	// bit is 1 for RF on, 0 if RF is off (unused in RF switches)
#define MODE_INTREF 0x00000020 	// bit is 1 for internal osc., 0 for external reference (unused in RF switches)
#define MODE_SWEEP 0x0000000F 	// bottom 4 bits are used to keep the pattern control bits

#define MODE_PWMON 0x00000100 	// we keep a copy of the PWM control bits here, 1 for int PWM on
#define MODE_EXTPWM 0x00000200 	// 1 for ext. PWM input enabled
#define PWM_MASK 0x00000300

// ----------- Pattern Type Masks --------

#define SWPAT_REPEAT		0x02		// MASK: bit = 1 for repeating pattern operation
#define SWPAT_ONCE			0x01		// MASK: bit = 1 for single repetition of the pattern

// ----------- Command Equates -----------

// Status returns for commands
#define LVSTATUS int

#define STATUS_OK 0
#define BAD_PARAMETER 0x80010000		// out of range input -- argument outside min/max etc.
#define BAD_HID_IO    0x80020000		// low level I/O failure, usually a USB or driver problem
#define DEVICE_NOT_READY 0x80030000		// device isn't open, no handle, etc.
#define STATUS_ERROR 0x80000000			// mask to test if we got an error

#define F_INVALID_DEVID		-1.0		// for functions that return a float
#define F_DEVICE_NOT_READY	-3.0


// Status returns for DevStatus

#define INVALID_DEVID 0x80000000 		// MSB is set if the device ID is invalid
#define DEV_CONNECTED 0x00000001 		// LSB is set if a device is connected
#define DEV_OPENED 0x00000002 			// set if the device is opened
#define SWP_ACTIVE 0x00000004 			// set if the device is sweeping
#define SWP_UP 0x00000008 				// set if the device is sweeping up in frequency
#define SWP_REPEAT 0x00000010 			// set if the device is in continuous sweep mode
#define	FAST_PULSE_OPTION 0x00000080	// set if the fast pulse mode option is installed

// Internal values in DevStatus
#define DEV_LOCKED   0x00002000 		// set if we don't want read thread updates of the device parameters
#define DEV_RDTHREAD   0x00004000 		// set when the read thread is running

// Flags to encode pulse mode time ranges
#define PM48Mhz			0x10000000		// used to select the 48Mhz pulse mod clock
#define PM1Mhz			0x00000000		// used to select the 1Mhz pulse mod clock or sw pulsing

// ------------- API Functions -----------------
void fnLSW_SetTestMode(bool testmode);
int fnLSW_GetNumDevices();
int fnLSW_GetDevInfo(DEVID *ActiveDevices);
int fnLSW_GetModelName(DEVID deviceID, char *ModelName);
int fnLSW_InitDevice(DEVID deviceID);
int fnLSW_CloseDevice(DEVID deviceID);
int fnLSW_GetSerialNumber(DEVID deviceID);

int fnLSW_GetDeviceStatus(DEVID deviceID);

int fnLSW_GetNumSwitches(DEVID deviceID);
LVSTATUS fnLSW_SetSwitch(DEVID deviceID, int inputselect);
int fnLSW_GetActiveSwitch(DEVID deviceID);
int fnLSW_GetSwitchSetting(DEVID deviceID);
LVSTATUS fnLSW_SetUseExternalControl(DEVID deviceID, bool external);
int fnLSW_GetUseExternalControl(DEVID deviceID);

LVSTATUS fnLSW_SetPulseOnTime(DEVID deviceID, float pulseontime);
LVSTATUS fnLSW_SetPulseOffTime(DEVID deviceID, float pulseofftime);
LVSTATUS fnLSW_EnableInternalPulseMod(DEVID deviceID, bool on);

LVSTATUS fnLSW_SetFastPulsedOutput(DEVID deviceID, float pulseontime, float pulsereptime, bool on);

float fnLSW_GetPulseOnTime(DEVID deviceID);
float fnLSW_GetPulseOffTime(DEVID deviceID);
int fnLSW_GetPulseMode(DEVID deviceID);
int fnLSW_GetHasFastPulseMode(DEVID deviceID);

LVSTATUS fnLSW_SetPattern(DEVID deviceID, int num_entries, int sw_select[], int holdtime[]);
LVSTATUS fnLSW_SetPatternEntry(DEVID deviceID, int sw_select, int holdtime, int index, bool last_entry);
LVSTATUS fnLSW_StartPattern(DEVID deviceID, bool go);

LVSTATUS fnLSW_SetPatternType(DEVID deviceID, bool continuous );
int fnLSW_GetPatternEntrySwitch(DEVID deviceID, int index);
int fnLSW_GetPatternEntryTime(DEVID deviceID, int index);
int fnLSW_GetPatternType(DEVID deviceID);
int fnLSW_GetPatternLength(DEVID deviceID);

LVSTATUS fnLSW_SaveSettings(DEVID deviceID);
char* fnLSW_perror(LVSTATUS status);
char* fnLSW_LibVersion(void);
