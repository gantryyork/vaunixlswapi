#include <usb.h>
#include <linux/hid.h>	/* AK: Changed include for modern linux. */
#include <stdbool.h>	/* AK: Added include for 'bool' type */
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <pthread.h>
#include <time.h>
#include "LSWhid.h"

#define DEBUG_OUT 0  	/* set this to 1 in order to see debugging output, 2 for a ton of output, or 3 for many tons */
#define FALSE 0
#define TRUE !FALSE

#define PACKET_CTRL_LEN 8
#define PACKET_INT_LEN 8
#define INTERFACE 0
#define ENDPOINT_INT_IN 0x82
#define TIMEOUT 500
#define LIBVER "1.00"

void *brick_handler_function (void *ptr);

// ------ globals we'll be using at runtime ------------------

char errmsg[32]; 		// For the status->string converter

bool bVerbose = FALSE; 	// True to generate debug oriented printf output

bool TestMode = FALSE; 	// if TestMode is true we fake it -- no HW access
						// TestMode defaults to FALSE for production builds
						
bool first_status = TRUE;

LSWPARAMS lsw [MAXDEVICES]; // an array of structures each of which holds the info for a given 
							// device. The DeviceID is the index into the array. Devices may come and go
							// so there is no guarantee that the active elements are contiguous

time_t starttime, currtime;

// ------ variables for device identification ------
// product names
char sVNX1[32] = "LSW-502PDT";
char sVNX2[32] = "LSW-502P4T";
char sVNX3[32] = "LSW-602PDT";
char sVNX4[32] = "LSW-602P4T";
// device VID and PID
unsigned short devVID = 0x041f; 	// device VID for Vaunix Devices

unsigned short dev1PID = 0x1230; 	// device PID for Vaunix LSW-502PDT
unsigned short dev2PID = 0x1231; 	// device PID for Vaunix LSW-502P4T
unsigned short dev3PID = 0x1232; 	// device PID for Vaunix LSW-602PDT RD: added 8/2015
unsigned short dev4PID = 0x1233; 	// device PID for Vaunix LSW-602P4T RD: added 8/2015

/* stuff for the threads */
pthread_t threads[MAXDEVICES];
usb_dev_handle *thread_devhandles[MAXDEVICES];
#define THREAD_IDLE 0
#define THREAD_START 1
#define THREAD_EXIT 3
#define THREAD_DEAD 4
#define THREAD_ERROR -1

// --------------- Device IO support functions ----------------------------

bool CheckDeviceOpen(DEVID deviceID) {
  if (TestMode) return TRUE;	// in test mode all devices are always available

  if ((lsw[deviceID].DevStatus & DEV_OPENED) && (deviceID != 0))
    return TRUE;
  else
    return FALSE;
}

// ------------------------------------------------------------------------
bool DevNotLocked(DEVID deviceID) {
  if (TestMode) return TRUE;	// this shouldn't happen, but just in case...
  if (!(lsw[deviceID].DevStatus & DEV_LOCKED))
    return TRUE;				// we return TRUE if the device is not locked!
  else
    return FALSE;
}

// ------------------------------------------------------------------------
void LockDev(DEVID deviceID, bool lock) {
  if (TestMode) return;			// this shouldn't happen, but just in case...
  if (lock) {
    lsw[deviceID].DevStatus = lsw[deviceID].DevStatus | DEV_LOCKED;
    return;
  } else {
    lsw[deviceID].DevStatus = lsw[deviceID].DevStatus & ~DEV_LOCKED;
    return;
  }
}

/* A function to display the status as string */
char* fnLSW_perror(LVSTATUS status) {
  strcpy(errmsg, "STATUS_OK");
  if (BAD_PARAMETER == status) strcpy(errmsg, "BAD_PARAMETER");
  if (BAD_HID_IO == status) strcpy(errmsg, "BAD_HID_IO");
  if (DEVICE_NOT_READY == status) strcpy(errmsg, "DEVICE_NOT_READY");
  
  // Status returns for DevStatus
  if (INVALID_DEVID == status) strcpy(errmsg, "INVALID_DEVID");
  if (DEV_CONNECTED == status) strcpy(errmsg, "DEV_CONNECTED");
  if (DEV_OPENED == status) strcpy(errmsg, "DEV_OPENED");
  if (SWP_ACTIVE == status) strcpy(errmsg,  "SWP_ACTIVE");
  if (SWP_UP == status) strcpy(errmsg, "SWP_UP");
  if (SWP_REPEAT == status) strcpy(errmsg, "SWP_REPEAT");
  
  return errmsg;

}

char LibVersion[] = LIBVER;
char* fnLSW_LibVersion(void) {
  return LibVersion;
}
  
/* functions based on hid_io.cpp */
bool VNXOpenDevice(DEVID deviceID) {

  if (!(lsw[deviceID].DevStatus & DEV_CONNECTED))	// we can't open a device that isn't there!
    return DEVICE_NOT_READY;
  
  if (DEBUG_OUT > 1) printf("Starting thread %d\r\n", deviceID);
  lsw[deviceID].thread_command = THREAD_START; /* open device and start processing */
  pthread_create(&threads[deviceID], NULL, brick_handler_function, (void*)deviceID);
  lsw[deviceID].DevStatus = lsw[deviceID].DevStatus | DEV_OPENED;
  
  return STATUS_OK;
}

void report_data_decode(unsigned char rcvdata[], int tid) {
  int i;
  unsigned long dataval;
  char temp[32];

  if ((DEBUG_OUT > 1) && (rcvdata[0] != 0x4e)) {
    printf("Decoding ");
    for (i=0; i<8; i++)
      printf("%02x ", rcvdata[i]);
    printf("\r\n");
  }

  /* the first byte tells us the type, the second is the data length
     tid is the thread ID it came from so we can stash the value into lsw[] */
  /* first decode the bytes */
  dataval = 0;
  if (0 < rcvdata[1]) {
    for (i=0; i<rcvdata[1]; i++)
      dataval = (dataval<<8) + rcvdata[1+rcvdata[1]-i];
  }
  if ((DEBUG_OUT > 1) && (rcvdata[0] != VNX_SW_STATUS)) printf("Data payload decodes to %ld (%08x)\r\n", dataval, dataval);
  /* now we'll assign it to lsw[] */

  // handle the status report
  switch(rcvdata[0]) {
  case VNX_SW_STATUS:
    if (DEBUG_OUT > 0) printf("VNX_SW_STATUS: got one!\r\n");
    if (DevNotLocked(tid)) {
	  lsw[tid].ActiveSwitch = (int)rcvdata[3] + 1;		// update active switch value, which could be from external HW
      lsw[tid].PatternIndex = (int)rcvdata[4];			// PatternIndex = byteblock[2]
	  if (first_status){
		lsw[tid].SwitchSetting = (int)rcvdata[2] + 1;	// reporting the setting back from the HW is tricky, so we only
		first_status = FALSE;							// do it once at startup
	  }
      if (DEBUG_OUT > 0) printf("  VNX_SW_STATUS sez Active Switch=%d\r\n", lsw[tid].ActiveSwitch);

      if (rcvdata[6] & (SWPAT_ONCE | SWPAT_REPEAT))		// are we running a pattern?
		lsw[tid].DevStatus = lsw[tid].DevStatus | SWP_ACTIVE;
      else
		lsw[tid].DevStatus = lsw[tid].DevStatus & ~SWP_ACTIVE;

      // -- fill in the repeating pattern mode bit
      if (rcvdata[6] & (SWPAT_REPEAT))		// are we in a repeating pattern mode?
		lsw[tid].DevStatus = lsw[tid].DevStatus | SWP_REPEAT;
      else
		lsw[tid].DevStatus = lsw[tid].DevStatus & ~SWP_REPEAT;
      if (DEBUG_OUT > 0) printf("  VNX_SW_STATUS sez Status=%02x\r\n", lsw[tid].DevStatus);
      break;
    } /* if devnotlocked() */
    break;

  //  handle the various responses to GET commands
  case VNX_SW_SWITCH:
    if (DEBUG_OUT > 0) printf("Active Switch reported = %d, Switch Setting reported = %d\n", rcvdata[3], rcvdata[2]);
    if (DevNotLocked(tid))
		first_status = TRUE;
    break;

  case VNX_ONTIME:
    if (DEBUG_OUT > 0) printf(" Pulse On Time = %d\n", dataval);
    if (DevNotLocked(tid))
      lsw[tid].PulseOnTime = dataval;
    break;

  case VNX_OFFTIME:
    if (DEBUG_OUT > 0) printf(" Pulse Off Time = %d\n", dataval);
    if (DevNotLocked(tid))
      lsw[tid].PulseOffTime = dataval;
    break;
	
  case VNX_PULSE_MODE:
    if (DEBUG_OUT > 0) printf(" Pulse Mode = %d\n", rcvdata[2]);
    if (DevNotLocked(tid)) {
		if (rcvdata[2]& 0x80){			// the bit is true for disabled fast PWM
		  lsw[tid].DevStatus = lsw[tid].DevStatus & ~FAST_PULSE_OPTION;
		}
		else							// the option is present, enable its use
		{
			lsw[tid].DevStatus = lsw[tid].DevStatus | FAST_PULSE_OPTION;
		}
		lsw[tid].Modebits = lsw[tid].Modebits & ~PWM_MASK;	// clear the PWM bitfield
		dataval <<= 8;										// align the PWM bits with their bitfield
															// in the Modebits
		lsw[tid].Modebits = lsw[tid].Modebits | dataval;	// or in the PWM mode bits
	}
    break;

  case VNX_RFMUTE:
    if (DEBUG_OUT > 0) {
      if (rcvdata[2])
	strcpy(temp, "RF ON");
      else
	strcpy(temp, "RF OFF");
      printf("%s \n", temp);
    }

    if (DEBUG_OUT > 0) printf("Parsing a RFMUTE report..\n");

    if (DevNotLocked(tid)) {
      if (rcvdata[2])
	lsw[tid].Modebits = lsw[tid].Modebits | MODE_RFON;
      else
	lsw[tid].Modebits = lsw[tid].Modebits & ~MODE_RFON;
    }
    break;

  case VNX_SW_EXTMOD:
    if (DEBUG_OUT > 0) printf("Decoding an External Switch Mode Reply \n");
      if (DevNotLocked(tid)) {
	   if (rcvdata[2])
	   {
		lsw[tid].Use_External_Pulse = TRUE;
		lsw[tid].Use_External_SWControl = TRUE;
		lsw[tid].Modebits = lsw[tid].Modebits | MODE_EXTPWM;
	   }
	   else
	   {
		lsw[tid].Use_External_Pulse = FALSE;
		lsw[tid].Use_External_SWControl = FALSE;
		lsw[tid].Modebits = lsw[tid].Modebits & ~MODE_EXTPWM;
	   }
	}
    break;

  case VNX_SWEEP:
    if (DEBUG_OUT > 0) printf(" Pattern Type = %d\n", rcvdata[2] & MODE_SWEEP);
    if (DevNotLocked(tid)) {
      lsw[tid].Modebits = lsw[tid].Modebits & ~MODE_SWEEP;
	  lsw[tid].Modebits = lsw[tid].Modebits | (rcvdata[2] % MODE_SWEEP);
	  lsw[tid].PatternType = rcvdata[2] & MODE_SWEEP;
    }
    break;

  case VNX_SW_SWPAT:
    if (DEBUG_OUT > 0) printf(" Pattern Element %d Hold Time = %d\n", rcvdata[7], dataval);
    if (DevNotLocked(tid))
	  if(rcvdata[7] >= MAX_PATTERN_LENGTH) rcvdata[7] = MAX_PATTERN_LENGTH - 1;
	  lsw[tid].SwitchPatternSelection[rcvdata[7]] = (rcvdata[6] & 0x0F) + 1;
      lsw[tid].SwitchPatternHoldTime[rcvdata[7]] = dataval;
    break;

  case VNX_GETSERNUM:
    if (DEBUG_OUT > 0) printf(" Serial Number = %d\n", dataval);
    if (DevNotLocked(tid))
      lsw[tid].SerialNumber = dataval;		// NB -- we never use this path!
    break;
  } /* switch */

  return;
}

// ************* The read thread handler for the brick ***********************
void *brick_handler_function (void *threadID) {
  int i, tid;
  tid = (int)threadID;
  struct usb_bus *busses;
  struct usb_bus *bus;
  int usb_status;
  char fullpath[128];
  int retries;
 
  if (DEBUG_OUT > 0) printf("Starting thread for device %d\r\n", tid);
  while ((lsw[tid].thread_command >=0) &&
	 (lsw[tid].thread_command != THREAD_EXIT)) {
    switch(lsw[tid].thread_command) {
    case THREAD_IDLE: /* idle */
      /* this is where we wait for incoming USB data */
      usb_status = -1;
      retries = 50;
      while ((usb_status < 0) && (retries--) && (THREAD_IDLE == lsw[tid].thread_command)) {
	usb_status = usb_interrupt_read(thread_devhandles[tid],  // handle
					ENDPOINT_INT_IN, // endpoint
					lsw[tid].rcvbuff, // buffer
					PACKET_INT_LEN, // max length
					TIMEOUT);
	if (usb_status < 0) usleep(1000); /* wait 20 ms before trying again */
      }
      //      printf("Thread %d reports %d...\r\n", tid, usb_status);
      if (usb_status >= 0) {
	if (DEBUG_OUT > 1) {
	  printf("Thread %d reports %d...", tid, usb_status);
	  for (i=0; i<usb_status; i++)
	    printf("%02x ", lsw[tid].rcvbuff[i]);
	  printf("\r\n");
	}
	/* decode the HID data */
	report_data_decode(lsw[tid].rcvbuff, tid);
	if (DEBUG_OUT > 1) printf("Decoded device %d data %02x, decodewatch=%02x\r\n", tid, lsw[tid].rcvbuff[0], lsw[tid].decodewatch);
	if (lsw[tid].decodewatch == lsw[tid].rcvbuff[0]) {
	  if (DEBUG_OUT > 0) printf("Clearing decodewatch %02x for thread %d\r\n", lsw[tid].decodewatch, tid);
	  lsw[tid].decodewatch = 0;
	}
      } else
	if (DEBUG_OUT > 0) perror("THREAD_IDLE");
      break;
    case THREAD_START: /* starting up */
      /* we'll open the device. First we have to locate it */
      if (DEBUG_OUT > 0) printf("Thread %d is looking for the device\r\n", tid);

      usb_find_busses();
      usb_find_devices();
      busses = usb_get_busses();

      lsw[tid].thread_command = THREAD_ERROR; /* assume it will fail */
      for (bus = busses; bus; bus = bus->next) {
	if (THREAD_IDLE == lsw[tid].thread_command) break;
	struct usb_device *dev;

	for (dev = bus->devices; dev; dev = dev->next) {
	  if (THREAD_IDLE == lsw[tid].thread_command) break;
	  if (DEBUG_OUT > 1) printf("Thread %d sez- Vendor: %04x PID: %04x\r\n", tid, dev->descriptor.idVendor, dev->descriptor.idProduct);
	  thread_devhandles[tid] = usb_open(dev);
	  usb_status = usb_get_string_simple(thread_devhandles[tid], dev->descriptor.iSerialNumber, lsw[tid].rcvbuff, sizeof(lsw[tid].rcvbuff));
	  if (DEBUG_OUT > 1) printf("string %d = [%s] looking to match [%s]\r\n", dev->descriptor.iSerialNumber, lsw[tid].rcvbuff, lsw[tid].Serialstr);
	  if ((dev->descriptor.idVendor == lsw[tid].idVendor) &&
	      (dev->descriptor.idProduct == lsw[tid].idProduct) &&
	      (0 == strcmp(lsw[tid].rcvbuff, lsw[tid].Serialstr))) {
	    /* we found the device. We'll open it */
	    if (DEBUG_OUT > 1) printf("Opening file [%s]\r\n", dev->filename);
	    thread_devhandles[tid] = usb_open(dev);

	    usb_detach_kernel_driver_np(thread_devhandles[tid], 0);

	    usb_status = usb_set_configuration (thread_devhandles[tid], 1);
	    if (DEBUG_OUT > 1) printf ("set configuration: %s\n", usb_status ? "failed" : "passed");

	    usb_status = usb_claim_interface (thread_devhandles[tid], 0);
	    if (DEBUG_OUT > 1) printf ("claim interface: %s\n", usb_status ? "failed" : "passed");

	    lsw[tid].thread_command = THREAD_IDLE;
	    break;
	  } else {
	    /* if the device we opened isn't the one we wanted, close it */
	    usb_close(thread_devhandles[tid]);
	  }
	} /* for dev */
      } /* for bus */
      break;
    } /* switch */
  } /* while */
  if (DEBUG_OUT > 0) printf("Exiting thread for device %d because command=%d\r\n", tid, lsw[tid].thread_command);
  if (THREAD_EXIT == lsw[tid].thread_command)
    usb_close(thread_devhandles[tid]);
  lsw[tid].thread_command = THREAD_DEAD;
  pthread_exit(NULL);
}

// -------------- SendReport -------------------------------------------------

bool SendReport(int deviceID, char command, char *pBuffer, int cbBuffer)
{
  int i;
  int send_status;
  int retries;
  // Make sure the buffer that is being passed to us fits
  if (cbBuffer > HR_BLOCKSIZE) {
    // Report too big, don't send!
    return FALSE;
  }

  char Report[8];
  
  if (DEBUG_OUT > 1) printf("SR: command=%x cbBuffer=%x\r\n", command, cbBuffer);
  lsw[deviceID].sndbuff[0] = command;		// command to device
  lsw[deviceID].sndbuff[1] = cbBuffer;
  lsw[deviceID].sndbuff[2] = pBuffer[0];
  lsw[deviceID].sndbuff[3] = pBuffer[1];
  lsw[deviceID].sndbuff[4] = pBuffer[2];
  lsw[deviceID].sndbuff[5] = pBuffer[3];
  lsw[deviceID].sndbuff[6] = pBuffer[4];
  lsw[deviceID].sndbuff[7] = pBuffer[5];
  if (DEBUG_OUT > 1) {
    printf("SR: ");
    for (i=0; i<8; i++) {
      printf("%02x ", lsw[deviceID].sndbuff[i]);
    }
    printf("\r\n");
  }

  /* we have to wait for a file handle to appear */
  retries = 0;
  while ((0 == thread_devhandles[deviceID]) && (retries++ < 10))
    sleep(1);
  /* we have data to write to the device */
  if (DEBUG_OUT > 1) printf("SR: sending the write...\r\n");
  send_status = usb_control_msg(thread_devhandles[deviceID],
				0x21,
				0x09, //HID_REPORT_SET,
				0x200,
				0,
				lsw[deviceID].sndbuff,
				PACKET_CTRL_LEN,
				TIMEOUT);

  if (DEBUG_OUT > 1) {
    printf("(status=%d handle=%d)", send_status, thread_devhandles[deviceID]);
    if (send_status < 0) perror("SendReport"); else printf("\r\n");
  }

  return TRUE;
}

// ------------ GetParameter ---------------------------------------------
//
// The GetParam argument is the command byte sent to the device to get
// a particular value. The response is picked up by the read thread and
// parsed by it. The parser clears the corresponding event.


bool GetParameter(int deviceID, int GetParam)
{
	char VNX_param[6] = {0, 0, 0, 0, 0, 0};
	int timedout;

	if (DEBUG_OUT > 0) printf(" sending a GET command = %x\n", (char) GetParam );
	lsw[deviceID].decodewatch = (char) GetParam;
	if (!SendReport(deviceID, (char)GetParam, VNX_param, 0)) {
	  return FALSE;
	}

	if (DEBUG_OUT > 0) printf(" SendReport sent a GET command successfully to device %d = %x\n", deviceID, (char) GetParam );
	
	starttime = time(NULL);
	timedout = 0;

	/* wait until the value is decoded or 2 seconds have gone by */
	while ((lsw[deviceID].decodewatch > 0) && (0 == timedout)) {
	  if ((time(NULL)-starttime) > 2) timedout = 1;
	}

	return (0 == timedout) ? TRUE : FALSE;
}

// -------------- Get Routines to read device settings --------------------
//
// Note: for these functions deviceID is not checked for validity
//		 since it was already checked in the calling program.

bool GetSwitch(DEVID deviceID) {
  if (!GetParameter(deviceID, VNX_SW_SWITCH))
    return FALSE;

  return TRUE;
}

// -------------------------------

bool GetPatternType(DEVID deviceID) {
  if (!GetParameter(deviceID, VNX_SWEEP))
    return FALSE;

  return TRUE;
}

// -------------------------------

bool GetPatternStart(DEVID deviceID) {
  if (!GetParameter(deviceID, VNX_SW_START))
    return FALSE;

  return TRUE;
}

// -------------------------------
bool GetPatternEntry(DEVID deviceID, int index) {

   char VNX_param[6] = {0, 0, 0, 0, 0, 0};
   int timedout;

	if (DEBUG_OUT > 0) printf(" sending a GetPatternEntry command for index = %d\n", index);
	
	VNX_param[0] = (char) index;
	lsw[deviceID].decodewatch = (char) VNX_SW_SWPAT;
	if (!SendReport(deviceID, (char) VNX_SW_SWPAT, VNX_param, 1)) {
	  return FALSE;
	}

	if (DEBUG_OUT > 0) printf(" SendReport sent a GetPatternEntry command successfully to device %d\n", deviceID);
	
	starttime = time(NULL);
	timedout = 0;

	/* wait until the value is decoded or 2 seconds have gone by */
	while ((lsw[deviceID].decodewatch > 0) && (0 == timedout)) {
	  if ((time(NULL)-starttime) > 2) timedout = 1;
	}

	return (0 == timedout) ? TRUE : FALSE;



}

// -------------------------------

bool GetUseExtSwitchControl(DEVID deviceID) {	
  if (!GetParameter(deviceID, VNX_SW_EXTMOD))
    return FALSE;

  return TRUE;
}

// -------------------------------
bool GetPulseOnTime(DEVID deviceID) {
  if (!GetParameter(deviceID, VNX_ONTIME))
    return FALSE;

  return TRUE;
}

// -------------------------------
bool GetPulseOffTime(DEVID deviceID) {	
  // --- first we send the command out to the device --
  if (!GetParameter(deviceID, VNX_OFFTIME))
    return FALSE;
  return TRUE;
}

// -------------------------------
bool GetPulseMode(DEVID deviceID) {	
  // --- first we send the command out to the device --
  if (!GetParameter(deviceID, VNX_PULSE_MODE))
    return FALSE;
  return TRUE;
}

// -------------------------------
bool GetRF_On(DEVID deviceID) {
  if (!GetParameter(deviceID, VNX_RFMUTE))
    return FALSE;

  return TRUE;
}

/* functions to manage devices, not getting or retrieving data */
/*-------------------------------------------------------------*/

void FindVNXDevices()
{
  bool bFound;
  int hTemp;  			// temporary variable
  int HWType; 			// temporary variable for hardware/model type
  int HWNumSwitches;	// temporary variable for number of switches this model has
  char HWName[32];  	// temporary variable for the hardware model name
  char HWSerial[8]; 	// temporary holder for the serial number
  struct usb_dev_handle *devhandle;
  struct usb_bus *busses;
  char sendbuff[8];
  char rcvbuff[32];
  int usb_status;
    
  usb_init();
  if (DEBUG_OUT > 2)
    usb_set_debug(3); 	/* if we want lots of debug, let's see the USB output too. */
  else
    usb_set_debug(0);
  usb_find_busses();
  usb_find_devices();
  
  busses = usb_get_busses();
        
  struct usb_bus *bus;
  int c, i, a;
  int send_status, open_status;
  
  /* ... */
    
  // We need to remove devices from our table that are no longer connected ---
  // to do this we clear the "connected" flag for each table entry that is not open initially
  // then, as we find them we re-set the "connected" flag
  // anybody who doesn't have his "connected" flag set at the end is gone - we found it
  // previously but not this time
  
  for (i = 1; i<MAXDEVICES; i++){
    if ((lsw[i].SerialNumber != 0) && !(lsw[i].DevStatus & DEV_OPENED))
      lsw[i].DevStatus = lsw[i].DevStatus & ~DEV_CONNECTED; 	
  }

  for (bus = busses; bus; bus = bus->next) {
    struct usb_device *dev;
    
    for (dev = bus->devices; dev; dev = dev->next) {
      if (DEBUG_OUT > 1) printf("Vendor: %04x PID: %04x\r\n", dev->descriptor.idVendor, dev->descriptor.idProduct);
      HWType = 0;
	  
      /* check this device to see if it's one of our devices */
      if ((devVID == dev->descriptor.idVendor) &&
	  (dev1PID == dev->descriptor.idProduct)) HWType = 1;
      if ((devVID == dev->descriptor.idVendor) &&
	  (dev2PID == dev->descriptor.idProduct)) HWType = 2;
      if ((devVID == dev->descriptor.idVendor) &&
	  (dev3PID == dev->descriptor.idProduct)) HWType = 3;
	  if ((devVID == dev->descriptor.idVendor) &&
	  (dev4PID == dev->descriptor.idProduct)) HWType = 4;
	  
      if (HWType) { /* we like this device and we'll keep it */
	  
		if (DEBUG_OUT > 1) printf("Opening device %04x:%04x serial %04x type %d\r\n",
							dev->descriptor.idVendor,
							dev->descriptor.idProduct,
							dev->descriptor.iSerialNumber, HWType);
		devhandle = usb_open(dev);
		if (DEBUG_OUT > 1)  printf("LSW device found @ address [%s]\r\n", dev->filename);
		usb_status = usb_get_string_simple(devhandle, dev->descriptor.iSerialNumber, rcvbuff, sizeof(rcvbuff));
		if (DEBUG_OUT > 1) printf("string %d = [%s]\r\n", dev->descriptor.iSerialNumber, rcvbuff);
		if (usb_status < 0) strcpy(HWSerial, ""); else strcpy(HWSerial, rcvbuff+3);
		usb_close(devhandle);
	
		switch(HWType) {
		case 1:	 // LSW-502PDT
			strcpy(HWName, sVNX1);
			HWNumSwitches = 2;
			break;
		case 2:  // LSW-502P4T
			strcpy(HWName, sVNX2);
			HWNumSwitches = 4;
			break;
 		case 3:  // LSW-602PDT
			strcpy(HWName, sVNX3);
			HWNumSwitches = 2;
			break;
		case 4:  // LSW-602P4T
			strcpy(HWName, sVNX4);
			HWNumSwitches = 4;
			break;
		} /* HWType switch */
	
		/* find an open slot to save the data */
		// lets see if we have this unit in our table of devices already
		bFound = FALSE;
	
		for (i = 1; i<MAXDEVICES; i++){
			if (lsw[i].SerialNumber == atoi(HWSerial)) {
				// we already have the device in our table
				bFound = TRUE;
				lsw[i].DevStatus = lsw[i].DevStatus | DEV_CONNECTED; // its here, mark it as connected
				// at this point the device is present, but not in use, no sense looking more
				break;
			}
		}	// end of for loop
	
		// if the device isn't in the table we need to add it
		if (!bFound) {
			hTemp = 0;
			for (i = 1; i<MAXDEVICES; i++) {
				if (lsw[i].SerialNumber == 0) {
				hTemp = i;
				break;
				}
			} // end of for loop search for an empty slot in our array of devices
	  
			/* save all of the data we've already aquired */
			if (hTemp) {
				lsw[hTemp].SerialNumber = atoi(HWSerial);		            	// save the device's serial number
				lsw[hTemp].DevStatus = lsw[hTemp].DevStatus | DEV_CONNECTED;    // mark it as present
				strcpy (lsw[hTemp].ModelName, HWName);     		    			// save the device's model name

				lsw[hTemp].NumSwitches = HWNumSwitches;

				/* The device has been closed so let's make sure we can find it again */
				lsw[hTemp].idVendor = dev->descriptor.idVendor;
				lsw[hTemp].idProduct = dev->descriptor.idProduct;
				lsw[hTemp].idType = HWType;
				strcpy(lsw[hTemp].Serialstr, rcvbuff);
		
				if (DEBUG_OUT > 1) {
				printf("Stored as new device #%d\r\n", hTemp);
				printf("Serial number=%d\r\n", lsw[hTemp].SerialNumber);
				printf("Devstatus=%08x\r\n", lsw[hTemp].DevStatus);
				printf("Model name=%s\r\n", lsw[hTemp].ModelName);
				printf("NumSwitches=%d\r\n", lsw[hTemp].NumSwitches);

				printf("Vendor ID=%04x\r\n", lsw[hTemp].idVendor);
				printf("Product ID=%04x\r\n", lsw[hTemp].idProduct);
				printf("Serial number=%s\r\n", lsw[hTemp].Serialstr);
				}
			
			} else {
				// our table of devices is full, not much we can do
			}
		} /* if !bfound  */
		/* get any other data we might need */
      } /* if HWType */
    } /* for dev */
  } /* for bus */

  /* clean up the structure and mark unused slots */
  for (i = 1; i<MAXDEVICES; i++){
    if ((lsw[i].SerialNumber != 0) && !(lsw[i].DevStatus & DEV_CONNECTED))
      lsw[i].SerialNumber = 0;	// mark this slot as unused 	

    if (lsw[i].SerialNumber == 0)
      lsw[i].DevStatus = 0;		// clear the status for robustness!
  }	// end of zombie removal for loop
}

/* ----------------------------------------------------------------- */

void fnLSW_Init(void) {
  /* clear out the storage structure. Must be called once before anything else */
  int i;
  int status;

  for (i = 0; i<MAXDEVICES; i++){
    lsw[i].DevStatus = 0; 		// init to no devices connected
    lsw[i].SerialNumber = 0; 	// clear the serial number
    lsw[i].ModelName[0] = 0; 	// put a null string in each model name field
  }

  usb_init();
  if (DEBUG_OUT > 0)  printf("library version %s\r\n", fnLSW_LibVersion());
}

void fnLSW_SetTestMode(bool testmode) {
  TestMode = testmode;
}

int fnLSW_GetNumDevices() {
  int retval = 0;
  int NumDevices = 0;
  int i;
  
  // See how many devices we can find, or have found before
  if (TestMode){
    // construct a fake device

    lsw[1].SerialNumber = 55502;
    lsw[1].DevStatus = lsw[1].DevStatus | DEV_CONNECTED;
	lsw[1].idType = 1;
    lsw[1].NumSwitches = 2;
    strcpy (lsw[1].ModelName, "LSW-502PDT");

    // construct a second fake device
    lsw[2].SerialNumber = 55504;
    lsw[2].DevStatus = lsw[2].DevStatus | DEV_CONNECTED;
	lsw[2].idType = 2;
	lsw[2].NumSwitches = 4;
    strcpy (lsw[2].ModelName, "LSW-502P4T");

    retval = 2;
    
  } else {
    // go look for some real hardware
    FindVNXDevices();

    // Total up the number of devices we have
    for (i = 0; i < MAXDEVICES; i++){
      if (lsw[i].DevStatus & DEV_CONNECTED) NumDevices++; 
    }
    retval = NumDevices;

  }
  return retval;
}

int fnLSW_GetDevInfo(DEVID *ActiveDevices) {
  int i;
  int NumDevices = 0;
  
  if ( ActiveDevices == NULL) return 0; // bad array pointer, no place to put the DEVIDs
  
  for (i = 1; i < MAXDEVICES; i++){ 	// NB -- we never put an active device in lsw[0] - so DEVIDs start at 1
    if (lsw[i].DevStatus & DEV_CONNECTED) {
      ActiveDevices[NumDevices] = i;
      NumDevices++;
    }
  }
  
  return NumDevices;
}

 int fnLSW_GetModelName(DEVID deviceID, char *ModelName) {
   int NumChars = 0;
   
   if (deviceID >= MAXDEVICES){
     return 0;
   }
   
   NumChars = strlen(lsw[deviceID].ModelName);
   // If NULL result pointer, just return the number of chars in the name
   if ( ModelName == NULL) return NumChars;
   strcpy(ModelName, lsw[deviceID].ModelName);
   
   return NumChars;
 }
 
 int fnLSW_InitDevice(DEVID deviceID) {
   int i;
 
   if ((deviceID >= MAXDEVICES) || (deviceID == 0)) {
     return INVALID_DEVID;
   }
   
   if (TestMode)
     lsw[deviceID].DevStatus = lsw[deviceID].DevStatus | DEV_OPENED;
   else {
     // Go ahead and open a handle to the hardware
     if (VNXOpenDevice(deviceID))//VNXOpenDevice returns 0 if the open succeeded
       return DEVICE_NOT_READY;
     if (DEBUG_OUT > 0) printf("Time to start getting parameters from device %d\r\n", deviceID);
     
     // Get the rest of the device parameters from the device
     
     if (!GetSwitch(deviceID)) {
       return BAD_HID_IO;
     }
     
     if (!GetUseExtSwitchControl(deviceID)) {
       return BAD_HID_IO;
     }
     
     if (!GetPulseOffTime(deviceID)) {
       return BAD_HID_IO;
     }
     
     if (!GetPulseOnTime(deviceID)) {
       return BAD_HID_IO;
     }
     
     if (!GetPulseMode(deviceID)) {
       return BAD_HID_IO;
     }
     
     if (DEBUG_OUT > 0) printf ("PulseOnTime = 0x%x, PulseOffTime = 0x%x\n", lsw[deviceID].PulseOnTime, lsw[deviceID].PulseOffTime); 
     
     // --- calculate the pulse on and off times in seconds from what we read ---
     
     if ((lsw[deviceID].PulseOnTime & 0xF0000000) == 0x10000000) {
       lsw[deviceID].PulseOnTimeSeconds = ((float) ((lsw[deviceID].PulseOnTime & 0x0FFFFFFF)) * 2.083333e-8);
       lsw[deviceID].PulseOffTimeSeconds = ((float) ((lsw[deviceID].PulseOffTime & 0x0FFFFFFF)) * 2.083333e-8);
     }
     else {
       lsw[deviceID].PulseOnTimeSeconds = ((float) ((lsw[deviceID].PulseOnTime & 0x0FFFFFFF)) * 1.00000e-6);
       lsw[deviceID].PulseOffTimeSeconds = ((float) ((lsw[deviceID].PulseOffTime & 0x0FFFFFFF)) * 1.00000e-6);
     }
     
     // get the parameters for the switch patterns
     if (!GetPatternType(deviceID)) {
       return BAD_HID_IO;
     }
     
     // now lets populate the switch pattern from the device's saved switch pattern
     for (i = 0; i < MAX_PATTERN_LENGTH; i++) {        // get the saved switch pattern from the device
       if (!GetPatternEntry(deviceID, i)) {
	 return BAD_HID_IO;
       }
     }
     
     // and finally we figure out the number of entries in the pattern
     lsw[deviceID].PatternLength = 0;
     
     for (i = 0; i < MAX_PATTERN_LENGTH; i++) {        // look for the end of pattern marker
       if (lsw[deviceID].SwitchPatternSelection[i] != SWPAT_END) lsw[deviceID].PatternLength++;
     }
     
   } // end of real device open process case
   
   // if we got here everything worked OK
   return STATUS_OK;
 }

int fnLSW_CloseDevice(DEVID deviceID) {
  
  if (deviceID >= MAXDEVICES)
    return INVALID_DEVID;
  
  if (!CheckDeviceOpen(deviceID))
    return DEVICE_NOT_READY;

  if (TestMode)
    lsw[deviceID].DevStatus = lsw[deviceID].DevStatus & ~DEV_OPENED;
  else {

    // Go ahead and close this hardware - the first step is to stop its read thread
    lsw[deviceID].thread_command = THREAD_EXIT;
    
    // The thread handler will close the device. We'll wait up to 1 second then give up.
    int retries;
    retries = 10;
    while (retries && (lsw[deviceID].thread_command != THREAD_DEAD)) {
      usleep(100000); /* wait 100 ms */
      retries--;
    }
    if (DEBUG_OUT > 0) printf("After telling the thread to close, we have thread_command=%d retries=%d\r\n", lsw[deviceID].thread_command, retries);
    lsw[deviceID].thread_command = THREAD_IDLE;

    // Mark it closed in our list of devices
    lsw[deviceID].DevStatus = lsw[deviceID].DevStatus & ~DEV_OPENED;
  }

  return STATUS_OK;

}

int fnLSW_GetSerialNumber(DEVID deviceID) {
  if (deviceID >= MAXDEVICES)
    return 0;
  
  return lsw[deviceID].SerialNumber;
}


// Functions to set parameters

// Select the switch setting, switch numbers 1 -> N

LVSTATUS fnLSW_SetSwitch(DEVID deviceID, int inputselect) {
	if (deviceID >= MAXDEVICES)
	  return INVALID_DEVID;

	if (!CheckDeviceOpen(deviceID))
	  return DEVICE_NOT_READY;

	LockDev(deviceID, TRUE);

	int old_switchsetting = lsw[deviceID].SwitchSetting;
	
	if ((inputselect > 0) && (inputselect <= lsw[deviceID].NumSwitches)) {
	  lsw[deviceID].SwitchSetting = inputselect;
	  
	  if (TestMode){
	    LockDev(deviceID, FALSE);
	    return STATUS_OK;		// in test mode we update our internal variables, but don't talk to the real HW
	  }
	} else {
	  LockDev(deviceID, FALSE);
	  return BAD_PARAMETER;
	}

	// the switch value is OK, lets send it to the hardware
	int sw_select = inputselect - 1;	// at the HW level switch selection is zero based
	unsigned char *ptr = (unsigned char *) &sw_select;
	

	if (!SendReport(deviceID, VNX_SW_SWITCH | VNX_SET, ptr, 4)) {
	  lsw[deviceID].SwitchSetting = old_switchsetting;
	  LockDev(deviceID, FALSE);
	  return BAD_HID_IO;
	}

	LockDev(deviceID, FALSE);
	return STATUS_OK;
}


// Select internal or external control of the switches, true for external control

LVSTATUS fnLSW_SetUseExternalControl(DEVID deviceID, bool external) {
	if (deviceID >= MAXDEVICES)
	  return INVALID_DEVID;

	if (!CheckDeviceOpen(deviceID))
	  return DEVICE_NOT_READY;
	  
	unsigned char VNX_command[6] = {0, 0, 0, 0, 0, 0};

	if (external){
		
		lsw[deviceID].Use_External_Pulse = TRUE;
		lsw[deviceID].Use_External_SWControl = TRUE;
		lsw[deviceID].Modebits = lsw[deviceID].Modebits | MODE_EXTPWM;
		VNX_command[0] = 1;
	}
	else {
		lsw[deviceID].Use_External_Pulse = FALSE;
		lsw[deviceID].Use_External_SWControl = FALSE;
		lsw[deviceID].Modebits = lsw[deviceID].Modebits & ~MODE_EXTPWM;
		VNX_command[0] = 0;
	}

	if (TestMode){
		return STATUS_OK;		// in test mode we update our internal variables, but don't talk to the real HW
	}

	if (!SendReport(deviceID, VNX_SW_EXTMOD | VNX_SET, VNX_command, 1)){
		return BAD_HID_IO;
	}

	return STATUS_OK;
}

// ---------------- Functions related to the switch patterns --------------------------

LVSTATUS fnLSW_SetPattern(DEVID deviceID, int num_entries, int sw_select[], int holdtime[]) {
	int i;
	if (deviceID >= MAXDEVICES)
	  return INVALID_DEVID;

	if (!CheckDeviceOpen(deviceID))
	  return DEVICE_NOT_READY;

	if (num_entries > MAX_PATTERN_LENGTH || num_entries < 1)
		return BAD_PARAMETER;
	
	if (TestMode) {
		lsw[deviceID].PatternLength = num_entries;
		for (i = 0; i < num_entries; i++) {
			lsw[deviceID].SwitchPatternSelection[i] = sw_select[i];
			lsw[deviceID].SwitchPatternHoldTime[i] = holdtime[i];
		}
		return STATUS_OK;		// in test mode we update our internal variables, but don't talk to the real HW
	}

	// lets send the pattern to the hardware
	unsigned char VNX_command[6] = {0, 0, 0, 0, 0, 0};

	for (i = 0; i < num_entries; i++) {
		VNX_command[0] = (char) (holdtime[i] & 0x000000FF);
		VNX_command[1] = (char) ((holdtime[i] >> 8) & 0x000000FF);
		VNX_command[2] = (char) ((holdtime[i] >> 16) & 0x000000FF);
		VNX_command[3] = (char) ((holdtime[i] >> 24) & 0x000000FF);
		VNX_command[4] = (char) ((sw_select[i] - 1) | 0x10);
		VNX_command[5] = (char) (i);

	if (DEBUG_OUT > 1) printf("Sending pattern element %d\n", i);

		if (!SendReport(deviceID, VNX_SW_SWPAT | VNX_SET, VNX_command, 6)){
			return BAD_HID_IO;
		}
	}
	
	// now we send the end of pattern marker to the Lab Brick
		VNX_command[0] = 1;
		VNX_command[1] = 0;
		VNX_command[2] = 0;
		VNX_command[3] = 0;
		VNX_command[4] = SWPAT_END;
		VNX_command[5] = (char) (num_entries);		// since our index is 0 based, num_entries points to the slot after
													// our set of pattern entries
		// put in the pattern end marker for the last entry
		if (!SendReport(deviceID, VNX_SW_SWPAT | VNX_SET, VNX_command, 6)){
			return BAD_HID_IO;
		}

	// if we got this far we succeeded in updating the hardware, so we should update our record of the pattern
	lsw[deviceID].PatternLength = num_entries;

	if (DEBUG_OUT > 2) printf("Setting Pattern Length = %d\n", num_entries);	

	for (i = 0; i < num_entries; i++) {
		lsw[deviceID].SwitchPatternSelection[i] = sw_select[i];
		lsw[deviceID].SwitchPatternHoldTime[i] = holdtime[i];
	}

	return STATUS_OK;
}

// --------------- Set a single entry ----------------

LVSTATUS fnLSW_SetPatternEntry(DEVID deviceID, int sw_select, int holdtime, int index, bool last_entry) {
	if (deviceID >= MAXDEVICES)
	  return INVALID_DEVID;

	if (!CheckDeviceOpen(deviceID))
	  return DEVICE_NOT_READY;
	  
	if (index > (MAX_PATTERN_LENGTH - 1) || index < 0) {			// the index is 0 based 
		return BAD_PARAMETER;
	}

	if (sw_select > lsw[deviceID].NumSwitches || sw_select < 1) {	// the selector for switches is 1 based
		return BAD_PARAMETER;
	}

	if (holdtime > MAX_HOLD_TIME || holdtime < 1) {	// the time at each step must be in range
		return BAD_PARAMETER;
	}

	if (TestMode) {
		lsw[deviceID].SwitchPatternSelection[index] = sw_select;
		lsw[deviceID].SwitchPatternHoldTime[index] = holdtime;

		if (last_entry) lsw[deviceID].PatternLength = index + 1;

		return STATUS_OK;		// in test mode we update our internal variables, but don't talk to the real HW
	}


	// lets send the pattern entry to the hardware
	unsigned char VNX_command[6] = {0, 0, 0, 0, 0, 0};

	VNX_command[0] = (char) (holdtime & 0x000000FF);
	VNX_command[1] = (char) ((holdtime >> 8) & 0x000000FF);
	VNX_command[2] = (char) ((holdtime >> 16) & 0x000000FF);
	VNX_command[3] = (char) ((holdtime >> 24) & 0x000000FF);
	VNX_command[4] = (char) ((sw_select - 1) | 0x10);
	VNX_command[5] = (char) (index);

	if (DEBUG_OUT > 2) printf("Sending a pattern element setting command\n");

	
	if (!SendReport(deviceID, VNX_SW_SWPAT | VNX_SET, VNX_command, 6)){
		return BAD_HID_IO;
	}

	if (last_entry)
	{
		VNX_command[0] = 1;
		VNX_command[1] = 0;
		VNX_command[2] = 0;
		VNX_command[3] = 0;
		VNX_command[4] = SWPAT_END;
		VNX_command[5] = (char) (index + 1);
		// put in the pattern end marker if we are setting the last entry
		if (!SendReport(deviceID, VNX_SW_SWPAT | VNX_SET, VNX_command, 6)){
		return BAD_HID_IO;
		}

	}

	// if we got this far we succeeded in updating the hardware, so we should update our record of the pattern
	lsw[deviceID].SwitchPatternSelection[index] = sw_select;
	lsw[deviceID].SwitchPatternHoldTime[index] = holdtime;

	if (last_entry) lsw[deviceID].PatternLength = index + 1;

	return STATUS_OK;
}

// Start a switch pattern

LVSTATUS fnLSW_StartPattern(DEVID deviceID, bool go) {
	if (deviceID >= MAXDEVICES)
	  return INVALID_DEVID;

	if (!CheckDeviceOpen(deviceID))
	  return DEVICE_NOT_READY;
	
	unsigned char VNX_command[6] = {0, 0, 0, 0, 0, 0};

	if (go){
		VNX_command[0] = (char) lsw[deviceID].Modebits & MODE_SWEEP;
	}
	else {
		VNX_command[0] = 0;
	}

	if (TestMode){
		return STATUS_OK;		// in test mode we update our internal variables, but don't talk to the real HW
	}

	if (DEBUG_OUT > 1) printf(" Sending a start pattern command = %x\n", VNX_command[0] );

	if (!SendReport(deviceID, VNX_SW_PATSTART | VNX_SET, VNX_command, 1)){
		return BAD_HID_IO;
	}

	return STATUS_OK;
}

// Set the time to wait at the end of the ramp

LVSTATUS fnLSW_SetPatternType(DEVID deviceID, bool continuous ) {
	if (deviceID >= MAXDEVICES)
	  return INVALID_DEVID;

	if (!CheckDeviceOpen(deviceID))
	  return DEVICE_NOT_READY;

	if (continuous)
	{
		lsw[deviceID].Modebits = lsw[deviceID].Modebits | SWPAT_REPEAT;		// Repeated pattern
		lsw[deviceID].Modebits = lsw[deviceID].Modebits & ~SWPAT_ONCE;
		lsw[deviceID].PatternType = SWPAT_REPEAT;
	}
	else
	{
		lsw[deviceID].Modebits = lsw[deviceID].Modebits | SWPAT_ONCE;		// one time pattern
		lsw[deviceID].Modebits = lsw[deviceID].Modebits & ~SWPAT_REPEAT;
		lsw[deviceID].PatternType = SWPAT_ONCE;
	}

	return STATUS_OK;
}

// --------------------- functions related to pulsed switching --------------------------


LVSTATUS fnLSW_EnableInternalPulseMod(DEVID deviceID, bool on) {
	if (deviceID >= MAXDEVICES)
		return INVALID_DEVID;

	if (!CheckDeviceOpen(deviceID))
		return DEVICE_NOT_READY;

	unsigned char VNX_command[6] = {0, 0, 0, 0, 0, 0};

	if (on){
		lsw[deviceID].Modebits = lsw[deviceID].Modebits | MODE_PWMON;
	}
	else {
		lsw[deviceID].Modebits = lsw[deviceID].Modebits & ~MODE_PWMON;
	}

	VNX_command[0] = (lsw[deviceID].Modebits & PWM_MASK) >> 8;

	if (TestMode){
		return STATUS_OK;		// in test mode we update our internal variables, but don't talk to the real HW
	}

	if (!SendReport(deviceID, VNX_PULSE_MODE | VNX_SET, VNX_command, 1)){
		return BAD_HID_IO;
	}

	return STATUS_OK;
}

// This function takes two float arguments - pulse on time in seconds and pulse rep time in seconds

LVSTATUS fnLSW_SetFastPulsedOutput(DEVID deviceID, float pulseontime, float pulsereptime, bool on) {
	int temp_pulseontime;
	int temp_pulseofftime;
	unsigned char VNX_command[6] = {0, 0, 0, 0, 0, 0};

	if (deviceID >= MAXDEVICES)
		return INVALID_DEVID;

	if (!CheckDeviceOpen(deviceID))
		return DEVICE_NOT_READY;

	float old_pulseontime = lsw[deviceID].PulseOnTimeSeconds;
	float old_pulseofftime = lsw[deviceID].PulseOffTimeSeconds;

	if (pulsereptime <= pulseontime) {		// the on time has to be less than the repetition time
		return BAD_PARAMETER;
	}

	// ------ first we have to convert from the floating point times to our integers ------
	// 
	// we have to pick the range of units depending on whether or not we can use the 48MHz, 1 MHz or 1Ms clocks

	if (DEBUG_OUT > 2)  printf ("pulseontime = %f, pulsereptime = %f\n", pulseontime, pulsereptime);



	if (fabs(pulsereptime - .001) <= .00001) {		// use the 48MHz clock
		if ((pulseontime + 5.0e-9 > pulsereptime)){	// the on time plus one clock has to be less than the repetition time
		return BAD_PARAMETER;
		}

		// generate the integer on time

		if (DEBUG_OUT > 2)  printf ("using 48MHz clock\n");

		temp_pulseontime = (int) (48 * (pulseontime * 1.0e+6));
		temp_pulseofftime = (int) ((48 * (pulsereptime * 1.0e+6)) - temp_pulseontime);
		temp_pulseontime |= PM48Mhz;

		if (DEBUG_OUT > 2)  printf ("temp_pulseofftime = %d", temp_pulseofftime);

		if (temp_pulseofftime <= 0) return BAD_PARAMETER;

		//	temp_pulseofftime |= PM48Mhz; We don't mark the range in the off time!
		if (DEBUG_OUT > 2)  printf ("temp_pulseofftime = %d", temp_pulseofftime);


	}
	else if (pulsereptime > .001 && pulsereptime <= .050) {	// use the 1MHz clock
		if ((pulseontime + 1.0e-6 > pulsereptime)){			// the on time plus one clock has to be less than the repetition time
		return BAD_PARAMETER;
		}

		if (DEBUG_OUT > 1)  printf ("using 1MHz clock\n");
		temp_pulseontime = (int) (pulseontime * 1.0e+6);
		temp_pulseofftime = (int) (pulsereptime * 1.0e+6) - temp_pulseontime;
		if (temp_pulseofftime <= 0) return BAD_PARAMETER;

	}
	else {											// for rep time > 50 ms we use the software timer (its really the same as the above case)
		if ((pulseontime + .001 > pulsereptime)){	// the on time plus one clock has to be less than the repetition time
		return BAD_PARAMETER;
		}

		if (pulsereptime > 1000) return BAD_PARAMETER;	// maximum time is 1000 seconds

	if (DEBUG_OUT > 2) printf ("using 1 ms. timer\n");

		// ---- we represent the time in 1 microsecond units --------
		temp_pulseontime = (int) (pulseontime * 1.0e+6);
		temp_pulseofftime = (int) (pulsereptime * 1.0e+6) - temp_pulseontime;

		if (temp_pulseofftime <= 0) return BAD_PARAMETER;
	}

	// At this point we can update our local copies of the on and off time in seconds
	// We'll restore the old values if somehow the hardware I/O fails
		lsw[deviceID].PulseOnTimeSeconds = pulseontime; 
		lsw[deviceID].PulseOffTimeSeconds = pulsereptime - pulseontime;

	if (DEBUG_OUT > 2)  printf ("PulseOnTimeSeconds = %f, PulseOffTimeSeconds = %f\n", lsw[deviceID].PulseOnTimeSeconds, lsw[deviceID].PulseOffTimeSeconds); 

	// Now send the parameters to the device if we aren't in test mode

	if (TestMode) {
		if (on){				// keep the mode bits in sync
			lsw[deviceID].Modebits = lsw[deviceID].Modebits | MODE_PWMON;
		}
		else {
			lsw[deviceID].Modebits = lsw[deviceID].Modebits & ~MODE_PWMON;
		}

		return STATUS_OK;		// in test mode we update our internal variables, but don't talk to the real HW
	}


	// First we disable any active fast pulse mode operation, including external mode ...

	VNX_command[0] = 0;			// pulse mode off, internal pulse modulation selected

	if (!SendReport(deviceID, VNX_PULSE_MODE | VNX_SET, VNX_command, 1)){
		// -- our IO failed, so leave the old settings and bail out --
		//	  we can't do much about restoring the HW pulse mode state...
		lsw[deviceID].PulseOnTimeSeconds = old_pulseontime; 
		lsw[deviceID].PulseOffTimeSeconds = old_pulseofftime;

		return BAD_HID_IO;
	}

	// Then we send the on time, followed by the off time

	unsigned char *ptr = (unsigned char *) &temp_pulseontime;

	//	if (DEBUG_OUT > 2)  printf("deviceID = 0x%x ptr = 0x%x Pulse On Time LSByte = 0x%x PulseOnTime = %d\n", lsw[deviceID].hDevice, ptr, *ptr, temp_pulseontime);
	if (DEBUG_OUT > 2)  printf("deviceID = 0x%x ptr = 0x%x Pulse On Time LSByte = 0x%x PulseOnTime = %d\n", deviceID, ptr, *ptr, temp_pulseontime);

	
	if (!SendReport(deviceID, VNX_ONTIME | VNX_SET, ptr, 4)){
		// -- our IO failed, so leave the old settings and bail out --
		lsw[deviceID].PulseOnTimeSeconds = old_pulseontime; 
		lsw[deviceID].PulseOffTimeSeconds = old_pulseofftime;
		return BAD_HID_IO;
	}

	ptr = (unsigned char *) &temp_pulseofftime;

	//	if (DEBUG_OUT > 2)  printf("deviceID = 0x%x ptr = 0x%x Pulse Off Time LSByte = 0x%x PulseOffTime = %d\n", lsw[deviceID].hDevice, ptr, *ptr, temp_pulseofftime);
	if (DEBUG_OUT > 2)  printf("deviceID = 0x%x ptr = 0x%x Pulse Off Time LSByte = 0x%x PulseOffTime = %d\n", deviceID, ptr, *ptr, temp_pulseofftime);

	
	if (!SendReport(deviceID, VNX_OFFTIME | VNX_SET, ptr, 4)){
		// -- we're in a pickle here, we set the new pulse on time, but failed on the new off time setting
		//    so our state variables may not be viable value wise, but since talking to the device is failing
		//    we really can't do much about it!

		lsw[deviceID].PulseOffTimeSeconds = old_pulseofftime;	
		return BAD_HID_IO;
	}

	// -- time to activate the pulse mode operation with the new settings --

	if (on){
		lsw[deviceID].Modebits = lsw[deviceID].Modebits | MODE_PWMON;
	}
	else {
		lsw[deviceID].Modebits = lsw[deviceID].Modebits & ~MODE_PWMON;
	}

	VNX_command[0] = (lsw[deviceID].Modebits & PWM_MASK) >> 8;

	if (!SendReport(deviceID, VNX_PULSE_MODE | VNX_SET, VNX_command, 1)){

		// -- a failure here leaves the settings intact, and in sync, except for the mode bits
		//    probably not worth worrying about trying to restore them since who knows what is
		//    going on below us to cause the failure...

		return BAD_HID_IO;
	}

	if (DEBUG_OUT > 2)  printf("Set Pulse Mode to: 0x%x\n", VNX_command[0]); 

	return STATUS_OK;		// it seems everything turned out OK!
}


// Set the internal pulse modulation pulse on time
// This code converts from the floating point on time value (in seconds) to the ranged integer units
// it does not check for required relationship with off time.

int fnLSW_SetPulseOnTime(DEVID deviceID, float pulseontime) {
	int temp_pulseontime;

	if (deviceID >= MAXDEVICES)
		return INVALID_DEVID;

	if (!CheckDeviceOpen(deviceID))
		return DEVICE_NOT_READY;

	float old_pulseontime = lsw[deviceID].PulseOnTimeSeconds;

	if (pulseontime >= 0.10e-6 && pulseontime < 1000){

		lsw[deviceID].PulseOnTimeSeconds = pulseontime;
		if (TestMode){
			return STATUS_OK;		// in test mode we update our internal variables, but don't talk to the real HW
		}
	}
	else {
		return BAD_PARAMETER;		// we end up here if the on time is less than 100ns
	}


	if (pulseontime <= .001) {		// use the 48MHz clock
		// generate the integer on time
		temp_pulseontime = (int) (48 * (pulseontime * 1.0e+6));
		temp_pulseontime |= PM48Mhz;
		
	} else {
		temp_pulseontime = (int) (pulseontime * 1.0e+6);
	}

	// the pulse on time value is OK, lets send it to the hardware
	unsigned char *ptr = (unsigned char *) &temp_pulseontime;

	if (DEBUG_OUT > 2) printf("deviceID = %x ptr = %x Pulse On Time = %x\n", deviceID, ptr, *ptr);

	if (!SendReport(deviceID, VNX_ONTIME | VNX_SET, ptr, 4)){
		lsw[deviceID].PulseOnTimeSeconds = old_pulseontime; // IO failed, restore the old value	
		return BAD_HID_IO;
	}

	// we keep a copy of the scaled integer representation around too for internal use
	lsw[deviceID].PulseOnTime = temp_pulseontime;

	return STATUS_OK;
}


// Set the internal pulse modulation pulse off time
// Same as SetPulseOnTime as far as its parameters are concerned

int fnLSW_SetPulseOffTime(DEVID deviceID, float pulseofftime) {
	int temp_pulseofftime;

	if (deviceID >= MAXDEVICES)
		return INVALID_DEVID;

	if (!CheckDeviceOpen(deviceID))
		return DEVICE_NOT_READY;

	float old_pulseofftime = lsw[deviceID].PulseOffTimeSeconds;
	
	// ------ make sure parameter is in range ----------
	if (pulseofftime >= 150.0e-9 && pulseofftime < 1000){

		lsw[deviceID].PulseOffTimeSeconds = pulseofftime;
		if (TestMode){
			return STATUS_OK;		// in test mode we update our internal variables, but don't talk to the real HW
		}
	}
	else {
		return BAD_PARAMETER;		// we end up here if the off time is less than 150ns or greater than 1000 sec
	}


	if (pulseofftime <= .001){		// use the 48MHz clock

		// generate the integer off time, it does not have the range flags!!
		temp_pulseofftime = (int) (48 * (pulseofftime * 1.0e+6));
	}
	else { 
		temp_pulseofftime = (int) (pulseofftime * 1.0e+6);
	}

	// the pulse on time value is OK, lets send it to the hardware
	unsigned char *ptr = (unsigned char *) &temp_pulseofftime;

	if (DEBUG_OUT > 2) printf("deviceID = %x ptr = %x Pulse Off Time = %x\n", deviceID, ptr, *ptr);

	
	if (!SendReport(deviceID, VNX_OFFTIME | VNX_SET, ptr, 4)){
		lsw[deviceID].PulseOffTimeSeconds = old_pulseofftime;	
		return BAD_HID_IO;
	}

	// we keep a copy of the scaled integer representation around too for internal use
	lsw[deviceID].PulseOffTime = temp_pulseofftime;

	return STATUS_OK;
}

// Save the user settings to flash for autonomous operation

LVSTATUS fnLSW_SaveSettings(DEVID deviceID) {
  if (deviceID >= MAXDEVICES)
    return INVALID_DEVID;

  if (!CheckDeviceOpen(deviceID))
    return DEVICE_NOT_READY;

  if (TestMode)
    return STATUS_OK;		// in test mode we update our internal variables, but don't talk to the real HW

  char VNX_savesettings[] = {0x42, 0x55, 0x31}; //three byte key to unlock the user protection.

  if (!SendReport(deviceID, VNX_SAVEPAR | VNX_SET, VNX_savesettings, 3))
    return BAD_HID_IO;
  
  return STATUS_OK;
}

// ------------- Functions to get parameters --------------------- 

// Get the currently active switch

int fnLSW_GetActiveSwitch(DEVID deviceID) {
  if (deviceID >= MAXDEVICES)
    return INVALID_DEVID;
  
  if (!CheckDeviceOpen(deviceID))
    return DEVICE_NOT_READY;
  
  return lsw[deviceID].ActiveSwitch;
}

// Get the switch set by the user

int fnLSW_GetSwitchSetting(DEVID deviceID) {
  if (deviceID >= MAXDEVICES)
    return INVALID_DEVID;
  
  if (!CheckDeviceOpen(deviceID))
    return DEVICE_NOT_READY;
  
  return lsw[deviceID].SwitchSetting;
}

// Get the pattern type

int fnLSW_GetPatternType(DEVID deviceID) {
  if (deviceID >= MAXDEVICES)
    return INVALID_DEVID;

  if (!CheckDeviceOpen(deviceID))
    return DEVICE_NOT_READY;

  return lsw[deviceID].PatternType;
}

// Get the pattern length

int fnLSW_GetPatternLength(DEVID deviceID) {

  if (deviceID >= MAXDEVICES)
    return INVALID_DEVID;

  if (!CheckDeviceOpen(deviceID))
    return DEVICE_NOT_READY;

  return lsw[deviceID].PatternLength;
}

// Get the dwell time for one pattern entry

int fnLSW_GetPatternEntryTime(DEVID deviceID, int index) {

  if (deviceID >= MAXDEVICES)
    return INVALID_DEVID;

  if (!CheckDeviceOpen(deviceID))
    return DEVICE_NOT_READY;
	
  if (index > (MAX_PATTERN_LENGTH - 1) || index < 0)	// the index is 0 based
	return BAD_PARAMETER;

  return lsw[deviceID].SwitchPatternHoldTime[index];
}


// Get the switch setting for one pattern entry

int fnLSW_GetPatternEntrySwitch(DEVID deviceID, int index) {

  if (deviceID >= MAXDEVICES)
    return INVALID_DEVID;

  if (!CheckDeviceOpen(deviceID))
    return DEVICE_NOT_READY;
	
  if (index > (MAX_PATTERN_LENGTH - 1) || index < 0)	// the index is 0 based
	return BAD_PARAMETER;

  return lsw[deviceID].SwitchPatternSelection[index];
}

// Find out whether or not the Lab Brick is set to use external control signals

int fnLSW_GetUseExternalControl(DEVID deviceID) {

	if (deviceID >= MAXDEVICES)
		return INVALID_DEVID;

	if (!CheckDeviceOpen(deviceID))
		return DEVICE_NOT_READY;

	return lsw[deviceID].Use_External_SWControl;
}

// Fast pulse mode is standard equipment for a switch, but keep this function in case
// somebody is porting code written for one of the other products that uses this call

int fnLSW_GetHasFastPulseMode(DEVID deviceID)
{

	if (deviceID >= MAXDEVICES)
		return INVALID_DEVID;

	if (!CheckDeviceOpen(deviceID))
		return DEVICE_NOT_READY;

	return 1;		// every switch has fast pulse mode
}


// Get the pulse on time

float fnLSW_GetPulseOnTime(DEVID deviceID) {

	if (deviceID >= MAXDEVICES)
		return F_INVALID_DEVID;

	if (!CheckDeviceOpen(deviceID))
		return F_DEVICE_NOT_READY;

	return lsw[deviceID].PulseOnTimeSeconds;
}

// Find out if pulse mode output is active

int fnLSW_GetPulseMode(DEVID deviceID) {

	if (deviceID >= MAXDEVICES)
		return INVALID_DEVID;

	if (!CheckDeviceOpen(deviceID))
		return DEVICE_NOT_READY;

	if (lsw[deviceID].Modebits & MODE_PWMON){
		return 1;
		
	} else {
		return 0;
	}
}

// Get the pulse off time

float fnLSW_GetPulseOffTime(DEVID deviceID) {

	if (deviceID >= MAXDEVICES)
		return F_INVALID_DEVID;

	if (!CheckDeviceOpen(deviceID))
		return F_DEVICE_NOT_READY;
		
	return lsw[deviceID].PulseOffTimeSeconds;
}

// Get the state of the RF output - on or off

int fnLSW_GetRF_On(DEVID deviceID) {

  if (deviceID >= MAXDEVICES)
    return INVALID_DEVID;

  if (!CheckDeviceOpen(deviceID))
    return DEVICE_NOT_READY;

  if (lsw[deviceID].Modebits & MODE_RFON)
    return 1;
  else
    return 0;
}

// Get the number of switches

int fnLSW_GetNumSwitches(DEVID deviceID) {

  if (deviceID >= MAXDEVICES)
    return INVALID_DEVID;

  return lsw[deviceID].NumSwitches;
}
