// ---------------- Example program for the Vaunix RF Switch family ------------------
//
// -----------------------------------------------------------------------------------


#include <stdbool.h>	/* AK: Added include for 'bool' type */
#include <stdio.h>
#include <unistd.h>   	/* AK: Added include for error-free getlogin(). */
#include "LSWhid.h"
#include <math.h>

#define FALSE 0
#define TRUE !FALSE

/* function prototypes */

/* globals */


/* code begins here */
int main (int argc, char *argv[]) {
  int nDevices, nActive;
  int i, j, result, status;
  char cModelName[32];
  char c;
  char *username;
  DEVID activeDevices[MAXDEVICES];
  bool realhardware;

  /* AK: Added <unistd.h> to includes to avoid seg fault on getlogin(). */
  username = getlogin(); 

  if (0 != strcmp(username, "root")) {
    printf("Hi %s,\r\n", username);
    printf("Accessing USB ports on a Linux machine require root level\r\n");
    printf("access. You are not logged in as root. You may be able to\r\n");
    printf("proceed if you have used 'chmod' to change the access mode\r\n");
    printf("of the appropriate devices in /dev/bus/usb. That requires\r\n");
    printf("root access also. We'll continue, but if you don't see your\r\n");
    printf("LSW devices or no data can be read from or written to them,\r\n");
    printf("that's probably the problem. su to root and try again.\r\n\r\n");
    printf("Try running with 'sudo', or become root by running 'su' before.\r\n\r\n");
    
  }
  fnLSW_Init();
  /* If you have actual hardware attached, set this to TRUE. Setting to FALSE will run in test mode */
  realhardware = TRUE;
  fnLSW_SetTestMode(!realhardware);
  
  nDevices = fnLSW_GetNumDevices();
  printf("LSW test/demonstration program using library version %s\r\n\r\n", fnLSW_LibVersion());
  if (0 == nDevices) {
    printf("No Vaunix LSW devices located. Would you like to run in test mode? "); fflush(0);
    c = getchar();
    if ('Y' == (c & 0xdf)) {
      printf("\r\nSwitching to test mode.\r\n");
      realhardware = FALSE;
      fnLSW_Init();
      fnLSW_SetTestMode(!realhardware);
      nDevices = fnLSW_GetNumDevices();
    }
  }
  printf("Found %d devices\r\n", nDevices);
  
  for (i=1; i<=nDevices; i++) {
    result = fnLSW_GetModelName(i, cModelName);
    printf("  Model %d is %s (%d chars)\r\n", i, cModelName, result);
  }
  printf("\r\n");
  
  nActive = fnLSW_GetDevInfo(activeDevices);
  printf("We have %d active devices\r\n", nActive);
  
  for (i=0; i<nActive; i++) {
    /* let's open and init each device to get the threads running */
    status = fnLSW_InitDevice(activeDevices[i]);
    printf("  Opened device %d of %d. Return status=0x%08x (%s)\r\n", activeDevices[i], nActive, status, fnLSW_perror(status));
  }
  
  /* the data structure is filled by polling and we need a few seconds to do that */
  
  for (i=0; i<nActive; i++) {
    if (i > 0) printf("\r\n");
    
    /* only do this if not in test mode */
    printf("  Device %d is active\r\n", activeDevices[i]);
    /* dump what we know - that we read from the hardware */
    if (realhardware) {
      printf("  GetNumSwitches returned %d\r\n", fnLSW_GetNumSwitches(activeDevices[i]));
      printf("  GetPulseOnTime returned %g\r\n", fnLSW_GetPulseOnTime(activeDevices[i]));
      printf("  GetPulseOffTime returned %g\r\n", fnLSW_GetPulseOffTime(activeDevices[i]));
    }
    
    //    printf("  Device %d is active\r\n", activeDevices[i]);
    
    status = fnLSW_GetModelName(activeDevices[i], cModelName);
    printf("  Device %d (%s) has ", activeDevices[i], cModelName);
    status = fnLSW_GetSerialNumber(activeDevices[i]);
    printf("  Serial number=%d\r\n", status);
    
    printf("Select switch 1 for 5 seconds...\r\n");
    status = fnLSW_SetSwitch(activeDevices[i], 1);
    sleep(5);
    printf("Select switch 2 for 5 seconds...\r\n");
    status = fnLSW_SetSwitch(activeDevices[i], 2);
    sleep(5);
    if (fnLSW_GetNumSwitches(activeDevices[i]) == 4) {
      printf("Select switch 3 for 5 seconds...\r\n");
      status = fnLSW_SetSwitch(activeDevices[i], 3);
      sleep(5);
      printf("Select switch 4 for 5 seconds...\r\n");
      status = fnLSW_SetSwitch(activeDevices[i], 4);
      sleep(5);
    }
    
    printf("Pulsed Mode for 5 seconds...\r\n");
    status = fnLSW_SetFastPulsedOutput(activeDevices[i], .002, .004, TRUE);
    sleep(5);
    status = fnLSW_EnableInternalPulseMod(activeDevices[i], FALSE);
    
  }
  /* close the devices */
  for (i=0; i<nActive; i++) {
    status = fnLSW_CloseDevice(activeDevices[i]);
    printf("Closed device %d. Return status=0x%08x (%s)\r\n", activeDevices[i], status, fnLSW_perror(status));
  }
  printf("End of test\r\n");
  return 0;
}







