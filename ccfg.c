/*
 *  ======== ccfg.c ========
 *  Customer Configuration for CC26xx and CC13xx devices.  This file is used to
 *  configure Boot ROM, start-up code, and SW radio behaviour.
 *
 *  By default, driverlib startup_files/ccfg.c settings are used.  However, if
 *  changes are required there are two means to do so:
 *
 *    1.  Remove this file and copy driverlib's startup_files/ccfg.c file in
 *        its place.  Make all changes to the file.  Changes made are local to
 *        the project and will not affect other projects.
 *
 *    2.  Perform changes to driverlib startup_files/ccfg.c file.  Changes
 *        made to this file will be applied to all projects.  This file must
 *        remain unmodified.
 */

#define SET_CCFG_BL_CONFIG_BOOTLOADER_ENABLE    0xC5    // Enable ROM boot loader
#define SET_CCFG_BL_CONFIG_BL_LEVEL             0x1     // Active high to open boot loader backdoor
#define SET_CCFG_BL_CONFIG_BL_PIN_NUMBER        20      // DIO20
#define SET_CCFG_BL_CONFIG_BL_ENABLE            0xC5    // Enabled bootloader backdoor

#include <ti/devices/DeviceFamily.h>
/* ~/ti/simplelink_cc13x2_sdk_2_30_00_45/source/ti/devices/cc13x2_cc26x2_v1/startup_files/ccfg.c */
#include DeviceFamily_constructPath(startup_files/ccfg.c)
