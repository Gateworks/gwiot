/***** Includes *****/
/* Standard C Libraries */
#include <stdlib.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <unistd.h>
#include <ctype.h>

/* BIOS header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>

#include "RFQueue.h"
#include "smartrf_settings.h"

#define RFMUX

/* Driverlib Header files */
#include DeviceFamily_constructPath(driverlib/rf_prop_mailbox.h)

#define THREADSTACKSIZE 1024

/* Packet RX/TX Configuration */
/* Max length byte the radio will accept */
#define PAYLOAD_LENGTH         30
/* Set Transmit (echo) delay to 100ms */
#define TX_DELAY             (uint32_t)(4000000*0.1f)
/* Set packet interval to 1000ms */
#define PACKET_INTERVAL      (uint32_t)(4000000*1.0f)
/* Set Receive timeout to 500ms */
#define RX_TIMEOUT           (uint32_t)(4000000*0.5f)
/* NOTE: Only two data entries supported at the moment */
#define NUM_DATA_ENTRIES       2
/* The Data Entries data field will contain:
 * 1 Header byte (RF_cmdPropRx.rxConf.bIncludeHdr = 0x1)
 * Max 30 payload bytes
 * 1 status byte (RF_cmdPropRx.rxConf.bAppendStatus = 0x1) */
#define NUM_APPENDED_BYTES     2

/*
 *  =============================== PIN ===============================
 */
#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>

/* Discrete Inputs */
#define Board_PIN_BTN1		IOID_15
#define Board_PIN_BTN2		IOID_14

/* LEDs */
#define Board_PIN_LEDRED	IOID_6
#define Board_PIN_LEDGRN	IOID_7

/* RF Antenna switch */
#define Board_RF_24GHZ		IOID_28
#define Board_RF_HIGH_PA	IOID_29
#define Board_RF_SUB1GHZ	IOID_30

/* UART Board */
#define Board_UART0_RX		IOID_12         /* RXD */
#define Board_UART0_TX		IOID_13         /* TXD */
#define Board_UART0_CTS		IOID_19         /* CTS */
#define Board_UART0_RTS		IOID_18         /* RTS */

const PINCC26XX_HWAttrs PINCC26XX_hwAttrs = {
	.intPriority = ~0,
};

/* * PIN Init table - all hardware muxed pins */
const PIN_Config pinInitTable[] = {
	/* LEDs */
	Board_PIN_LEDRED
	 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
	Board_PIN_LEDGRN
	 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,

	/* Buttons */
	Board_PIN_BTN1
	 | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_BOTHEDGES | PIN_HYSTERESIS,
	Board_PIN_BTN2
	 | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_BOTHEDGES | PIN_HYSTERESIS,
	IOID_26
	 | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_BOTHEDGES | PIN_HYSTERESIS,
	IOID_21
	 | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_BOTHEDGES | PIN_HYSTERESIS,

	/* UART */
	Board_UART0_RX
	 | PIN_INPUT_EN | PIN_PULLDOWN,
	Board_UART0_TX
	 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PULLDOWN,

	/* RF Antenna MUX GPIOs */
	Board_RF_24GHZ
	 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
	Board_RF_HIGH_PA
	 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
	Board_RF_SUB1GHZ
	 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,

	PIN_TERMINATE
};

/* * GPIO pinTable - pins we will manipulate via PIN driver */
const PIN_Config pinTable[] = {
	/* LEDs */
	Board_PIN_LEDRED
	 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
	Board_PIN_LEDGRN
	 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,

	/* Buttons */
	Board_PIN_BTN1
	 | PIN_GPIO_OUTPUT_DIS | PIN_INPUT_EN | PIN_PULLUP,
	Board_PIN_BTN2
	 | PIN_GPIO_OUTPUT_DIS | PIN_INPUT_EN | PIN_PULLUP,
	IOID_26
	 | PIN_GPIO_OUTPUT_DIS | PIN_INPUT_EN | PIN_PULLUP,
	IOID_21
	 | PIN_GPIO_OUTPUT_DIS | PIN_INPUT_EN | PIN_PULLUP,

	PIN_TERMINATE
};

static PIN_Handle pinHandle;

/* Board_keyCallback - called when a pin interrupt triggers:
 *  pinID: the pin number that triggered the event
 */
static int keycount = 0;
static void Board_keyCallback(PIN_Handle hPin, PIN_Id pinId)
{
	printf("key:%d:%d\r\n", pinId, keycount++);
}


/*
 *  =============================== Power ===============================
 */
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26X2.h>

const PowerCC26X2_Config PowerCC26X2_config = {
	.policyInitFxn      = NULL,
	.policyFxn          = &PowerCC26XX_standbyPolicy,
	.calibrateFxn       = &PowerCC26XX_calibrate,
	.enablePolicy       = true,
	.calibrateRCOSC_LF  = true,
	.calibrateRCOSC_HF  = true,
};


/*
 *  =============================== UART ===============================
 */
#include <ti/drivers/UART.h>
#include <ti/drivers/uart/UARTCC26XX.h>

UARTCC26XX_Object uartCC26XXObjects;

uint8_t uartCC26XXRingBuffer[32];

const UARTCC26XX_HWAttrsV2 uartCC26XXHWAttrs = {
        .baseAddr       = UART0_BASE,
        .powerMngrId    = PowerCC26XX_PERIPH_UART0,
        .intNum         = INT_UART0_COMB,
        .intPriority    = ~0,
        .txPin          = Board_UART0_TX,
        .rxPin          = Board_UART0_RX,
        .ctsPin         = PIN_UNASSIGNED,
        .rtsPin         = PIN_UNASSIGNED,
        .ringBufPtr     = uartCC26XXRingBuffer,
        .ringBufSize    = sizeof(uartCC26XXRingBuffer),
        .txIntFifoThr   = UARTCC26XX_FIFO_THRESHOLD_1_8,
        .rxIntFifoThr   = UARTCC26XX_FIFO_THRESHOLD_4_8,
        .errorFxn       = NULL
};

const UART_Config UART_config = {
        .fxnTablePtr = &UARTCC26XX_fxnTable,
        .object      = &uartCC26XXObjects,
        .hwAttrs     = &uartCC26XXHWAttrs
};

const uint_least8_t UART_count = 1;

/* UART Task: trasmit UART buffer */
static UART_Handle uart;
static char uart_txbuf[2048];
static Semaphore_Struct semStruct;
static Semaphore_Handle semHandle;
static Task_Struct uartTask;
static uint8_t uartTaskStack[THREADSTACKSIZE];
static void uartTaskFunction(UArg arg0, UArg arg1)
{
	while(1) {
		/* Waiting for output */
		Semaphore_pend(semHandle, BIOS_WAIT_FOREVER);
		UART_write(uart, uart_txbuf, strlen(uart_txbuf));
		uart_txbuf[0] = 0;
	}
}


/* write to the UART TX buffer and post Semaphore
 * (avoid using UART_write so this can be called from ISR's and callbacks)
 */
int printf(const char *fmt, ...)
{
	va_list ap;
	int len, avail, size;

	Task_disable();
	len = strlen(uart_txbuf);
	avail = sizeof(uart_txbuf) - len - 1;
	va_start(ap, fmt);
	size = vsnprintf(uart_txbuf + len, avail, fmt, ap);
	va_end(ap);
	Semaphore_post(semHandle);
	Task_enable();

	return size;
}

/*
 *  =============================== RF Driver ===============================
 */
#include <ti/drivers/rf/RF.h>

/* Antenna switching */
static PIN_Handle antPins;
static PIN_State antennaState;
const PIN_Config antennaConfig[] = {
	Board_RF_24GHZ
	 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
	Board_RF_HIGH_PA
	 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
	Board_RF_SUB1GHZ
	 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
	PIN_TERMINATE
};

void initAntennaSwitch()
{
	antPins = PIN_open(&antennaState, antennaConfig);
}

/*
 *  Board-specific callback function to set the correct antenna path.
 *
 *  This function is called by the RF driver on global driver events and
 *  an be used to setup the antenna switch depending on the current PHY
 *  configuration.
 *
 *  For example on a CC1352P1_LAUNCHXL there is an RF Switch connected
 *  as such:
 *
 *  Path        DIO28 DIO29 DIO30
 *  =========== ===== ===== =====
 *  Off         0     0     0
 *  Sub-1 GHz   0     0     1
 *  2.4 GHz     1     0     0
 *  20 dBm TX   0     1     0
 */
void rfDriverCallback(RF_Handle client, RF_GlobalEvent events, void *arg)
{
	const char *res = "";
	bool    sub1GHz   = false;
	uint8_t loDiv = 0;

#ifdef RFMUX
	/* Switch off all paths first */
	PINCC26XX_setOutputValue(Board_RF_24GHZ, 0);
	PINCC26XX_setOutputValue(Board_RF_HIGH_PA, 0);
	PINCC26XX_setOutputValue(Board_RF_SUB1GHZ, 0);
#endif

	if (events & RF_GlobalEventRadioSetup) {
		/* Decode the current PA configuration. */
		RF_TxPowerTable_PAType paType =
			(RF_TxPowerTable_PAType)RF_getTxPower(client).paType;

		/* Decode the generic argument as a setup command. */
		RF_RadioSetup* setupCmd = (RF_RadioSetup*)arg;

		switch (setupCmd->common.commandNo) {
		case (CMD_RADIO_SETUP):
		case (CMD_BLE5_RADIO_SETUP):
			/* Sub-1GHz front-end. */
			loDiv = RF_LODIVIDER_MASK & setupCmd->common.loDivider;
			if (loDiv != 0)
				sub1GHz = true;
			break;
		case (CMD_PROP_RADIO_DIV_SETUP):
			/* Sub-1GHz front-end. */
			loDiv = RF_LODIVIDER_MASK & setupCmd->prop_div.loDivider;
			if (loDiv != 0)
				sub1GHz = true;
			break;
		default:
			break;
		}

		if (sub1GHz) {
			/* Sub-1 GHz */
			if (paType == RF_TxPowerTable_HighPA) {
				res = "sub1GHz PA";
#ifdef RFMUX
				/* PA enable --> HIGH PA
				 * LNA enable --> Sub-1 GHz
				 */
				PINCC26XX_setMux(antPins, Board_RF_24GHZ,
						 PINCC26XX_MUX_GPIO);
				PINCC26XX_setMux(antPins, Board_RF_HIGH_PA,
						 PINCC26XX_MUX_RFC_GPO3);
				PINCC26XX_setMux(antPins, Board_RF_SUB1GHZ,
						 PINCC26XX_MUX_RFC_GPO0);
#endif
			} else {
				res = "sub1GHz";
#ifdef RFMUX
				/* RF core active --> Sub-1 GHz */
				PINCC26XX_setMux(antPins, Board_RF_24GHZ,
						 PINCC26XX_MUX_GPIO);
				PINCC26XX_setMux(antPins, Board_RF_HIGH_PA,
						 PINCC26XX_MUX_GPIO);
				PINCC26XX_setMux(antPins, Board_RF_SUB1GHZ,
						 PINCC26XX_MUX_GPIO);
				PINCC26XX_setOutputValue(Board_RF_SUB1GHZ, 1);
#endif
			}
		} else {
			/* 2.4 GHz */
			if (paType == RF_TxPowerTable_HighPA) {
				res = "2.4GHz PA";
#ifdef RFMUX
				/* PA enable --> HIGH PA
				 * LNA enable --> 2.4 GHz
				 */
				PINCC26XX_setMux(antPins, Board_RF_24GHZ,
						 PINCC26XX_MUX_RFC_GPO0);
				PINCC26XX_setMux(antPins, Board_RF_HIGH_PA,
						 PINCC26XX_MUX_RFC_GPO3);
				PINCC26XX_setMux(antPins, Board_RF_SUB1GHZ,
						 PINCC26XX_MUX_GPIO);
#endif
			} else {
				res = "2.4GHz";
#ifdef RFMUX
				/* RF core active --> 2.4 GHz */
				PINCC26XX_setMux(antPins, Board_RF_24GHZ,
						 PINCC26XX_MUX_GPIO);
				PINCC26XX_setMux(antPins, Board_RF_HIGH_PA,
						 PINCC26XX_MUX_GPIO);
				PINCC26XX_setMux(antPins, Board_RF_SUB1GHZ,
						 PINCC26XX_MUX_GPIO);
				PINCC26XX_setOutputValue(Board_RF_24GHZ, 1);
#endif
			}
		}
	} else {
		/* Reset the IO multiplexer to GPIO functionality */
		res = "reset";
#ifdef RFMUX
		PINCC26XX_setMux(antPins, Board_RF_24GHZ, PINCC26XX_MUX_GPIO);
		PINCC26XX_setMux(antPins, Board_RF_HIGH_PA, PINCC26XX_MUX_GPIO);
		PINCC26XX_setMux(antPins, Board_RF_SUB1GHZ, PINCC26XX_MUX_GPIO);
#endif
	}
	printf("RF:0x%x:%s\r\n", events, res);
}

const RFCC26XX_HWAttrsV2 RFCC26XX_hwAttrs = {
	.hwiPriority        = ~0,     /* Lowest HWI priority */
	.xoscHfAlwaysNeeded = true,   /* Keep XOSC dependency while in stanby */

	/* Register the board specific callback */
	.globalCallback     = &rfDriverCallback,

	/* Subscribe the callback to both events */
	.globalEventMask    = RF_GlobalEventRadioSetup | RF_GlobalEventRadioPowerDown
};

static RF_Object rfObject;
static RF_Handle rfHandle;
/* Buffer which contains all Data Entries for receiving data.
 * Pragmas are needed to make sure this buffer is aligned to a 4 byte boundary
 * (requirement from the RF core)
 */
static uint8_t
rxDataEntryBuffer[RF_QUEUE_DATA_ENTRY_BUFFER_SIZE(NUM_DATA_ENTRIES,
                                                  PAYLOAD_LENGTH,
                                                  NUM_APPENDED_BYTES)]
                                                  __attribute__((aligned(4)));
/* Receive Statistics */
static rfc_propRxOutput_t rxStatistics;
/* Receive dataQueue for RF Core to fill in data */
static dataQueue_t dataQueue;
static rfc_dataEntryGeneral_t* currentDataEntry;
static uint8_t packetLength;
static uint8_t* packetDataPointer;
/* Transmit Packet */
static uint8_t txPacket[PAYLOAD_LENGTH];
static uint16_t seqNumber;

static void rx_echoCallback(RF_Handle h, RF_CmdHandle ch, RF_EventMask e)
{
	if (e & RF_EventRxEntryDone) {
		/* Successful RX */
		printf("rx:%d\r\n", seqNumber++);
		/* Toggle LEDRED to indicate RX */
		PIN_setOutputValue(pinHandle, Board_PIN_LEDRED,
				!PIN_getOutputValue(Board_PIN_LEDRED));

		/* Get current unhandled data entry */
		currentDataEntry = RFQueue_getDataEntry();

		/* Handle the packet data, located at &currentDataEntry->data:
		 * - Length is the first byte with the current configuration
		 * - Data starts from the second byte */
		packetLength      = *(uint8_t *)(&(currentDataEntry->data));
		packetDataPointer = (uint8_t *)(&(currentDataEntry->data) + 1);

		/* Copy payload+status byte to rxPacket then over to txPacket */
		memcpy(txPacket, packetDataPointer, packetLength);
		RFQueue_nextEntry();
	} else if (e & RF_EventLastCmdDone) {
		/* Successful Echo (TX)*/
		printf("tx\r\n");

	}
	else {
		/* any uncaught event */
		printf("err\r\n");
		/* Error Condition: set LED1/LED2 */
		PIN_setOutputValue(pinHandle, Board_PIN_LEDRED, 1);
		PIN_setOutputValue(pinHandle, Board_PIN_LEDGRN, 1);
	}
}

void rx(RF_Params *rfParams)
{
	const char *res;
	printf("rx thread\r\n");

	PIN_setOutputValue(pinHandle, Board_PIN_LEDGRN, 1);
	PIN_setOutputValue(pinHandle, Board_PIN_LEDRED, 0);

	/* Modify CMD_PROP_TX and CMD_PROP_RX commands for application needs */
	/* Set the Data Entity queue for received data */
	RF_cmdPropRx.pQueue = &dataQueue;
	/* Discard ignored packets from Rx queue */
	RF_cmdPropRx.rxConf.bAutoFlushIgnored = 1;
	/* Discard packets with CRC error from Rx queue */
	RF_cmdPropRx.rxConf.bAutoFlushCrcErr = 1;
	/* Implement packet length filtering to avoid PROP_ERROR_RXBUF */
	RF_cmdPropRx.maxPktLen = PAYLOAD_LENGTH;
	/* End RX operation when a packet is received correctly and move to the
	 * next command in the chain */
	RF_cmdPropRx.pktConf.bRepeatOk = 0;
	RF_cmdPropRx.pktConf.bRepeatNok = 1;
	RF_cmdPropRx.startTrigger.triggerType = TRIG_NOW;
	RF_cmdPropRx.pNextOp = (rfc_radioOp_t *)&RF_cmdPropTx;
	/* Only run the TX command if RX is successful */
	RF_cmdPropRx.condition.rule = COND_STOP_ON_FALSE;
	RF_cmdPropRx.pOutput = (uint8_t *)&rxStatistics;

	RF_cmdPropTx.pktLen = PAYLOAD_LENGTH;
	RF_cmdPropTx.pPkt = txPacket;
	RF_cmdPropTx.startTrigger.triggerType = TRIG_REL_PREVEND;
	RF_cmdPropTx.startTime = TX_DELAY;

	/* Request access to the radio */
	rfParams->nInactivityTimeout = 2000; /* 1s timeout */
	rfHandle = RF_open(&rfObject, &RF_prop,
			(RF_RadioSetup*)&RF_cmdPropRadioDivSetup, rfParams);

	/* Set the frequency */
	RF_postCmd(rfHandle, (RF_Op*)&RF_cmdFs, RF_PriorityNormal, NULL, 0);

	while(1)
	{
		/* Wait for a packet
		 * - When first of the two chained commands (RX) completes, the
		 * RF_EventCmdDone/RF_EventRxEntryDone events are raised on a
		 * successful packet reception, then the next command in chain
		 * (TX) is run
		 * - If RF core runs into an issue after receiving the packet
		 * incorrectly onlt RF_EventCmdDone event is raised; this is an
		 * error condition
		 * - If RF core successfully echos received packet the RF core
		 * should raise the RF_EventLastCmdDone event
		 */
		RF_EventMask terminationReason =
			RF_runCmd(rfHandle, (RF_Op*)&RF_cmdPropRx,
				  RF_PriorityNormal, rx_echoCallback,
				  (RF_EventRxEntryDone | RF_EventLastCmdDone));

		res = NULL;
		switch(terminationReason)
		{
		case RF_EventLastCmdDone:
			// A stand-alone radio operation command or last radio
			// operation command in a chain finished.
			break;
		case RF_EventCmdCancelled:
			res = "cancelled";
			// Command cancelled before it was started;
			// it can be caused by RF_cancelCmd() or RF_flushCmd().
			break;
		case RF_EventCmdAborted:
			res = "aborted";
			// Abrupt command termination caused by RF_cancelCmd()
			// or RF_flushCmd().
			break;
		case RF_EventCmdStopped:
			res = "stopped";
			// Graceful command termination caused by
			// RF_cancelCmd() or RF_flushCmd().
			break;
		default:
			res = "unknown";
			// Uncaught error event
		}
		if (res)
			printf("rx terminated:%d:%s\r\n",
				(int)terminationReason, res);

		uint32_t cmdStatus = ((volatile RF_Op*)&RF_cmdPropRx)->status;
		res = NULL;
		switch(cmdStatus)
		{
		case PROP_DONE_OK:
			// Packet received with CRC OK
			break;
		case PROP_DONE_RXERR:
			res = "rxerr";
			// Packet received with CRC error
			break;
		case PROP_DONE_RXTIMEOUT:
			res = "rxtimeout";
			// Observed end trigger while in sync search
			break;
		case PROP_DONE_BREAK:
			res = "break";
			// Observed end trigger while receiving packet when
			// the command is configured with endType set to 1
			break;
		case PROP_DONE_ENDED:
			res = "ended";
			// Received packet after having observed the end
			// trigger; if the command is configured with endType
			// set to 0, the end trigger will not terminate an
			// ongoing reception
			break;
		case PROP_DONE_STOPPED:
			res = "stopped";
			// received CMD_STOP after command started and,
			// if sync found, packet is received
			break;
		case PROP_DONE_ABORT:
			res = "abort";
			// Received CMD_ABORT after command started
			break;
		case PROP_ERROR_RXBUF:
			res = "rxbuf";
			// No RX buffer large enough for the received data
			// available at the start of a packet
			break;
		case PROP_ERROR_RXFULL:
			res = "rxbuffull";
			// Out of RX buffer space during reception in a
			// partial read
			break;
		case PROP_ERROR_PAR:
			res = "param";
			// Observed illegal parameter
			break;
		case PROP_ERROR_NO_SETUP:
			res = "setup";
			// Command sent without setting up radio in a supported
			// mode using CMD_PROP_RADIO_SETUP or CMD_RADIO_SETUP
			break;
		case PROP_ERROR_NO_FS:
			res = "fs";
			// Command sent without the synthesizer being programmed
			break;
		case PROP_ERROR_RXOVF:
			res = "rxovf";
			// RX overflow observed during operation
			break;
		default:
			res = "unknown";
			// Uncaught error event - these could come from the
			// pool of states defined in rf_mailbox.h
		}
		if (res)
			printf("status:0x%lx:%s\r\n", cmdStatus, res);
	}
}

void tx(RF_Params *rfParams)
{
	const char *res;
	uint32_t curtime;
	printf("tx thread: txPower=0x%04x (%ddBm)\r\n",
		RF_cmdPropRadioDivSetup.txPower,
		(RF_cmdPropRadioDivSetup.txPower == 0xffff) ? 20 : 14);

	/* Modify CMD_PROP_TX and CMD_PROP_RX commands for application needs */
	RF_cmdPropTx.pktLen = PAYLOAD_LENGTH;
	RF_cmdPropTx.pPkt = txPacket;
	RF_cmdPropTx.startTrigger.triggerType = TRIG_ABSTIME;
	RF_cmdPropTx.startTrigger.pastTrig = 1;
	RF_cmdPropTx.startTime = 0;
	RF_cmdPropTx.pNextOp = (rfc_radioOp_t *)&RF_cmdPropRx;
	/* Only run the RX command if TX is successful */
	RF_cmdPropTx.condition.rule = COND_STOP_ON_FALSE;

	/* Set the Data Entity queue for received data */
	RF_cmdPropRx.pQueue = &dataQueue;
	/* Discard ignored packets from Rx queue */
	RF_cmdPropRx.rxConf.bAutoFlushIgnored = 1;
	/* Discard packets with CRC error from Rx queue */
	RF_cmdPropRx.rxConf.bAutoFlushCrcErr = 1;
	/* Implement packet length filtering to avoid PROP_ERROR_RXBUF */
	RF_cmdPropRx.maxPktLen = PAYLOAD_LENGTH;
	RF_cmdPropRx.pktConf.bRepeatOk = 0;
	RF_cmdPropRx.pktConf.bRepeatNok = 0;
	RF_cmdPropRx.pOutput = (uint8_t *)&rxStatistics;
	/* Receive operation will end RX_TIMEOUT ms after command starts */
	RF_cmdPropRx.endTrigger.triggerType = TRIG_REL_PREVEND;
	RF_cmdPropRx.endTime = RX_TIMEOUT;

	/* Request access to the radio */
	rfHandle = RF_open(&rfObject, &RF_prop,
			(RF_RadioSetup*)&RF_cmdPropRadioDivSetup, rfParams);

	/* Set the frequency */
	RF_postCmd(rfHandle, (RF_Op*)&RF_cmdFs, RF_PriorityNormal, NULL, 0);

	/* Get current time */
	curtime = RF_getCurrentTime();

	while(1)
	{
		printf("tx:%d\r\n", seqNumber);
		/* Create packet with incrementing sequence number
		 * and random payload */
		txPacket[0] = (uint8_t)(seqNumber >> 8);
		txPacket[1] = (uint8_t)(seqNumber++);
		uint8_t i;
		for (i = 2; i < PAYLOAD_LENGTH; i++)
		{
			txPacket[i] = rand();
		}

		/* Set absolute TX time to utilize automatic power management */
		curtime += PACKET_INTERVAL;
		RF_cmdPropTx.startTime = curtime;

		/* Send packet */
		RF_EventMask terminationReason = RF_runCmd(rfHandle,
							  (RF_Op*)&RF_cmdPropTx,
							  RF_PriorityNormal,
							  NULL, 0);

		res = NULL;
		switch(terminationReason)
		{
		case RF_EventLastCmdDone:
			// A stand-alone radio operation command or the last
			// radio operation command in a chain finished.
			break;
		case RF_EventCmdCancelled:
			res = "cancelled";
			// Command cancelled before it was started; it can be
			// caused by RF_cancelCmd() or RF_flushCmd().
			break;
		case RF_EventCmdAborted:
			res = "aborted";
			// Abrupt command termination caused by RF_cancelCmd()
			// or RF_flushCmd().
			break;
		case RF_EventCmdStopped:
			res = "stopped";
			// Graceful command termination caused by
			// RF_cancelCmd() or RF_flushCmd().
			break;
		default:
			res = "unknown";
			// Uncaught error event
		}
		if (res)
			printf("txterminated:%d:%s\r\n",
				(int)terminationReason, res);

		uint32_t cmdStatus = ((volatile RF_Op*)&RF_cmdPropTx)->status;
		res = NULL;
		switch(cmdStatus)
		{
		case PROP_DONE_OK:
			// Packet transmitted successfully
			break;
		case PROP_DONE_STOPPED:
			res = "stopped";
			// received CMD_STOP while transmitting packet and
			// finished transmitting packet
			break;
		case PROP_DONE_ABORT:
			res = "abort";
			// Received CMD_ABORT while transmitting packet
			break;
		case PROP_ERROR_PAR:
			res = "param";
			// Observed illegal parameter
			break;
		case PROP_ERROR_NO_SETUP:
			res = "setup";
			// Command sent without setting up the radio in a
			// supported mode using CMD_PROP_RADIO_SETUP or
			// CMD_RADIO_SETUP
			break;
		case PROP_ERROR_NO_FS:
			res = "fs";
			// Command sent without the synthesizer being programmed
			break;
		case PROP_ERROR_TXUNF:
			res = "txunf";
			// TX underflow observed during operation
			break;
		default:
			res = "unkown";
			// Uncaught error event - these could come from the
			// pool of states defined in rf_mailbox.h
		}
		if (res)
			printf("txstatus:0x%lx:%s\r\n", cmdStatus, res);

		/* Toggle RED LED to indicate TX */
		PIN_setOutputValue(pinHandle, Board_PIN_LEDRED,
			!PIN_getOutputValue(Board_PIN_LEDRED));
		/* Toggle GRN LED if high PA */
		if (RF_cmdPropRadioDivSetup.txPower == 0xffff) {
			PIN_setOutputValue(pinHandle, Board_PIN_LEDGRN,
				!PIN_getOutputValue(Board_PIN_LEDGRN));
		}
	}
}

static Task_Struct rfTask;
static uint8_t rfTaskStack[THREADSTACKSIZE];

static void rfTaskFunction(UArg arg0, UArg arg1)
{
	char mode_rx = 1;
	char highpa = 1;
	int input;
	int rz;
	RF_Params rfParams;

	/* set both LEDs off */
	PIN_setOutputValue(pinHandle, Board_PIN_LEDGRN, 0);
	PIN_setOutputValue(pinHandle, Board_PIN_LEDRED, 0);

	printf("\r\n\r\nrfDemo:\r\nMode (T/r)?:");
	rz = UART_read(uart, &input, 1);
	printf("\r\n");
	if (rz == 1) {
		if (tolower(input) != 'r')
			mode_rx = 0;
		printf("high gain (Y/n):");
		rz = UART_read(uart, &input, 1);
		if (rz == 1 && tolower(input) == 'n')
			highpa = 0;
		printf("\r\n");
	}
	else
	{
		// BTN2 || DIO_26 low = TX (default RX)
		// BTN1 || DIO_21 low = 14dBm (default 20dBm)
		printf("BTN1:DIO%d=%d\r\n", Board_PIN_BTN1,
			(int)PIN_getInputValue(Board_PIN_BTN1));
		printf("BTN2:DIO%d=%d\r\n", Board_PIN_BTN2,
			(int)PIN_getInputValue(Board_PIN_BTN2));
		printf("DIO26:DIO%d=%d\r\n", IOID_26,
			(int)PIN_getInputValue(IOID_26));
		printf("DIO21:DIO%d=%d\r\n", IOID_21,
			(int)PIN_getInputValue(IOID_21));
		if (!PIN_getInputValue(Board_PIN_BTN2) ||
		    !PIN_getInputValue(IOID_26))
			mode_rx = 0;
		if (!PIN_getInputValue(Board_PIN_BTN1) ||
		    !PIN_getInputValue(IOID_21))
			highpa = 0;
	}

	printf("modex_rx=%d highpa=%d\r\n", mode_rx, highpa);

	/* High Gain PA modifies the following in RF_cmdPropRadioDivSetup:
	 * txPower (14dBm:0x013F 20dBm:0xFFFF)
	 * pRegOverride (14dBm:add ADI_REG_OVERRIDE(0,12,0xF8))
	 */
	if (highpa) {
		RF_cmdPropRadioDivSetup.txPower = 0xffff;
		RF_cmdPropRadioDivSetup.pRegOverride = pOverrides20dBm;
	}

	/* Initialize RF Com */
	RF_Params_init(&rfParams);

	if( RFQueue_defineQueue(&dataQueue,
				rxDataEntryBuffer,
				sizeof(rxDataEntryBuffer),
				NUM_DATA_ENTRIES,
				PAYLOAD_LENGTH + NUM_APPENDED_BYTES))
	{
		/* Failed to allocate space for all data entries */
		PIN_setOutputValue(pinHandle, Board_PIN_LEDRED, 1);
		PIN_setOutputValue(pinHandle, Board_PIN_LEDGRN, 1);
		while(1);
	}

	if (mode_rx)
		rx(&rfParams);
	else
		tx(&rfParams);
}

int main(void)
{
	PIN_State pinState;
	Task_Params rfTaskParams;
	Task_Params uartTaskParams;

	/* Power init */
	Power_init();

	/* Initialize PIN config */
	PIN_init(pinInitTable);

	initAntennaSwitch();

	/* Configure PINs for GPIO use */
	pinHandle = PIN_open(&pinState, pinTable);
	PIN_registerIntCb(pinHandle, Board_keyCallback);
	PIN_setConfig(pinHandle, PIN_BM_IRQ, Board_PIN_BTN1 | PIN_IRQ_NEGEDGE);
	PIN_setConfig(pinHandle, PIN_BM_IRQ, Board_PIN_BTN2 | PIN_IRQ_NEGEDGE);

	/* Create a UART with data processing off. */
	UART_init();
	UART_Params uartParams;
	UART_Params_init(&uartParams);
	uartParams.writeDataMode = UART_DATA_BINARY;
	uartParams.readDataMode = UART_DATA_BINARY;
	uartParams.readReturnMode = UART_RETURN_FULL;
	uartParams.readTimeout = 5000 * (1000 / Clock_tickPeriod); // 5s
	uartParams.readEcho = UART_ECHO_OFF;
	uartParams.baudRate = 115200;
	uart = UART_open(0, &uartParams);
	/* Construct Semaphore for resource lock of UART buffer */
	Semaphore_Params semParams;
	Semaphore_Params_init(&semParams);
	Semaphore_construct(&semStruct, 0, &semParams);
	semHandle = Semaphore_handle(&semStruct);
	/* Create Task for transmitting UART buffer */
	Task_Params_init(&uartTaskParams);
	uartTaskParams.stackSize = THREADSTACKSIZE;
	uartTaskParams.priority = 1;
	uartTaskParams.stack = &uartTaskStack;
	uartTaskParams.arg0 = 0;
	Task_construct(&uartTask, uartTaskFunction, &uartTaskParams, NULL);

	/* Create Task for RF */
	Task_Params_init(&rfTaskParams);
	rfTaskParams.stackSize = THREADSTACKSIZE;
	rfTaskParams.priority = 2;
	rfTaskParams.stack = &rfTaskStack;
	rfTaskParams.arg0 = 0;
	Task_construct(&rfTask, rfTaskFunction, &rfTaskParams, NULL);

	/* Start BIOS */
	BIOS_start();

	return 0;
}
