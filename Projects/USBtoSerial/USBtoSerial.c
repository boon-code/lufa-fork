/*
             LUFA Library
     Copyright (C) Dean Camera, 2014.

  dean [at] fourwalledcubicle [dot] com
           www.lufa-lib.org
*/

/*
  Copyright 2014  Dean Camera (dean [at] fourwalledcubicle [dot] com)

  Permission to use, copy, modify, distribute, and sell this
  software and its documentation for any purpose is hereby granted
  without fee, provided that the above copyright notice appear in
  all copies and that both that the copyright notice and this
  permission notice and warranty disclaimer appear in supporting
  documentation, and that the name of the author not be used in
  advertising or publicity pertaining to distribution of the
  software without specific, written prior permission.

  The author disclaims all warranties with regard to this
  software, including all implied warranties of merchantability
  and fitness.  In no event shall the author be liable for any
  special, indirect or consequential damages or any damages
  whatsoever resulting from loss of use, data or profits, whether
  in an action of contract, negligence or other tortious action,
  arising out of or in connection with the use or performance of
  this software.
*/

/** \file
 *
 *  Main source file for the USBtoSerial project. This file contains the main tasks of
 *  the project and is responsible for the initial application hardware configuration.
 */

#include "USBtoSerial.h"
#include <util/delay.h>
#include <avr/sleep.h>

/** Circular buffer to hold data from the host before it is sent to the device via the serial port. */
static RingBuffer_t USBtoUSART_Buffer;

/** Underlying data buffer for \ref USBtoUSART_Buffer, where the stored bytes are located. */
static uint8_t      USBtoUSART_Buffer_Data[128];

/** Circular buffer to hold data from the serial port before it is sent to the host. */
static RingBuffer_t USARTtoUSB_Buffer;

/** Underlying data buffer for \ref USARTtoUSB_Buffer, where the stored bytes are located. */
static uint8_t      USARTtoUSB_Buffer_Data[128];

/** LUFA CDC Class driver interface configuration and state information. This structure is
 *  passed to all CDC Class driver functions, so that multiple instances of the same class
 *  within a device can be differentiated from one another.
 */
USB_ClassInfo_CDC_Device_t VirtualSerial_CDC_Interface =
	{
		.Config =
			{
				.ControlInterfaceNumber         = INTERFACE_ID_CDC_CCI,
				.DataINEndpoint                 =
					{
						.Address                = CDC_TX_EPADDR,
						.Size                   = CDC_TXRX_EPSIZE,
						.Banks                  = 1,
					},
				.DataOUTEndpoint                =
					{
						.Address                = CDC_RX_EPADDR,
						.Size                   = CDC_TXRX_EPSIZE,
						.Banks                  = 1,
					},
				.NotificationEndpoint           =
					{
						.Address                = CDC_NOTIFICATION_EPADDR,
						.Size                   = CDC_NOTIFICATION_EPSIZE,
						.Banks                  = 1,
					},
			},
	};

static uint8_t monitor_refresh = 1;
static FILE usb_stream;

volatile uint16_t *bootKeyPtr = (volatile uint16_t *)0x0800;

static void enter_bootloader (void)
{
	cli();
	MCUCR |= (1 << IVCE);
	MCUCR = (1 << IVSEL);
	bootKeyPtr[0] = 0x7777;
	wdt_enable(WDTO_250MS);
	while (1) {}
}

#define PSAVE_MCU_LOCKED() do { \
        sleep_enable(); \
        set_sleep_mode(SLEEP_MODE_PWR_SAVE); \
        sei(); \
        sleep_cpu(); \
        sleep_disable(); \
} while (0)

#define PDOWN_MCU_LOCKED() do { \
        sleep_enable(); \
        set_sleep_mode(SLEEP_MODE_PWR_DOWN); \
        sei();			     \
        sleep_cpu(); \
        sleep_disable(); \
} while (0)

static volatile wdt_counter = 0;

ISR(WDT_vect)
{
	++wdt_counter;

	switch(wdt_counter) {
	case 10:
		LEDs_SetAllLEDs(LEDS_LED1);
		break;
	case 21:
	case 31:
	case 11:
		LEDs_SetAllLEDs(0);
		break;
	case 20:
		LEDs_SetAllLEDs(LEDS_LED2);
		break;
	case 30:
		LEDs_SetAllLEDs(LEDS_LED3);
		break;
	default:
		break;
	}
}

#define WDT_CONFIG_MASK (_BV(WDIE) | _BV(WDE))

#define wdt_config_locked(config,value) \
__asm__ __volatile__ ( \
	"in __tmp_reg__,__SREG__" "\n\t" \
	"cli" "\n\t" \
	"wdr" "\n\t" \
	"sts %0,%1" "\n\t" \
	"out __SREG__,__tmp_reg__" "\n\t" \
	"sts %0,%2" "\n\t" \
 \
	: /* no outputs */ \
	: "M" (_SFR_MEM_ADDR(_WD_CONTROL_REG)), \
	"r" (_BV(_WD_CHANGE_BIT) | _BV(WDE)), \
	"r" ((uint8_t) ((value & 0x08 ? _WD_PS3_MASK : 0x00) | \
             (config & WDT_CONFIG_MASK) | (value & 0x07)) ) \
	: "r0" \
)

static void blink (uint8_t max)
{
	uint8_t i;

	for (i = 0; i < max; ++i) {
		LEDs_SetAllLEDs(LEDS_LED1 | LEDS_LED2 | LEDS_LED3);
		_delay_ms(250);
		LEDs_SetAllLEDs(0);
		_delay_ms(250);
	}
}

static void enter_deep_sleep (void)
{
	uint8_t i;

	cli();
	LEDs_SetAllLEDs(0);
	USB_Detach();
	blink(4);
	wdt_counter = 0;
	while (wdt_counter < 1) {
		wdt_config_locked(_BV(WDIE),WDTO_8S);
		PDOWN_MCU_LOCKED();
	}
	wdt_disable();
	blink(4);
	USB_Attach();
}

#define MSR_COUNTER_MAX 8000
#define MSR_RUNNING_bm 0x1
#define MSR_ADC_STARTED_bm 0x2

static uint8_t msr_state = 0;
static uint16_t msr_counter = 0;

static inline uint8_t msr_is_started (void)
{
	return (msr_state & MSR_RUNNING_bm);
}

static void msr_start (void)
{
	if (msr_state & MSR_RUNNING_bm)
		return;

	ADMUX = (1 << 6) | /* REF0 => AVcc */
		(1 << 5) | /* ADLAR=1 => fill ADCH */
		0b110; /* ADC6 */

	ADCSRA = (1 << ADEN) | /* enable ADC */
		 (1 << ADIF) | /* reset IRQ */
		 0b111; /* divide clock by 128 */
	ADCSRB = 0x0;

	msr_state = MSR_RUNNING_bm;
}

static void msr_stop (void)
{
	ADCSRA = 0x0;
	ADCSRB = 0x0;
	ADMUX = 0x0;

	msr_state = 0;
}

static void msr_update (void)
{
	float ftmp;
	uint8_t adc_result;

	if (!(msr_state & MSR_RUNNING_bm))
		return;

	if (msr_counter == 0) {
		if (msr_state & MSR_ADC_STARTED_bm) {
			if ((ADCSRA & (1 << ADIF)) != 0) {
				adc_result = ADCH;
				ftmp = adc_result;
				ftmp = ftmp * (5.0f/256.0f);
				fprintf(&usb_stream,
					"\r                       "
					"\r[measure] A1 = %4.2fV   ",
					ftmp);
				msr_counter = MSR_COUNTER_MAX;
				msr_state &= ~(MSR_ADC_STARTED_bm);
			}
		} else {
			ADCSRA |= (1 << ADSC) |
				  (1 << ADIF);
			msr_state |= MSR_ADC_STARTED_bm;
		}
	} else {
		--msr_counter;
	}
}

static void ctrl_monitor (USB_ClassInfo_CDC_Device_t *vdev)
{
	static uint8_t slave_reset = 0;
	static uint8_t rpi_powered = 0;
	int16_t rx_byte;

	if (monitor_refresh) {
		monitor_refresh = 0;
		fprintf(&usb_stream, "\r\nMonitor\r\n=======\r\n"
			"a) Reset slave: %s (A0)\r\n"
			"b) Enter bootloader\r\n"
			"m) Measure mode: %s (A1)\r\n"
			"s) Sleep\r\n"
			"r) Rpi powered %s (D4)\r\n",
			slave_reset ? "enabled" : "disabled",
			msr_is_started() ? "enabled" : "disabled",
			rpi_powered ? "enabled" : "disabled");
	}

	rx_byte = CDC_Device_ReceiveByte(vdev);
	if (rx_byte >= 0) {
		monitor_refresh = 1;
		switch (rx_byte) {
		case 'A':
		case 'a':
			if (slave_reset == 0) {
				slave_reset = 1;
				PORTF |= (1 << 7);
			} else {
				slave_reset = 0;
				PORTF &= ~(1 << 7);
			}
			break;

		case 'B':
		case 'b':
			cli();
			blink(3);
			USB_Disable();
			_delay_ms(2000);
			blink(2);
			enter_bootloader();
			break;

		case 'M':
		case 'm':
			if (msr_is_started()) {
				msr_stop();
			} else {
				msr_start();
			}
			fprintf(&usb_stream,
				"\r                       \r\n");
			break;

		case 'S':
		case 's':
			enter_deep_sleep();
			break;

		case 'R':
		case 'r':
			if (rpi_powered == 0) {
				rpi_powered = 1;
				PORTD |= (1 << 4);
			} else {
				rpi_powered = 0;
				PORTD &= ~(1 << 4);
			}
			break;

		default:
			break;
		}
	}
}

/** Main program entry point. This routine contains the overall program flow, including initial
 *  setup of all components and the main program loop.
 */
int main(void)
{
	SetupHardware();

	RingBuffer_InitBuffer(&USBtoUSART_Buffer, USBtoUSART_Buffer_Data, sizeof(USBtoUSART_Buffer_Data));
	RingBuffer_InitBuffer(&USARTtoUSB_Buffer, USARTtoUSB_Buffer_Data, sizeof(USARTtoUSB_Buffer_Data));

	VirtualSerial_CDC_Interface.State.LineEncoding.BaudRateBPS = 9600; /* Reset variable to some default value */

	CDC_Device_CreateStream(&VirtualSerial_CDC_Interface, &usb_stream);

	LEDs_SetAllLEDs(LEDMASK_USB_NOTREADY);
	GlobalInterruptEnable();

	for (;;) {
		if (VirtualSerial_CDC_Interface.State.LineEncoding.BaudRateBPS == 1200) {
			/* control interface */
			ctrl_monitor(&VirtualSerial_CDC_Interface);
			msr_update();
		} else {
			/* Only try to read in bytes from the CDC interface if the transmit buffer is not full */
			if (!(RingBuffer_IsFull(&USBtoUSART_Buffer))) {
				int16_t ReceivedByte = CDC_Device_ReceiveByte(&VirtualSerial_CDC_Interface);

				/* Store received byte into the USART transmit buffer */
				if (!(ReceivedByte < 0))
					RingBuffer_Insert(&USBtoUSART_Buffer, ReceivedByte);
			}

			uint16_t BufferCount = RingBuffer_GetCount(&USARTtoUSB_Buffer);
			if (BufferCount) {
				Endpoint_SelectEndpoint(VirtualSerial_CDC_Interface.Config.DataINEndpoint.Address);

				/* Check if a packet is already enqueued to the host - if so, we shouldn't try to send more data
				 * until it completes as there is a chance nothing is listening and a lengthy timeout could occur */
				if (Endpoint_IsINReady()) {
					/* Never send more than one bank size less one byte to the host at a time, so that we don't block
					 * while a Zero Length Packet (ZLP) to terminate the transfer is sent if the host isn't listening */
					uint8_t BytesToSend = MIN(BufferCount, (CDC_TXRX_EPSIZE - 1));

					/* Read bytes from the USART receive buffer into the USB IN endpoint */
					while (BytesToSend--) {
						/* Try to send the next byte of data to the host, abort if there is an error without dequeuing */
						if (CDC_Device_SendByte(&VirtualSerial_CDC_Interface,
									RingBuffer_Peek(&USARTtoUSB_Buffer)) != ENDPOINT_READYWAIT_NoError) {
							break;
						}

						/* Dequeue the already sent byte from the buffer now we have confirmed that no transmission error occurred */
						RingBuffer_Remove(&USARTtoUSB_Buffer);
					}
				}
			}

			/* Load the next byte from the USART transmit buffer into the USART */
			if (!(RingBuffer_IsEmpty(&USBtoUSART_Buffer)))
				Serial_SendByte(RingBuffer_Remove(&USBtoUSART_Buffer));
		}

		CDC_Device_USBTask(&VirtualSerial_CDC_Interface);
		USB_USBTask();
	}
}

/** Configures the board hardware and chip peripherals for the demo's functionality. */
void SetupHardware(void)
{
#if (ARCH == ARCH_AVR8)
	/* Disable watchdog if enabled by bootloader/fuses */
	MCUSR &= ~(1 << WDRF);
	wdt_disable();

	/* Disable clock division */
	clock_prescale_set(clock_div_1);
#endif

	DDRF |= (1 << 7);
	DDRF &= ~(1 << 6);
	PORTF &= ~(1 << 7);

	PORTD &= ~(1 << 4);
	DDRD |= (1 << 4);

	/* Hardware Initialization */
	LEDs_Init();
	USB_Init();
}

/** Event handler for the library USB Connection event. */
void EVENT_USB_Device_Connect(void)
{
	LEDs_SetAllLEDs(LEDMASK_USB_ENUMERATING);
}

/** Event handler for the library USB Disconnection event. */
void EVENT_USB_Device_Disconnect(void)
{
	LEDs_SetAllLEDs(LEDMASK_USB_NOTREADY);
}

/** Event handler for the library USB Configuration Changed event. */
void EVENT_USB_Device_ConfigurationChanged(void)
{
	bool ConfigSuccess = true;

	ConfigSuccess &= CDC_Device_ConfigureEndpoints(&VirtualSerial_CDC_Interface);

	LEDs_SetAllLEDs(ConfigSuccess ? LEDMASK_USB_READY : LEDMASK_USB_ERROR);
}

/** Event handler for the library USB Control Request reception event. */
void EVENT_USB_Device_ControlRequest(void)
{
	CDC_Device_ProcessControlRequest(&VirtualSerial_CDC_Interface);
}

//void EVENT_CDC_Device_ControLineStateChanged(USB_ClassInfo_CDC_Device_t* const CDCInterfaceInfo)
//{
//	fprintf(&usb_stream, "H2D: %lx\r\n", (long)CDCInterfaceInfo->State.ControlLineStates.HostToDevice);
//	fprintf(&usb_stream, "D2H: %lx\r\n", (long)CDCInterfaceInfo->State.ControlLineStates.DeviceToHost);
//}

/** ISR to manage the reception of data from the serial port, placing received bytes into a circular buffer
 *  for later transmission to the host.
 */
ISR(USART1_RX_vect, ISR_BLOCK)
{
	uint8_t ReceivedByte = UDR1;

	if (USB_DeviceState == DEVICE_STATE_Configured)
		RingBuffer_Insert(&USARTtoUSB_Buffer, ReceivedByte);
}

/** Event handler for the CDC Class driver Line Encoding Changed event.
 *
 *  \param[in] CDCInterfaceInfo  Pointer to the CDC class interface configuration structure being referenced
 */
void EVENT_CDC_Device_LineEncodingChanged(USB_ClassInfo_CDC_Device_t* const CDCInterfaceInfo)
{
	uint8_t ConfigMask = 0;

	switch (CDCInterfaceInfo->State.LineEncoding.ParityType) {
	case CDC_PARITY_Odd:
		ConfigMask = ((1 << UPM11) | (1 << UPM10));
		break;

	case CDC_PARITY_Even:
		ConfigMask = (1 << UPM11);
		break;
	}

	if (CDCInterfaceInfo->State.LineEncoding.CharFormat == CDC_LINEENCODING_TwoStopBits)
		ConfigMask |= (1 << USBS1);

	switch (CDCInterfaceInfo->State.LineEncoding.DataBits) {
	case 6:
		ConfigMask |= (1 << UCSZ10);
		break;

	case 7:
		ConfigMask |= (1 << UCSZ11);
		break;

	case 8:
		ConfigMask |= ((1 << UCSZ11) | (1 << UCSZ10));
		break;
	}

	/* Must turn off USART before reconfiguring it, otherwise incorrect operation may occur */
	UCSR1B = 0;
	UCSR1A = 0;
	UCSR1C = 0;

	/* Set the new baud rate before configuring the USART */
	UBRR1  = SERIAL_2X_UBBRVAL(CDCInterfaceInfo->State.LineEncoding.BaudRateBPS);

	/* Reconfigure the USART in double speed mode for a wider baud rate range at the expense of accuracy */
	UCSR1C = ConfigMask;
	UCSR1A = (1 << U2X1);
	UCSR1B = ((1 << RXCIE1) | (1 << TXEN1) | (1 << RXEN1));

	if (CDCInterfaceInfo->State.LineEncoding.BaudRateBPS == 1200)
		monitor_refresh = 1;
}

