Example Summary
---------------
This example provides basics for demonstrating UART usage, LED control,
buttons, and Sub1GHz TX/RX.

Mode of operation (TX or RX) and tx-power (20dBm or 14dBm) is selected
automatically based on the state of DIO_14 pins (accessible as
pushbuttons on some hardware such as the GW16122 and the TI Launchpads):
 - DIO_14/BTN2 or DIO_26:
   - if either of these are logic level low on power-up (ie button pressed
     or DIO grounded) TX mode is automatically selected.
 - DIO_15/BTN1 or DIO_21:
   - if either of these are logic level low on power-up (ie button pressed
     or DIO grounded) 20dBm TX power is used for TX.
 - if these pins are logic level high the user will be prompted for
   configuration via UART for 5 seconds before defaulting to RX and 20dBm.

smartrf_settings.c contains structures obtained from TI SmartRF Studio
that by default configure the RF for:
 - 915MHz
 - 25kHz deviation
 - 20byte packets
 - 14dBm or 20dBm depending on the power-up button/DIO state


Example Usage
-------------
The user will require two boards, one running in TX mode (Board_TX) and the
other in RX mode (Board_RX).

Power up Board_RX with no buttons held down. The GRN LED will light 
solid indicating the board is in RX mode. When a packet is received the
RED led will toggle between on and off.

Power up Board_TX with BTN2 held down. Observe both GRN and RED LEDs toggling
once a second indicating a packet has been transmitted at 20dBm. If BTN1
was held during power-up only the GRN LED will toggle indicating a packet
has been transmitted at 14dBm. 

