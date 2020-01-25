The Seatalk library transmits and receives Seatalk sentences on a Seatalk v1 bus.

Details of the Seatalk protocol are due to Thomas Knauf. See:

 http://www.thomasknauf.de/seatalk.htm
 
The Seatalk bus operates at 4800 Baud. A Seatalk frame is little-endian, with one start bit, one stop bit and no parity bit. The idle state is high. The payload size is 9 bits. Bit 8 in the first payload of a sentence has the value 1. Bits 8 in subsequent payloads have the value 0.

The library uses Timer2 and PCINT0. This leaves Timer0 and Timer1 available to the application. The use of an 8 bit timer results in bit jitter of approximately 8%.

Input and output sentences are arrays of uint16_t of size no more than 11. Sentences are terminated with an entry of 0xFFFF.

The library exposes five methods:

void begin(uint8_t tx, uint8_t rx) bool tx_buffer_empty() void send_sentence(uint16_t sentence[]) bool rx_buffer_full(), and void load_sentence(uint16_t sentence[]);

begin() must be called during setup. tx and rx pins must be distinct and in the range 0..7 inclusive. tx_buffer_empty() returns TRUE if the bus is available to transmit data. send_sentence() transmits a sentence. rx_buffer_full() returns TRUE if a received sentence is available for processing. load_sentence() loads a received data into a sentence.

The library does not test for bus collision. So, a receiving application should perform syntax and sanity checks.

load_sentence() records bus data between idle periods of duration greater than one frame.

The library has only been tested on an Arduino Uno R3 with a 16 MHz system clock.
