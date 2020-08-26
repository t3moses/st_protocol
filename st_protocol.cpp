/* 
This is the driver for the Real wind display v02 hardware (see "Real wind display v02.sch").
It uses pin A5 for both input and output Seatalk data (not simultaneously).
The data direction is set by a jumper on the circuit board and by the software.
The hardware inverts data in both the send and receive directions.
 */
 
#include <st_protocol.h>

#define FCPU 16 // MHz
#define BITS_PER_FRAME 11
#define PRESCALER 256
#define BAUD_RATE 4800
#define LOG_MICROSECONDS_PER_TICK 4 // Base 2 log of PRESCALER / FCPU
#define MICROSECONDS_PER_BIT 208 // 1 / BAUD_RATE x 10^6
#define MICROSECONDS_PER_FRAME 2291 // MICROSECONDS_PER_BIT x BITS_PER_FRAME
#define TICKS_PER_BIT 13 // MICROSECONDS_PER_BIT / MICROSECONDS_PER_TICK
#define TICKS_PER_FRAME 143 // TICKS_PER_BIT x BITS_PER_FRAME
#define SUP 1 // ISR(TIMER2_COMPA_vect) is scheduled to run SUP x MICROSECONDS_PER_TICK microseconds prior to bus update instant.

volatile uint16_t _tx_buffer[SUP_BUFFER_SIZE], _rx_buffer[SUP_BUFFER_SIZE];
volatile uint8_t  _tx_buffer_pointer_max, _rx_buffer_pointer_max;

/*
 * _tx_buffer[0] contains the duration in microseconds of the run of 0s that starts a datagram.
 * _tx_buffer[_tx_buffer_pointer_max] contains the duration in microseconds of the run of 1s that ends a datagram.
 * _rx_buffer[0] contains the duration in microseconds of the run of 0s that starts a datagram.
 * _rx_buffer[_rx_buffer_pointer_max] contains the duration in microseconds of the run of 1s that ends a datagram.
 */

 volatile uint8_t* _tx_port;
 volatile uint8_t _rx_port;     // Bit mask for the port that contains _rx_pin (1 = Port B, 2 = Port C, 4 = Port D).
 volatile uint8_t _tx_bit_mask; // Bit mask that selects _tx_pin from the port register.
 volatile uint8_t _rx_bit_mask; // Bit mask that selects _rx_pin from the port register.
 volatile bool _sender;

ISR(TIMER2_COMPA_vect) {

  st_protocol::OCR2A_ISR();

}

ISR(PCINT1_vect) { // The data are on Port C.

  st_protocol::PCINT_ISR();
  
}


st_protocol::st_protocol( bool sender ) {

    _sender = sender;
    
}

void st_protocol::begin() {

  noInterrupts();
    
  if ( _sender ) {
        
  /*
   * Set up Timer 2 (8 bit for ATmega328P) to schedule output updates when operating as a Seatalk sender end-point.
   * Set a tick interval of 16 microseconds.  A tick is one cycle of the prescaler output.
   * At 4800 Baud, this gives a jitter value of ~8%, and a max interval of 4 milliseconds or 1.75 frames of bus data.
   */
    
    TCCR2A = 0;                                  // Start by setting all ...
    TCCR2B = 0;                                  // ... control register bits to 0.
    TIMSK2 = 0;
    TCNT2 = 0;                                   // reset Timer 2.
    OCR2A = TICKS_PER_FRAME;                     // Intial output compare register A value

    TCCR2A = (TCCR2A & 0b11111100) | 0b00000010; // Set bit 1 (WGM21), reset bit 0 (WGM20) and ...
    TCCR2B = (TCCR2B & 0b11110111) | 0b00000000; // ... reset bit 3 (WGM22) for Clear Timer
                                               // on Compare Match (CTC) mode.
    TCCR2B = (TCCR2B & 0b11111000) | 0b00000110; // Set bits 2..0 (CS22 ..CS20) to 110
                                               // for a prescaler value of 256.
    TIMSK2 = (TIMSK2 & 0b11111101) | 0b00000010; // Set bit 1 (OCIE2A) to call
                                               // ISR(TIMER2_COMPA_vect) when
                                               // TCNT2 equals OCR2A.
    /*
     * Initialize the Seatalk bus output when operating as a Seatalk sender.
     * _tx_port is a pointer to the output port register (Port C).
     * _tx_bit_mask is a bit-mask for the output pin (Pin A5).
     * *_tx_port |= _tx_bit_mask sets the output pin HIGH.
     * *_tx_port &= ~_tx_bit_mask sets the output pin LOW.
     * Remeber that the hardware interface introduces an inversion.
     */

    _tx_port     = 0x02; // Port C
    _tx_bit_mask = 0x20; // Pin A5
    *_tx_port &= ~_tx_bit_mask; // Set the output in the idle state (0 because the interface inverts the signal).
    _tx_buffer_pointer_max = 0;
        
   }
        
  else {

   /*
    * Initialize the Seatalk bus input when operating as a Seatalk receiver.
    * _rx_port is a pointer to the input register for Port C.
    * _rx_bit_mask  is a bit-mask for the input pin (Pin A5).
    * _rx_port & _rx_bit_mask evaluates to 0 if the value at the input is 0.
    * Otherwise it evaluates to some other value.
    */

     _rx_port     = 0x02; // Port C
     _rx_bit_mask = 0x20; // Pin A5

     PCICR = 0;
     PCICR |= _rx_port;
     PCMSK1 |= _rx_bit_mask;
     _rx_buffer_pointer_max = 0;

  }
    
  interrupts();

}

//------------------------------

// CLASSES FOR TRANSMITTING SEATALK DATA

//------------------------------

bool st_protocol::tx_buffer_empty() {

  noInterrupts();
  bool result = (_tx_buffer_pointer_max == 0);
  interrupts();
  
  return result;
  
}

//------------------------------

void st_protocol::send_datagram( datagram_t tx_datagram ) {
    
/*
 * Check that tx_datagram[] contains a valid Seatalk datagram.
 * Convert each datagram payload, in place, to a frame.
 * Transmit each frame.
 */

  uint8_t i = 0;
  while ( tx_datagram[i] != 0xFFFF ) {
    if ( i == 0 && ( 0x0100 & tx_datagram[i] ) == 0 ) { // Bit 8 of first payload is 0
      return;
    }
    if ( i != 0 && ( 0x0100 & tx_datagram[i] ) != 0 ) { // Bit 8 of a subsequent payload is 1
      return;
    }
    i++;
  }
  i = 0;
  while ( tx_datagram[i] != 0xFFFF ) { // Convert payloads to frames by adding start and stop bits.
    tx_datagram[i] = tx_datagram[i] << 1;
    tx_datagram[i] &= 0x03FF;
    tx_datagram[i] |= 0x0400;
    i++;
  }
  frame_from_datagram( tx_datagram );

}

//------------------------------

void st_protocol::frame_from_datagram( datagram_t tx_datagram ) {
    
/*
 * Send each datagram frame in turn.
 */
    
uint8_t run_index, frame_index;

  run_index = 0;
  frame_index = 0;
  while ( tx_datagram[frame_index] != 0xFFFF ) {
    run_from_frame( tx_datagram[frame_index], &run_index );
    frame_index++;
  }

  noInterrupts();
  _tx_buffer_pointer_max = run_index - 1;
  interrupts();

}

//------------------------------

void st_protocol::run_from_frame( uint16_t frame, uint8_t* run_index ) {
    
/*
 * Convert each frame to a sequence of runs.  The first run includes the start bit,
 * so its value is 0.
 */
    
uint8_t run_length, run_value;

  run_length = 1;
  run_value = frame & 0x0001;
  for ( uint8_t i = 1; i < BITS_PER_FRAME; i++ ) {
    frame = frame >> 1;
    if (( 0x0001 & frame ) == run_value ) {
      run_length++;
    }
    else {
      tx_buffer_from_run(run_length, run_index);
      ( *run_index )++;
      run_length = 1;
      run_value ^= 0x0001;
    }
  }
  tx_buffer_from_run( run_length, run_index );
  ( *run_index )++;
}

//------------------------------

void st_protocol::tx_buffer_from_run( uint8_t run_length, uint8_t* run_index ) {
    
  noInterrupts();
  _tx_buffer[*run_index] = run_length * MICROSECONDS_PER_BIT;
  interrupts();

}

//------------------------------

// CLASSES FOR RECEIVING SEATALK DATA

//------------------------------

bool st_protocol::rx_buffer_full() {
 
  noInterrupts(); 
  bool result = ( _rx_buffer_pointer_max != 0 );
  interrupts();
  
  return result;
  
}

//------------------------------

void st_protocol::load_datagram( datagram_t rx_datagram ) {
    
/*
 * If _rx_buffer[] is full, fill rx_datagram[] with frames.
 * Check that each frame has a start and a stop bit.
 * Convert each datagram frame, in place, to a payload.
 * Set _rx_buffer_pointer_max = 0 to indicate to ISR(PCINT0_vect) that it can 
 * refill _rx_buffer[].
 */
    
uint8_t i;
bool corrupted;
    
  noInterrupts();
  uint8_t run_index_max = _rx_buffer_pointer_max;
  interrupts();

  if ( run_index_max == 0 ) { // No data has been received.
    return;
  }
  else { // There is data in _rx_buffer[].   
    datagram_from_frame( rx_datagram, run_index_max );
    corrupted = false;
    i = 0;
    while ( !corrupted && ( rx_datagram[i] != 0xFFFF )) {
      if ((( 0x0001 & rx_datagram[i] ) != 0 ) || (( 0x0400 & rx_datagram[i] ) == 0 )) { // Start bit is not 0 or stop bit is 0.
        corrupted = true;
        rx_datagram[0] = 0xFFFF;
      }
      i++;
    }
    if( !corrupted ) { // Convert frames to payloads.
      i = 0;
      while ( rx_datagram[i] != 0xFFFF ) {
        rx_datagram[i] = rx_datagram[i] >> 1;
        rx_datagram[i] &= 0x01FF;
        i++;
      }
    }
    
  noInterrupts();
  _rx_buffer_pointer_max = 0;
  interrupts();
  
  }
}

//------------------------------


void st_protocol::datagram_from_frame( datagram_t rx_datagram, uint8_t run_index_max ) {
    
/*
Populate rx_datagram with a sequence of frames.  Terminate the datagram with the entry 0xFFFF.
*/

uint8_t run_index, frame_index;

  run_index = 0;
  frame_index = 0;
  do {
    rx_datagram[ frame_index ] = frame_from_run( &run_index, run_index_max );
    frame_index++;
  }
  while (( run_index < run_index_max ) && ( frame_index < ( SUP_DATAGRAM_SIZE - 1 )));

  rx_datagram[ frame_index ] = 0xFFFF;
}

//------------------------------

uint16_t st_protocol::frame_from_run( uint8_t* run_index, uint8_t run_index_max ) {
    
/*
 * Get the sequence of runs from _rx_buffer[] to build one frame.
 * Note: each frame starts with a 0.
 * Update *run_index.  *run_index is the index into _rx_buffer[].
 * Data on the wite are little-emdian.  But, frames are big-endian.
 */

uint16_t frame, run_value;
uint8_t bit_index, run_length;

  bit_index = 0; // The current position in the frame.
  run_value = 0; // The bit value of the run, in bit position BITS_PER_FRAME + 1.
  frame = 0; // Initialize the frame.
  do {
    
    // Get the length of the next run, bearing in mind that the last run of a frame,
    // which includes the stop bit, may also include some idle bits.
      
    run_length = min( run_from_rx_buffer( run_index ), BITS_PER_FRAME - bit_index );
      
    for ( uint8_t i = 0; i < run_length; i++ ) {
        
    // Add to the frame a run with a value of run_value and a length of run_length.
        
      frame |= run_value;
      frame = frame >> 1; // little-endian.
    }
    run_value ^= ( 1 << ( BITS_PER_FRAME + 1 )); // Invert run_value.
    bit_index += run_length;
    ( *run_index )++;
  }
  while ( bit_index < BITS_PER_FRAME );

  frame = frame >> 1;
  return frame;
}

//------------------------------

uint8_t st_protocol::run_from_rx_buffer( uint8_t* run_index ) {
    
/*
 * Get a run duration from _rx_buffer and convert it to a run length in bit intervals.
 */

uint16_t this_run;
  
  noInterrupts();
  this_run = _rx_buffer[*run_index];
  interrupts();
 
  if (( this_run % MICROSECONDS_PER_BIT ) < ( MICROSECONDS_PER_BIT / 2 ))
    return uint8_t( this_run / MICROSECONDS_PER_BIT );
  else
    return uint8_t( this_run / MICROSECONDS_PER_BIT ) + 1;
}

//------------------------------

// INTERRUPT SERVICE ROUTINES

//------------------------------

void st_protocol::OCR2A_ISR() {
    
/*
Update the Seatalk bus based on the contents of _tx_buffer.  If _tx_buffer is empty,
then set the output high and schedule the ISR to run again after one frame interval.
If _tx_buffer contains data for a run of 1s or 0s, then update the output at the required
run-start time and schedule the ISR to run again when the run is set to end.

If no other ISR is currently running when ISR(TIMER2_COMPA_vect) is triggered, then it may be
called SUP TIMER2 ticks before the output has to be updated.  So, it may be necessary to
block while waiting.  However, in case another interrupt is running when ISR(TIMER2_COMPA_vect)
is called, then it may be triggered less than SUP TIMER2 ticks before the output has to be updated.
 
ts.isTouching() blocks for 430 microseconds.  ts.getPoints() blocks for 720 microseconds
This ISR blocks for between 0.43 and 1.15 milliseconds.
 
TIMER2_COMPA_vect runs with priority 8.

 */

static uint8_t tx_buffer_pointer;
static uint32_t this_run_start_time = MICROSECONDS_PER_FRAME;
static uint16_t this_run_duration = MICROSECONDS_PER_FRAME;

    
  if ( _tx_buffer_pointer_max == 0 ) { // There is no update available, so ...
      
    *_tx_port &= ~_tx_bit_mask; // Set the output in the idle state, and ...
    OCR2A = TICKS_PER_FRAME;    // schedule ISR(TIMER2_COMPA_vect) to run again after a delay equivalent to one frame of Seatalk data.
    tx_buffer_pointer = 0;

  }

  else {  // There are data available to be sent.

    if( tx_buffer_pointer & 0x01 ) // Set the output to the inverse of the LSB of tx_buffer_pointer.
      *_tx_port &= ~_tx_bit_mask;
    else
      *_tx_port |= _tx_bit_mask;

    this_run_duration = _tx_buffer[tx_buffer_pointer]; // Get the duration of the upcoming run.
    OCR2A = uint8_t(( this_run_duration >> LOG_MICROSECONDS_PER_TICK ) - SUP ); // Schedule the next interrupt.
    this_run_start_time = this_run_start_time + uint32_t( this_run_duration ); // Get the start time of the upcoming run.
      
    tx_buffer_pointer++;
    if ( tx_buffer_pointer >= _tx_buffer_pointer_max ) _tx_buffer_pointer_max = 0;
         
    }

  }

//------------------------------

void st_protocol::PCINT_ISR() {
    
/*
This ISR fills _rx_buffer with the durations, in microseconds, of runs of bits
on the bus.  The 0-index entry contains the duration of the run that starts
with a start bit.  So, it should be a run of 0s.  However, this is not tested here;
it should be tested by the function that processes _rx_buffer.

It stops filling _rx_buffer when it encounters a run duration greater than
MICROSECONDS_PER_FRAME or when _rx_buffer is full.  In the case wherea a run duration
greater than MICROSECONDS_PER_FRAME is encountered, the last entry in _rx_buffer
is set to MICROSECONDS_PER_FRAME.  In the case where _rx_buffer overflows, all runs
that arrive before one whose duration is greater than MICROSECONDS_PER_FRAME are lost,
but the last entry in _rx_buffer is still set to MICROSECONDS_PER_FRAME.
 */

uint32_t next_run_start_time; // Start time in midroseconds of the run that is just starting.
uint16_t previous_run_duration; // Duration in microseconds of the run that is just ending.
static uint32_t previous_run_start_time = 0;
static bool capturing = false; // Flag to indicate that run durations are being written to _rx_buffer.
static uint8_t rx_buffer_pointer;

  next_run_start_time = micros();
  previous_run_duration = uint16_t( min( uint32_t( MICROSECONDS_PER_FRAME ), ( next_run_start_time - previous_run_start_time )));

  if ( previous_run_duration == MICROSECONDS_PER_FRAME ) { // The previous run is the last run of a datagram.
    if ( capturing ) { // Data for the datagram just ending has been captured.
      capturing = false;
      _rx_buffer[ rx_buffer_pointer ] = MICROSECONDS_PER_FRAME; // Add the terminator to the buffer.
      _rx_buffer_pointer_max = rx_buffer_pointer; // Record the length of the buffer.
    }
    else { // We have not been capturing data.
      if ( _rx_buffer_pointer_max == 0 ) { // The last datagram has been processed, so start capturing.
        capturing = true;
        rx_buffer_pointer = 0;
      }
      else {} // The last datagram has not yet been loaded, so wait for the next datagram to start.
    }
  }
  else { // The previous run is not the last run of a datagram.
    if ( capturing ) { // Add run duration to _rx_buffer.
      _rx_buffer[ rx_buffer_pointer ] = previous_run_duration;
      if( rx_buffer_pointer < ( SUP_BUFFER_SIZE - 1 )) rx_buffer_pointer++;
      else { // The buffer is full, so truncate the input.
        capturing = false;
        _rx_buffer[ rx_buffer_pointer ] = MICROSECONDS_PER_FRAME; // Add the terminator to the buffer.
        _rx_buffer_pointer_max = rx_buffer_pointer; // Record the length of the buffer.
      }
    }
    else {} // We are skipping this datagram, because the last datagram has not yet been processed.
  }
  previous_run_start_time = next_run_start_time;

}
