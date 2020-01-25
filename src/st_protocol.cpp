#include "st_protocol.h"

#define FCPU 16 // MHz
#define BITS_PER_FRAME 11
#define PRESCALER 256
#define BAUD_RATE 4800
#define MICROSECONDS_PER_TICK 16 // PRESCALER / FCPU
#define MICROSECONDS_PER_BIT 208 // 1 / BAUD_RATE x 10^6
#define MICROSECONDS_PER_FRAME 2291 // MICROSECONDS_PER_BIT x BITS_PER_FRAME
#define TICKS_PER_BIT 13 // MICROSECONDS_PER_BIT / MICROSECONDS_PER_TICK
#define TICKS_PER_FRAME 143 // TICKS_PER_BIT x BITS_PER_FRAME
#define SUP 1 // ISR(TIMER2_COMPA_vect) is scheduled to run SUP x MICROSECONDS_PER_TICK us prior to bus update instant.

volatile uint16_t _tx_buffer[MAX_BUFFER_SIZE], _rx_buffer[MAX_BUFFER_SIZE];
volatile uint8_t  _tx_buffer_pointer_max, _rx_buffer_pointer_max, _tx_pin, _rx_pin;
/*
 * _tx_buffer[0] contains the duration in microseconds of the run of 0s that starts a sentence. 
 * _tx_buffer[_tx_buffer_pointer_max] contains the duration in microseconds of the run of 1s that ends a sentence.
 * _rx_buffer[0] contains the duration in microseconds of the run of 0s that starts a sentence. 
 * _tx_buffer[_rx_buffer_pointer_max] contains the duration in microseconds of the run of 1s that ends a sentence.
 */

 volatile uint8_t* _tx_port;
 volatile uint8_t _rx_port;     // Bit mask for the port that contains _rx_pin (1 = Port B, 2 = Port C, 4 = Port D).
 volatile uint8_t _tx_bit_mask; // Bit mask that selects _tx_pin from the port register.
 volatile uint8_t _rx_bit_mask; // Bit mask that selects _rx_pin from the port register.


ISR(TIMER2_COMPA_vect) {

  st_protocol::OCR2A_ISR();

}


ISR(PCINT2_vect) {

  st_protocol::PCINT2_ISR();
  
}

st_protocol::st_protocol(uint8_t TX, uint8_t RX) {
/*
 * TX and RX must be in the range 0..7 (Port D).
 */
  _tx_pin = TX;
  _rx_pin = RX;
}

void st_protocol::begin() {

/*
 * Set up Timer 2 (8 bit) for ATmega328P.  Set a tick interval of 16 microseconds.
 * A tick is one cycle of the prescaler output.  At 4800 Baud, this gives a jitter
 * value of ~8%, and a max interval of 4 milliseconds or 1.75 frames of bus data.
 */

  noInterrupts();

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
 * Initialize the Seatalk bus input.
 * _rx_port is a pointer to the input register for Port D.
 * _rx_port & _rx_bit_mask evaluates to 0 if the value at _rx_pin is 0.
 * Otherwise it evaluates to some other value.
 */
  pinMode(_rx_pin, INPUT_PULLUP);
  _rx_port = digitalPinToPort(_rx_pin);
  _rx_bit_mask = digitalPinToBitMask(_rx_pin);

  PCICR  |= _rx_port; // Enable pin-change interrupts on Port D.
  PCMSK2 |= _rx_bit_mask; // Enable Port D pin-change interrupts on _rx_pin.

/*
 * Initialize the Seatalk bus output.
 * _tx_port is a pointer to the output port register.
 * *_tx_port |= _tx_bit_mask sets _tx_pin high.
 * *_tx_port &= ~_tx_bit_mask sets _tx_pin low.
 */
 
  pinMode(_tx_pin, OUTPUT);
  _tx_bit_mask = digitalPinToBitMask(_tx_pin);
  _tx_port     = portOutputRegister(digitalPinToPort(_tx_pin)); // Pointer to the transmit port register.
  *_tx_port   |= _tx_bit_mask; // Set the output in the idle state.

  interrupts();

}

//------------------------------

// OBJECTS FOR TRANSMITTING SEATALK DATA

//------------------------------

bool st_protocol::tx_buffer_empty() {

  noInterrupts();
  bool result = (_tx_buffer_pointer_max == 0);
  interrupts();
  
  return result;
  
}

//------------------------------

void st_protocol::send_sentence(uint16_t tx_sentence[]) {
/*
 * If _tx_buffer is empty, check that tx_sentence[] contains a valid Seatalk sentence.
 * Convert each sentence payload, in place, to a frame.
 * Transmit each frame.
 */

  if (!st_protocol::tx_buffer_empty()) { // Last sentence hasn’t been sent yet
    return;
  }
  else {
    uint8_t i = 0;    
    while (tx_sentence[i] != 0xFFFF) {
      if (i == 0 && (0x0100 & tx_sentence[i]) == 0) { // Bit 8 of first payload is 0
        return;
      }
      if (i != 0 && (0x0100 & tx_sentence[i]) != 0) { // Bit 8 of a subsequent payload is 1
        return;
      }
      i++;
    }
    i = 0;
    while (tx_sentence[i] != 0xFFFF) { // Convert payloads to frames by adding start and stop bits.
      tx_sentence[i] = tx_sentence[i] << 1;
      tx_sentence[i] &= 0x03FF;
      tx_sentence[i] |= 0x0400;
      i++;
    }
    frame_from_sentence(tx_sentence);
  }
}

//------------------------------

void st_protocol::frame_from_sentence(uint16_t tx_sentence[]) {
/*
 * Send each sentence frame in turn.
 */
uint8_t run_index, frame_index;

  run_index = 0;
  frame_index = 0;
  while (tx_sentence[frame_index] != 0xFFFF) {
    run_from_frame(tx_sentence[frame_index], &run_index);
    frame_index++;
  }
      
  noInterrupts();
  _tx_buffer_pointer_max = run_index - 1;
  interrupts();
 
}

//------------------------------

void st_protocol::run_from_frame(uint16_t frame, uint8_t* run_index) {
/*
 * Convert each frame to a sequence of runs.  The first run includes the start bit,
 * so it's value is 0.
 */
uint8_t run_length, run_value;

  run_length = 1;
  run_value = frame & 0x0001;
  for (uint8_t i = 1; i < BITS_PER_FRAME; i++) {
    frame = frame >> 1;
    if ((0x0001 & frame) == run_value) {
      run_length++;
    }
    else {
      tx_buffer_from_run(run_length, run_index);
      (*run_index)++;
      run_length = 1;
      run_value ^= 0x0001;
    }
  }
  tx_buffer_from_run(run_length, run_index);
  (*run_index)++;
}

//------------------------------

void st_protocol::tx_buffer_from_run(uint8_t run_length, uint8_t* run_index) {
    
  noInterrupts();
  _tx_buffer[*run_index] = run_length * MICROSECONDS_PER_BIT;
  interrupts();

}

//------------------------------

// OBJECTS FOR RECEIVING SEATALK DATA

//------------------------------

bool st_protocol::rx_buffer_full() {
 
  noInterrupts(); 
  bool result = (_rx_buffer_pointer_max != 0);
  interrupts();
  
  return result;
  
}

//------------------------------

void st_protocol::load_sentence(uint16_t rx_sentence[]) {
/*
 * If _rx_buffer[] is full, fill rx_sentence[] with frames.
 * Check that each frame has a start and a stop bit.
 * Convert each sentence frame, in place, to a payload.
 * Set _rx_buffer_pointer_max = 0 to indicate to ISR(PCINT0_vect) that it can 
 * refill _rx_buffer[].
 */

  noInterrupts();
  uint8_t run_index_max = _rx_buffer_pointer_max;
  interrupts();

  if (run_index_max == 0) { // No data has been received.
    return;
  }
  else { // There is data in _rx_buffer[].   
    sentence_from_frame(rx_sentence, run_index_max);    
    uint8_t i = 0;
    while (rx_sentence[i] != 0xFFFF) {          
      if (((0x0001 & rx_sentence[i]) != 0) || ((0x0400 & rx_sentence[i]) == 0)) { // Start bit is not 0 or stop bit is not 1
        rx_sentence[0] = 0xFFFF;      
        return;
      }
      i++;
    }
    i = 0;
    while (rx_sentence[i] != 0xFFFF) { // Convert frames to payloads
      rx_sentence[i] = rx_sentence[i] >> 1;
      rx_sentence[i] &= 0x01FF;      
      i++;
    }
    
  noInterrupts();
  _rx_buffer_pointer_max = 0;
  interrupts();
  
  }
}

//------------------------------


void st_protocol::sentence_from_frame(uint16_t rx_sentence[], uint8_t run_index_max) {
/*
Populate rx_sentence with a sequence of frames.  Terminate the sentence with the entry 0xFFFF.
*/
uint8_t run_index, frame_index;

  run_index = 0;
  frame_index = 0;
  do {
    rx_sentence[frame_index] = frame_from_run(&run_index, run_index_max);   
    frame_index++;
  }
  while (run_index < run_index_max);
  rx_sentence[frame_index] = 0xFFFF;
}

//------------------------------

uint16_t st_protocol::frame_from_run(uint8_t* run_index, uint8_t run_index_max) {
/*
 * Get the sequence of runs from _rx_buffer[].  Build the corresponding frame, starting with 
 * a start bit (0).
 */
uint16_t frame, run_value;
uint8_t bit_index, run_length;

  bit_index = 0;
  run_value = 0;
  frame = 0;
  do {
    if (*run_index == run_index_max) {
      run_length = BITS_PER_FRAME - bit_index;
    }
    else {
      run_length = run_from_rx_buffer(run_index);
    }
    for (uint8_t i = 0; i < run_length; i++) {
      frame |= run_value;
      frame = frame >> 1; // little-endian.
    }
    run_value ^= 1 << (BITS_PER_FRAME + 1);
    bit_index += run_length;
    (*run_index)++;  
  }
  while (bit_index < BITS_PER_FRAME);
  frame = frame >> 1;
  return frame;
}

//------------------------------

uint8_t st_protocol::run_from_rx_buffer(uint8_t* run_index) {
/*
 * Get a run duration from _rx_buffer and convert it to a run length in bit intervals.
 */
uint16_t this_run;
  
  noInterrupts();
  this_run = _rx_buffer[*run_index];
  interrupts();
 
  if ((this_run % MICROSECONDS_PER_BIT) < (MICROSECONDS_PER_BIT / 2))
    return uint8_t(this_run / MICROSECONDS_PER_BIT);
  else
    return uint8_t(this_run / MICROSECONDS_PER_BIT) + 1;
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
*/

uint32_t next_run_start_time;
uint16_t this_run_duration;
static uint32_t previous_run_start_time = 0;
static uint16_t previous_run_duration = MICROSECONDS_PER_FRAME;
static uint8_t tx_buffer_pointer;

  if (_tx_buffer_pointer_max == 0) { // There is no update available, so set the output HIGH
    *_tx_port |= _tx_bit_mask;       // and schedule ISR(TIMER2_COMPA_vect) to run again
    OCR2A = TICKS_PER_FRAME;         // after a delay equivalent to one frame of Seatalk data.
    tx_buffer_pointer = 0;
  }
  else {                             // There are data available to be sent.
    this_run_duration = _tx_buffer[tx_buffer_pointer];
    next_run_start_time = previous_run_start_time + uint32_t(previous_run_duration);
    while(micros() < next_run_start_time); // Wait here for the start of the run.
    previous_run_start_time = micros();
    if(tx_buffer_pointer & 0x01) // Set the output to the LSB of tx_buffer_pointer.
      *_tx_port |= _tx_bit_mask;
    else
      *_tx_port &= ~_tx_bit_mask;
    previous_run_duration = this_run_duration;
    OCR2A = this_run_duration / MICROSECONDS_PER_TICK - SUP; // Schedule the next interrupt.
    tx_buffer_pointer++;
    if (tx_buffer_pointer == _tx_buffer_pointer_max)
      _tx_buffer_pointer_max = 0;
  }

}

void st_protocol::PCINT2_ISR() {
/*
This ISR fills rx_buffer with the durations, in microseconds, of runs of bits
on the bus.  The 0-index entry contains the duration of the run that starts
with a start bit.  So, it should be a run of 0s.  However, this is not tested here;
it should be tested by the function that processes rx_buffer.

It stops filling the buffer when it encounters a run duration greater than the
frame duration.
 */

uint32_t next_run_start_time; // Start time of the run that is just starting.
uint16_t previous_run_duration; // Duration in microseconds of the run that is just ending.
static uint32_t previous_run_start_time = 0;
static bool capturing = false; // Flag to indicate that run durations are being written to _rx_buffer.
static uint8_t rx_buffer_pointer;

  next_run_start_time = micros();
  previous_run_duration = uint16_t(min(uint32_t(MICROSECONDS_PER_FRAME), (next_run_start_time - previous_run_start_time)));

  if (previous_run_duration == MICROSECONDS_PER_FRAME) { // The previous run is the last run of a sentence.
    if (capturing) { // Data for the sentence just ending has been captured.
      capturing = false;
      _rx_buffer_pointer_max = rx_buffer_pointer;
      _rx_buffer[rx_buffer_pointer] = MICROSECONDS_PER_FRAME;
    }
    else { // We have not been capturing bus data.
      if (_rx_buffer_pointer_max == 0) { // The last buffer has been read, so start capturing.
        capturing = true;
        rx_buffer_pointer = 0;
      }
    }
  }
  else { // The previous run is not the last run of a sentence.
    if (capturing) { // Add run duration to _rx_buffer.  If not capturing, do nothing.
      _rx_buffer[rx_buffer_pointer] = previous_run_duration;
      rx_buffer_pointer++;
    }
  }
  previous_run_start_time = next_run_start_time;
}
