#ifndef SEATALK
  #define SEATALK

  #if (ARDUINO >=100)
    #include "Arduino.h"
  #else
    #include "WProgram.h"
  #endif

  #define MAX_SENTENCE_SIZE 12
  #define MAX_BUFFER_SIZE 60

  class st_protocol {
    
    public:
      st_protocol(uint8_t TX, uint8_t RX);
      void begin();
      bool tx_buffer_empty();
      void send_sentence(uint16_t tx_sentence[]);
      bool rx_buffer_full();
      void load_sentence(uint16_t rx_sentence[]);
      static void OCR2A_ISR();
      static void PCINT2_ISR();

    private:
    
      void frame_from_sentence(uint16_t tx_sentence[]);
      void run_from_frame(uint16_t frame, uint8_t* run_index);
      void tx_buffer_from_run(uint8_t run_length, uint8_t* run_index);

      void sentence_from_frame(uint16_t rx_sentence[], uint8_t run_index_max);
      uint16_t frame_from_run(uint8_t* run_index, uint8_t run_index_max);
      uint8_t run_from_rx_buffer(uint8_t* run_index);

  };
#endif
