#ifndef SEATALK_PROTOCOL
  #define SEATALK_PROTOCOL

  #if (ARDUINO >= 100)
    #include "Arduino.h"
  #else
    #include "WProgram.h"
  #endif

  #define SENDER true
  #define RECEIVER false
  #define SUP_BUFFER_SIZE 64
  #define SUP_DATAGRAM_SIZE 12

typedef uint16_t datagram_t[ SUP_DATAGRAM_SIZE ];

  class st_protocol {
    
    public:
      
      st_protocol( bool sender );
      void begin();
      
      bool tx_buffer_empty();
      void send_datagram(datagram_t tx_datagram);
      
      bool rx_buffer_full();
      void load_datagram(datagram_t rx_datagram);
      
      static void OCR2A_ISR();
      static void PCINT_ISR();

    private:
    
      void frame_from_datagram(datagram_t tx_datagram);
      void run_from_frame(uint16_t frame, uint8_t* run_index);
      void tx_buffer_from_run(uint8_t run_length, uint8_t* run_index);
      
      void datagram_from_frame(datagram_t rx_datagram, uint8_t run_index_max);
      uint16_t frame_from_run(uint8_t* run_index, uint8_t run_index_max);
      uint8_t run_from_rx_buffer(uint8_t* run_index);

  };
#endif
