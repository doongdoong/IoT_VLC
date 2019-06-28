#include <TimerOne.h>
#include <util/atomic.h>
#include "ESP8266.h"
#include <SoftwareSerial.h>

#define SSID        "Kim_SiYoung"
#define PASSWORD    "12345678"

SoftwareSerial mySerial(11, 10); /* RX:D11, TX:D10 */
ESP8266 wifi(mySerial);

#define SYMBOL_PERIOD 500 /* Defined a symbol period in us*/
#define WORD_LENGTH 10 /* Each byte is encoded on 10-bit with start, byte, stop */
#define SYNC_SYMBOL 0xD5 /* Synchronization symbol to send after a preamble, before data communication happens */
#define ETX 0x03
#define STX 0x02

//Fast manipulation of LED IO.
//These defines are for a LED connected on D13
/*#define OUT_LED() DDRB |= (1 << 5);
  #define SET_LED() PORTB |= (1 << 5)
  #define CLR_LED() PORTB &= ~(1 << 5)
*/

//These defines are for a RGB led connected to D2, D3, D4
/*#define OUT_LED() DDRD |= ((1 << 2) | (1 << 3) | (1 << 4))
  #define SET_LED() PORTD |= ((1 << 2) | (1 << 3) | (1 << 4))
  #define CLR_LED() PORTD &= ~((1 << 2) | (1 << 3) | (1 << 4))
*/

//These defines are for a single led connected to D2
#define OUT_LED() DDRD |= ((1 << 2))
#define SET_LED() PORTD |= ((1 << 2))
#define CLR_LED() PORTD &= ~((1 << 2))

unsigned char frame_buffer [38] ; //buffer for frame
char frame_index = -1; // index in frame
char frame_size = -1  ; // size of the frame to be sent

//state variables of the manchester encoder
unsigned char bit_counter = 0 ;
unsigned short data_word = 0 ;  //8bit data + start + stop
unsigned char half_bit = 0 ;
unsigned long int manchester_data ;

void to_manchester(unsigned char data, unsigned long int * data_manchester) {
  unsigned int i ;
  (*data_manchester) = 0x02 ; // STOP symbol
  (*data_manchester) = (*data_manchester) << 2 ;
  for (i = 0 ; i < 8; i ++) {
    if (data & 0x80) (*data_manchester) |=  0x02  ; // data LSB first
    else (*data_manchester) |= 0x01 ;
    (*data_manchester) = (*data_manchester) << 2 ;
    data = data << 1 ; // to next bit
  }
  (*data_manchester) |= 0x01 ; //START symbol
}

//emitter interrupt
void emit_half_bit() {
  if (manchester_data & 0x01) {
    SET_LED();
  } else {
    CLR_LED();
  }
  bit_counter -- ;
  manchester_data = (manchester_data >> 1);
  if (bit_counter == 0) {
    //is there still bytes to send in the frame ?
    manchester_data = 0xAAAAAAAA ; // keep sending ones if nothing to send
    if (frame_index >= 0 ) {
      if (frame_index < frame_size) {
        /*Serial.println(frame_index, DEC);
          Serial.println(frame_buffer[frame_index], HEX);*/
        to_manchester(frame_buffer[frame_index], &manchester_data);
        frame_index ++ ;
      } else {
        frame_index = -1 ;
        frame_size = -1 ;
      }
    }
    bit_counter = WORD_LENGTH * 2 ;
    //Serial.println(manchester_data, BIN);
  }
}

void init_frame(unsigned char * frame) {
  memset(frame, 0xAA, 3);
  frame[3] = SYNC_SYMBOL ;
  frame[4] = STX;
  frame_index = -1 ;
  frame_size = -1 ;
}

int create_frame(char * data, int data_size, unsigned char * frame) {
  memcpy(&(frame[5]), data, data_size);
  frame[5 + data_size] = ETX;
  return 1 ;
}

int write(char * data, int data_size) {
  if (frame_index >=  0) return -1 ;
  if (data_size > 32) return -1 ;
  create_frame(data, data_size, frame_buffer);
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    frame_index = 0 ;
    frame_size = data_size + 6 ;
  }
  return 0 ;
}

int transmitter_available() {
  if (frame_index >=  0) return 0 ;
  return 1 ;
}

void init_emitter() {
  manchester_data = 0xFFFFFFFF ;
  bit_counter = WORD_LENGTH * 2 ;
}

// the setup routine runs once when you press reset:
void setup() {
  Serial.begin(115200);
  //tcp setup
  Serial.print("setup begin\r\n");

  Serial.print("FW Version:");
  Serial.println(wifi.getVersion().c_str());

  if (wifi.setOprToStationSoftAP()) {
    Serial.print("to station + softap ok\r\n");
  } else {
    Serial.print("to station + softap err\r\n");
  }

  if (wifi.joinAP(SSID, PASSWORD)) {
    Serial.print("Join AP success\r\n");
    Serial.print("IP: ");
    Serial.println(wifi.getLocalIP().c_str());
  } else {
    Serial.print("Join AP failure\r\n");
  }

  if (wifi.enableMUX()) {
    Serial.print("multiple ok\r\n");
  } else {
    Serial.print("multiple err\r\n");
  }

  if (wifi.startTCPServer(8090)) {
    Serial.print("start tcp server ok\r\n");
  } else {
    Serial.print("start tcp server err\r\n");
  }

  if (wifi.setTCPServerTimeout(10)) {
    Serial.print("set tcp server timout 10 seconds\r\n");
  } else {
    Serial.print("set tcp server timout err\r\n");
  }

  Serial.print("setup end\r\n");


  OUT_LED();
  init_frame(frame_buffer);
  init_emitter();
  Timer1.initialize(SYMBOL_PERIOD); //1200 bauds
  Timer1.attachInterrupt(emit_half_bit);
}


// the loop routine runs over and over again forever:
char * msg = "Hello World" ;
char com_buffer [32] ;
char com_buffer_nb_bytes = 0 ;
uint8_t buffer[32] = {0};
uint8_t mux_id;
uint32_t len = 0;
uint32_t i = 0;
int k = 0;

void loop() {
  if (transmitter_available()) {
    if (len == 0) {
      len = wifi.recv(&mux_id, buffer, sizeof(buffer), 32);
    }

    else {
      if (i != len) {
        char c = (char)buffer[i++];
        com_buffer[com_buffer_nb_bytes] = c ;

        Serial.print(c);
      }

      com_buffer_nb_bytes ++ ;

      if (com_buffer_nb_bytes >= 32 || i == len) {
        if (write(com_buffer, com_buffer_nb_bytes) < 0) {
          Serial.println("Transmitter is busy");
        } else {
          Serial.println();
          i = 0;
          com_buffer_nb_bytes = 0 ;

          k++;

          if (k == 1) { // 같은 데이터를 전송할 반복횟수 조절
            len = 0;
            k = 0;
          }
        }

        delay(10);
      }
    }
  }
}
