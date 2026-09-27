//###### Both Programs are inside here. Set the role by uncomment the SENDER/RECEIVER and comment the other ##########
#include <SPI.h>
#include "printf.h"
#include "RF24.h"
#define CE_PIN 6
#define CSN_PIN 5
#define MISO 4
#define MOSI 3
#define SCK 2
#define LIGHT 25
#define INTV1 500
#define INTV2 50
#define SIZE 32
#define NUM_TASKS (sizeof(tasks) / sizeof(tasks[0])) 
//### Change the role here only #######  
#define SENDER 1
//#define RECEIVER 1  
#if defined(SENDER)
  #define ROLE 1
  bool radioNumber = 1;
#elif defined(RECEIVER)
  #define ROLE 0
  bool radioNumber = 0;
#else
  #error "Define SENDER or RECIEVER before compiling"
#endif
enum State { IDLE, SENT, GOT_REPLY };
static State radio_state = IDLE;
static uint32_t sent_at=0, last_cycle=0,got_it=0,last_delay=0;
uint8_t buffer[SIZE];
uint8_t pipe; //optional
uint32_t bad=0,succsess=0; //optional
bool ledState=0;
RF24 radio(CE_PIN, CSN_PIN);
uint8_t address[][6] = { "1Node", "2Node" };

void radio_task() {
  static uint32_t millis_500_loop = 0;
  uint32_t currentMillis = millis();
  uint32_t now = micros(); 
  switch (radio_state) {
    case IDLE:
      if ((ledState == 1)&&(now - last_cycle >= 50000)) {
        last_cycle = now;
        sent_at = now;
        buffer[0]=0;
        sent_at = micros();
        if(radio.write(&buffer, 1)){                 
          radio_state = SENT;
          succsess++;
        }else{bad++;}
        got_it = micros();
        ledState = 0;
        digitalWrite(LIGHT, ledState);      
        radio_state = SENT;
        break;
      }
      if (currentMillis - millis_500_loop >= INTV1) {
        last_cycle = now;       
        millis_500_loop = currentMillis;
        if (ledState == 0) {
          buffer[0]=1;
          sent_at = micros();       // remove this later
          if(radio.write(&buffer, 1)){                 
            radio_state = SENT;
          }
          got_it = micros();        // remove this later too
          ledState = 1;
          digitalWrite(LIGHT, ledState);         
        }  
        break;
      }
    case SENT:
      last_delay=got_it-sent_at;
      radio_state = IDLE;
      break;
    case GOT_REPLY:
      
      radio_state = IDLE;
      break;
    }
}
void print_task() {   // remove the printout later if you want better performance and replace it with smth usefull. 
  static uint32_t best=9999999,worst=0;
  Serial.print("delay:");
  Serial.print(last_delay);
  Serial.print(" max:");
  if (last_delay>worst){worst=last_delay;}
  Serial.print(worst);
  Serial.print(" min:");
  if ((last_delay<best)&&(last_delay!=0)){best=last_delay;}
  Serial.print(best);
  Serial.print(" lost:");
  Serial.print(bad);
  Serial.print(" succsessrate:");
  if((bad==0)||(succsess==0)){Serial.println(succsess);
  }else{Serial.println(succsess/bad);}
}  
struct Task {
    void (*fn)(void);
    uint32_t period_us;
    uint32_t last_run;
};
Task tasks[] = {
    { radio_task,    0,      0 },   // 0 = "run every iteration" (FIFO-driven)
    { print_task,  500000,   0 },   // 500ms
    //{ led_task,    50000,     0 },   // 50ms
    //{ imu_task,     10000,   0 },   // 100 Hz
    //{ serial_task,   5000,   0 },   // 200 Hz
    // add more if you want, keep timing in mind
};
void setup() {
  pinMode(LIGHT, OUTPUT);
  digitalWrite(LIGHT, 1);
  Serial.begin(115200);
  SPI.setSCK(SCK);
  SPI.setMOSI(MOSI);
  SPI.setMISO(MISO);
  SPI.begin(5000000);  
  if (!radio.begin()) {
    Serial.println(F("radio hardware is not responding!!"));
    while (1) {/*hold in infinite loop*/}  
  }
  radio.setDataRate(RF24_2MBPS);
  radio.setPALevel(RF24_PA_MAX);  // RF24_PA_MAX is default.
  radio.setAddressWidth(4);
  //radio.enableDynamicPayloads();  // ACK payloads are dynamically sized
  //radio.enableAckPayload();       
  radio.setAutoAck(true);
  radio.setRetries(5, 15);
  radio.openWritingPipe(address[radioNumber]);       
  radio.openReadingPipe(1, address[!radioNumber]);
  radio.stopListening();  
  if (!ROLE) {radio.startListening();}
  // For debugging info
  // printf_begin();             // needed only once for printing details
  // radio.printDetails();       // (smaller) function that prints raw register values
  // radio.printPrettyDetails(); // (larger) function that prints human readable data
  digitalWrite(LIGHT, 0);
}
void loop() {  
  if (ROLE) {           // SENDER
    uint32_t currentMicros = micros();
    for (int i = 0; i < NUM_TASKS; i++) {
      if (tasks[i].period_us == 0 || currentMicros - tasks[i].last_run >= tasks[i].period_us) {
        tasks[i].fn();
        tasks[i].last_run = currentMicros;
      }
    }  
  } else {         // RECEIVER
    static uint32_t millis_500_loop = 0;
    uint32_t currentMillis = millis();
    static uint32_t rx_count = 0;
    uint8_t pipe;
    if (radio.available(&pipe)) {
      rx_count++;  
      radio.read(&buffer, 1);  
      if(buffer[0]==0){
        if (ledState == 1) {
          ledState = 0;
          digitalWrite(LIGHT, ledState);
        }
      }else if(buffer[0]==1){
        if (ledState == 0) {
          ledState = 1;
          digitalWrite(LIGHT, ledState);
        }
      }
    }
    //#### uncomment the block for a printout ######### 
    /*if(currentMillis-millis_500_loop>=500){
      millis_500_loop=currentMillis;
      Serial.println(rx_count);
    }*/
  }  
}
