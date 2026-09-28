//###### Both Programs are inside here. Set the role by uncomment the #define SENDER/#define RECEIVER and comment the other ##########
#include <SPI.h>
#include "printf.h"
#include "RF24.h"
#define SCK 2
#define MOSI 3
#define MISO 4
#define CSN_PIN 5
#define CE_PIN 6
#define LIGHT 25
#define INTV1 500
#define INTV2 50000UL
#define SIZE 32
#define NUM_TASKS (sizeof(tasks) / sizeof(tasks[0]))   
//### Change the role here only ##### 
#define SENDER 1                  //#
//#define RECEIVER 1              //#
//###################################  
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

struct Task {
    void (*fn)(void);
    uint32_t period_us;
    uint32_t last_run;
};
uint8_t buffer[SIZE];
uint8_t received[SIZE];
uint8_t pipe; //optional
uint8_t test;
uint32_t bad=0,succsess=0; //optional
bool ledState=0;
RF24 radio(CE_PIN, CSN_PIN);
uint8_t address[][6] = { "1Nod", "2Nod" };

void radio_task() {
  static uint32_t millis_500_loop = 0;
  uint32_t currentMillis = millis();
  uint32_t now = micros(); 
  switch (radio_state) {
    case IDLE:
      if ((now - last_cycle >= INTV2)&&(ledState == 1)) {
        last_cycle = now;
        sent_at = now;
        ledState = 0;
        digitalWrite(LIGHT, ledState);
        buffer[0]=0;       
        radio.stopListening();
        radio.write(&buffer, 2);
        radio.startListening(); 
        test=radio.available(&pipe);       
        radio_state = SENT;
      }
      if (currentMillis - millis_500_loop >= INTV1) {
        sent_at = now; 
        last_cycle = now;       
        millis_500_loop = currentMillis;
        if (ledState == 0) {
          ledState = 1;
          digitalWrite(LIGHT, ledState);
          buffer[0]=1;
        }  
        radio.stopListening();
        radio.write(&buffer, 2);
        radio.startListening(); 
        test=radio.available(&pipe);
        radio_state = SENT;    
      }
      break;
    case SENT:
      if (test || radio.available(&pipe)) {
        uint8_t len = radio.getDynamicPayloadSize();
        radio.read(&received, len);
        got_it=now;
        last_delay=got_it-sent_at;       
        radio_state = IDLE;
        succsess++;
      } else if (now - sent_at > 5000) {radio_state = IDLE;bad++;}
      break;
    case GOT_REPLY:
      //do something with the received message
      radio_state = IDLE;
      break;
    }
}
void print_task() {// remove the printout later if you want better performance and replace it with smth usefull. 
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
  Serial.print(" failrate:");
  if((succsess==0)&&(bad==0)){Serial.print(100);
  }else if((succsess!=0)&&(bad==0)){Serial.print(0);
  }else if((succsess==0)&&(bad!=0)){Serial.print(100);
  }else{Serial.print(((float)bad/(float)succsess)*100);}
  Serial.print(" mail:");
  //Serial.print(received[0]);
  Serial.println(received[1]);
}  
Task tasks[] = {
    { radio_task,    0,      0 },   // 0 = "run every iteration" (FIFO-driven)
    { print_task,  950000,   1000 },   // 500000us
    //{ led_task,    50000,     0 },   // 50ms
    //{ imu_task,     10000,   0 },   // 100 Hz
    //{ serial_task,   5000,   0 },   // 200 Hz
    // add more if you want, keep timing in mind
};
void setup() {
  pinMode(LIGHT, OUTPUT);
  digitalWrite(LIGHT, 1);
  Serial.begin(115200);
  buffer[1]=2;
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
  radio.enableDynamicPayloads();  // ACK payloads are dynamically sized
  //radio.enableAckPayload();       
  //radio.stopListening(address[radioNumber]);  // put radio in TX mode
  radio.openWritingPipe(address[radioNumber]);
  radio.openReadingPipe(1, address[!radioNumber]);  // using pipe 1
  if (ROLE) {radio.stopListening();  
  }else {radio.startListening();}
  // For debugging info
   printf_begin();             // needed only once for printing details
  // radio.printDetails();       // (smaller) function that prints raw register values
  // radio.printPrettyDetails(); // (larger) function that prints human readable data
  digitalWrite(LIGHT, 0);
}
void loop() {  
  if (ROLE) {           // SENDER    
    unsigned long currentMicros = micros();
    for (int i = 0; i < NUM_TASKS; i++) {
      if (tasks[i].period_us == 0 || currentMicros - tasks[i].last_run >= tasks[i].period_us) {
        tasks[i].fn();
        tasks[i].last_run = currentMicros;
      }
    }   
  } else {         // RECEIVER
    uint8_t pipe;
    static uint32_t millis_500_loop = 0;
    uint32_t currentMillis = millis();
    static uint32_t rx_count = 0;
    if (radio.available(&pipe)) {  // is there a payload? get the pipe number that received it 
      uint8_t len = radio.getDynamicPayloadSize();
      radio.read(&received, len);  // get incoming payload
      //rx_count++;
      //Serial.println(buffer[0]);
      radio.stopListening();  // put in TX mode
      radio.writeFast(&received, len);  // load response to TX FIFO
      bool report = radio.txStandBy(300);
      radio.startListening();
      if(received[0]==0){
        if (ledState == 1) {
          ledState = 0;
          digitalWrite(LIGHT, ledState);
        }
      }else if(received[0]==1){
        if (ledState == 0) {
          ledState = 1;
          digitalWrite(LIGHT, ledState);
        }
      }      
    }
    /*if(currentMillis-millis_500_loop>=500){
      millis_500_loop=currentMillis;
      Serial.println(rx_count);
    }*/
  }  
}
