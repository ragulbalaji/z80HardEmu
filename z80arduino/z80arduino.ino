//#include <Arduino_FreeRTOS.h>

#define PIN_CLOCK 13
#define PIN_WAIT 5
#define PIN_MREQ 6
#define PIN_RD 3
#define PIN_WR 2
#define PIN_RESET 7
#define CLOCK_CYCLE_MS 500

/*TaskHandle_t xClockThreadHandle;
TaskHandle_t xReadThreadHandle;
TaskHandle_t xWriteThreadHandle;*/
int A_pins[24];
int D_pins[24];

bool stopclock = false;

void printf_local(const char *format, ...)
{
  char buf[160];
  va_list ap;
  va_start(ap, format);
  vsnprintf(buf, sizeof(buf), format, ap);
  va_end(ap);
  Serial.print(buf);
}
#define printf printf_local 

void TaskClock(void *pvParameters)  // This is a task.
{
  for (;;)
  {
    digitalWrite(PIN_CLOCK, HIGH);
    //printf("CLOCK HIGH\n");
    //vTaskDelay(CLOCK_CYCLE_MS / 2 / portTICK_PERIOD_MS);
   //printf("RD: %d, WR: %d\n", digitalRead(PIN_RD), digitalRead(PIN_WR));
    digitalWrite(PIN_CLOCK, LOW);
    //printf("CLOCK LOW\n");
    //vTaskDelay(CLOCK_CYCLE_MS/ 2 / portTICK_PERIOD_MS);
   //printf("RD: %d, WR: %d\n", digitalRead(PIN_RD), digitalRead(PIN_WR));
  }
}

unsigned char ROM_data[] = {
  //0x01, 0xff, 0xff, 0xc5, 0xc3, 0x06, 0x00, 0x00, 0x00
  0x31, 0x00, 0xFF, //LD SP, 0xFF00
  0x01, 0x00, 0x00, //LD   BC,0x0
  0x21, 0x00, 0x10, //LD HL, 0x1000
//LOOP:
  0xC5, //PUSH BC
  0x03, //INC BC
  0x7E, //LD   A,(HL)   
  0x3C, //INC   A  
  0x77, //LD   (HL),A 
  0xC3, 0x06, 0x00 //JP LOOP
};

unsigned char RAM_data[1000] = {0};


void TaskRead() {
  while(1){
    //vTaskSuspend(NULL);
    
    //vTaskResume(xClockThreadHandle);
  }
}


int handle_RD() {
    int start = micros();
    digitalWrite(PIN_WAIT, LOW);
    delay(1);
    uint16_t address = 0;
    for(int i = 0;i<15;i++)
    {
      if(digitalRead(A_pins[i]))
      {
        address |= (1<<i);
      }
    }
    uint8_t dataoutput;
    /*if (address < (sizeof(ROM_data)/sizeof(unsigned char)))
    {
      dataoutput = ROM_data[address];
    }
    else if (address >= 0x1000 && address - 0x1000 < (sizeof(RAM_data)/sizeof(unsigned char)))
    {
      dataoutput = RAM_data[address-0x1000];
    }
    else
    {
      dataoutput = 0x00;
    }*/
    
    printf("R,0x%04X\n",address);
    //printf("RD Triggered, RD: %d, WR: %d, MREQ: %d, addr: 0x%04X ", digitalRead(PIN_RD), digitalRead(PIN_WR), digitalRead(PIN_MREQ), address);
    Serial.flush();
    char cmd;
    
    while(!Serial.available());
    cmd = Serial.read();
    if(cmd == 'A')
    {
      reset();
      return 1;
    }
    while(!Serial.available());
    dataoutput = Serial.read();

    
    //dataoutput = 0x00;
    delay(1);
    for(int i = 0;i<8;i++)
    {
      pinMode(D_pins[i], OUTPUT);
      digitalWrite(D_pins[i], (dataoutput>>i) & 1);
    }
    //printf("data: 0x%02X\n", dataoutput);
    delay(1);
    //Serial.flush();
    digitalWrite(PIN_WAIT, HIGH);
    return 0;
}

void TaskWrite() {
  while(1) {
   /// vTaskSuspend(NULL);
    
  }
}
void handle_WR() {
    digitalWrite(PIN_WAIT, LOW);
    uint16_t address = 0;
    for(int i = 0;i<16;i++)
    {
      if(digitalRead(A_pins[i]))
      {
        address |= (1<<i);
      }
    }
    //printf("WR Triggered, RD: %d, WR: %d, MREQ: %d, addr: 0x%04X ", digitalRead(PIN_RD), digitalRead(PIN_WR), digitalRead(PIN_MREQ), address);
    
    uint8_t datainput = 0;
    for(int i = 0;i<8;i++)
    {
      pinMode(D_pins[i], INPUT);
      if(digitalRead(D_pins[i]))
      {
        datainput |= (1<<i);
      }
    }
    /*if (address >= 0x1000 && address - 0x1000 < (sizeof(RAM_data)/sizeof(unsigned char)))
    {
      RAM_data[address-0x1000] = datainput;
    }*/
    //printf("         data: 0x%02X\n", datainput);
    printf("W,0x%04X,0x%02X\n",address,datainput);
    digitalWrite(PIN_WAIT, HIGH);

}

void setup() {
  
  Serial.begin(115200);
  pinMode(PIN_RD, INPUT);
  pinMode(PIN_WR, INPUT);
  pinMode(PIN_MREQ, INPUT);
  pinMode(PIN_WAIT, OUTPUT);
  pinMode(PIN_RESET, OUTPUT);
  digitalWrite(PIN_WAIT, HIGH);
  
  //attachInterrupt(digitalPinToInterrupt(PIN_RD), handle_RD, FALLING);
  //attachInterrupt(digitalPinToInterrupt(PIN_WR), handle_WR, FALLING);
  
  randomSeed(analogRead(0));
  
  
  for(int i = 0;i<8;i++)
  {
    D_pins[i] = i+22;
    pinMode(D_pins[i], INPUT);
  }
  for(int i = 0;i<16;i++)
  {
    A_pins[i] = i+22+8;
    pinMode(A_pins[i], INPUT);
  }
  
  pinMode (PIN_CLOCK, OUTPUT); 
  delay(1000);
  
  /*xTaskCreate(
    TaskClock
    ,  (const portCHAR *)"Clock"   // A name just for humans
    ,  4096  // Stack size
    ,  NULL
    ,  2  // priority
    ,  &xClockThreadHandle );*/

   reset();
}
void reset() {
  //Serial.println("---RESET---");
   digitalWrite(PIN_RESET, LOW);
   for(int i = 0;i<5;i++)
   {
    digitalWrite(PIN_CLOCK, HIGH);
    //printf("CLOCK HIGH\n");
    delay(CLOCK_CYCLE_MS / 2);
   //printf("RD: %d, WR: %d\n", digitalRead(PIN_RD), digitalRead(PIN_WR));
    digitalWrite(PIN_CLOCK, LOW);
    //printf("CLOCK LOW\n");
    delay(CLOCK_CYCLE_MS / 2);
   }
   digitalWrite(PIN_RESET, HIGH);
}
void startexec(){
  
  
}

void stopexec(){
  //vTaskDelete(xClockThreadHandle);
}
void resumeexec(){
  //vTaskResume(xClockThreadHandle);
}

int lastWR = 1;
int lastRD = 1;
void loop() {

    if(Serial.available() > 0)
    {
      char cmd = Serial.read();
      switch(cmd)
      {
        case 'S':
          startexec();
          break;
        case 'E':
          stopexec();
          break;
        case 'R':
          reset();
          break;
      }
    }
    int curRD = digitalRead(PIN_RD);
    if(curRD == 0 && lastRD == 1)
    {
      if(handle_RD())
      {
        return;
      }
    }
    int curWR = digitalRead(PIN_WR);
    if(curWR == 0 && lastWR == 1)
    {
      handle_WR();
    }
    lastRD = curRD;
    lastWR = curWR;
    digitalWrite(PIN_CLOCK, HIGH);
    //printf("CLOCK HIGH\n");
    delay(CLOCK_CYCLE_MS / 2);
   //printf("RD: %d, WR: %d\n", digitalRead(PIN_RD), digitalRead(PIN_WR));
    digitalWrite(PIN_CLOCK, LOW);
    //printf("CLOCK LOW\n");
    delay(CLOCK_CYCLE_MS / 2);
   //printf("RD: %d, WR: %d\n", digitalRead(PIN_RD), digitalRead(PIN_WR));
    /*digitalWrite(PIN_CLOCK, HIGH);
    //printf("CLOCK HIGH\n");
    delay(CLOCK_CYCLE_MS/ 2);
    digitalWrite(PIN_CLOCK, LOW);
    //printf("CLOCK LOW\n");
    delay(CLOCK_CYCLE_MS/ 2);*/
  //printf("Address: 0x%04X, ale: %d, den: %d, dtr: %d\n", address|(address2<<8),ale_cnt,den_cnt,dtr_cnt);
}
