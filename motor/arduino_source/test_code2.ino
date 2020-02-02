#include "sam.h"

typedef unsigned long u32;
typedef unsigned int u16;
typedef unsigned char u8;
typedef long i32;
typedef int i16;
typedef char i8;

#define _BV(x) (1u << (x))
#define AIN1 29
#define AIN2 21
#define BIN1 22
#define BIN2 23

#define CIN1 24
#define CIN2 25
#define DIN1 26
#define DIN2 28

//#define PWMC1 23
//#define PWMC2 24
//#define MOTOR_A_ON  PIOD->PIO_SODR = _BV(PWMC1)
//#define MOTOR_A_OFF PIOD->PIO_CODR = _BV(PWMC1)

int temp;

void init_motor(){
    PIOC->PIO_PER = _BV(AIN1) | _BV(AIN2) | _BV(BIN1) | _BV(BIN2) | _BV(CIN1) | _BV(CIN2) | _BV(DIN1) | _BV(DIN2);
    PIOC->PIO_OER = _BV(AIN1) | _BV(AIN2) | _BV(BIN1) | _BV(BIN2) | _BV(CIN1) | _BV(CIN2) | _BV(DIN1) | _BV(DIN2);
    //PIOC->PIO_PER = _BV(PWMC1);
}

void motor_set_dir(u32 data){
    if(1 == data){              // front
        PIOC->PIO_SODR = _BV(AIN1) | _BV(BIN1) | _BV(CIN1) | _BV(DIN1);
        PIOC->PIO_CODR = _BV(AIN2) | _BV(BIN2) | _BV(CIN2) | _BV(DIN2);
    }
    else if(2 == data){         // back
        PIOC->PIO_SODR = _BV(AIN2) | _BV(BIN2) | _BV(CIN2) | _BV(DIN2);
        PIOC->PIO_CODR = _BV(AIN1) | _BV(BIN1) | _BV(CIN1) | _BV(DIN1);
    }
    else if(3 == data){         // left
        PIOC->PIO_SODR = _BV(AIN2) | _BV(BIN1) | _BV(CIN2) | _BV(DIN1);
        PIOC->PIO_CODR = _BV(AIN1) | _BV(BIN2) | _BV(CIN1) | _BV(DIN2);
    }
    else if(4 == data){         // right
        PIOC->PIO_SODR = _BV(AIN1) | _BV(BIN2) | _BV(CIN1) | _BV(DIN2);
        PIOC->PIO_CODR = _BV(AIN2) | _BV(BIN1) | _BV(CIN2) | _BV(DIN1);
    }
    else{                       // stop
        PIOC->PIO_CODR = _BV(AIN1) | _BV(AIN2) | _BV(BIN1) | _BV(BIN2) | _BV(CIN1) | _BV(CIN2) | _BV(DIN1) | _BV(DIN2);
    }
}
void setup() {
    SystemInit();
    init_motor();
    WDT->WDT_MR = 0x3fff8fff;
    PMC->PMC_PCER0 = _BV(ID_PIOC);
    Serial.begin(115200);
    Serial1.begin(115200);
}

void loop() {
    if(Serial1.available()){
        temp = Serial1.read();
        motor_set_dir(temp);
    }
    Serial.println(temp);
    delay(1);
}
