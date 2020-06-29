#include "mbed.h"

#include "bbcar.h"

#include "arm_math.h"

#include "FXOS8700CQ.h"

#include <math.h>

#include <stdlib.h>

#define bound 0.9



Ticker servo_ticker;

Ticker encoder_ticker;

PwmOut pin9(D9), pin8(D8);

DigitalIn pin3(D3);

DigitalInOut pin10(D10);

BBCar car(pin8, pin9, servo_ticker);

EventQueue queue(32 * EVENTS_EVENT_SIZE);
Thread thread1;
Thread t;

Serial pc(USBTX,USBRX); //tx,rx
Serial uart(D1,D0); //tx,rx
RawSerial xbee(D12, D11);

FXOS8700CQ acc(PTD9, PTD8, (0x1D<<1));

parallax_encoder encoder0(pin3, encoder_ticker);

float state[3] = {0};

float Kp = 0, Ki = 0, Kd = 0;

float a0 = 0, a1 = 0, a2 = 0;

void go(int speed, int distance);
void xbee_rx_interrupt(void);
void xbee_rx(void);
void reply_messange(char *xbee_reply, char *messange);
void check_addr(char *xbee_reply, char *messenger);
void recieve_thread(void);
void send_thread(void);


//The formula is:

//y[n] = y[n-1] + A0 * x[n] + A1 * x[n-1] + A2 * x[n-2]

//A0 = Kp + Ki + Kd

//A1 = (-Kp ) - (2 * Kd )

//A2 = Kd

void pid_init(){

    state[0] = 0;

    state[1] = 0;

    state[2] = 0;

    a0 = Kp + Ki + Kd;

    a1 = (-Kp) - 2*Kd;

    a2 = Kd;

}

float pid_process(float in){

    int out = in*a0 + a1*state[0] + a2*state[1] + state[2];


    //update state

    state[1] = state[0];

    state[0] = in;

    state[2] = out;


    return out;

}


void pid_control(char rotation, int turn){

  //pid control setup

  Kp = 2.0; Ki = 1.0; Kd = 0;

  pid_init();


  //sensor setup

  acc.enable();

  SRAWDATA accel_data, magn_data;

  char buff[256];

  float degree, target_degree, diff;


  wait(1);

  acc.get_data(&accel_data, &magn_data);

  degree = atan2(magn_data.y, magn_data.x) * 180 / PI;


  if(rotation == 'l'){

    target_degree = degree - turn;

  }else if(rotation == 'r'){

    target_degree = degree + turn;

  }else{

        target_degree = degree;

    }


    if(target_degree < -180){

        target_degree = 360 + target_degree;

    }else if(target_degree > 180){

        target_degree = 360 - target_degree;

    }

    diff = degree - target_degree;


      //The car will continue to turn to the target degree until the error is small enough

    while(abs(diff) > 8){

          //Process the PID control

          float correction = pid_process(diff);

          //bound the value from -0.9 to -.9

          correction = car.clamp(correction, bound, -bound);

          float turn = (rotation == 'l') ? (1-abs(correction)) : (-1+abs(correction));

          car.turn(car.turn2speed(turn),turn);

          wait(0.1);


          acc.get_data(&accel_data, &magn_data);

          degree = atan2(magn_data.y, magn_data.x) * 180 / PI;


          diff = degree - target_degree;

          pc.printf("degree:%f, target: %f, diff:%f \r\n", degree, target_degree, diff);

    }

      car.stop();


      pid_init();

}



int main() {

    pc.baud(9600);

    char xbee_reply[4];

    // XBee setting
    xbee.baud(9600);
    xbee.printf("+++");
    xbee_reply[0] = xbee.getc();
    xbee_reply[1] = xbee.getc();
    if(xbee_reply[0] == 'O' && xbee_reply[1] == 'K'){
        pc.printf("enter AT mode.\r\n");
        xbee_reply[0] = '\0';
        xbee_reply[1] = '\0';
    }
    xbee.printf("ATMY 0x240\r\n");
    reply_messange(xbee_reply, "setting MY : 0x240");

    xbee.printf("ATDL 0x140\r\n");
    reply_messange(xbee_reply, "setting DL : 0x140");

    xbee.printf("ATID 0x1\r\n");
    reply_messange(xbee_reply, "setting PAN ID : 0x1");

    xbee.printf("ATWR\r\n");
    reply_messange(xbee_reply, "write config");

    xbee.printf("ATMY\r\n");
    check_addr(xbee_reply, "MY");

    xbee.printf("ATDL\r\n");
    check_addr(xbee_reply, "DL");

    xbee.printf("ATCN\r\n");
    reply_messange(xbee_reply, "exit AT mode");
    xbee.getc();

     start

    pc.printf("start\r\n");

    t.start(callback(&queue, &EventQueue::dispatch_forever));*/

    go(100,130);
    xbee.printf("forward ");

    wait(1);

    pid_control('l',100);
    xbee.printf("left ");

    wait(1);

    go(100,87);
    xbee.printf("forward ");

    wait(1);

    pid_control('r',100);
    xbee.printf("right ");
    
    wait(1);

    go(-100,55);
    xbee.printf("backward ");

    wait(2);

    go(100,55);
    xbee.printf("forward ");

    wait(1);

    pid_control('r',100);
    xbee.printf("right ");
    
    wait(1);

    go(100,18);
    xbee.printf("forward ");

    wait(1);

    pid_control('l',100);
    xbee.printf("left ");


    uart.baud(9600);

    thread1.start(recieve_thread);
    send_thread();

    pid_control('r',100);
    xbee.printf("right ");

    go(100,25);
    xbee.printf("forward ");

    pid_control('r',100);
    xbee.printf("right ");

    go(100,125);
    xbee.printf("forward ");

    pid_control('r',100);
    xbee.printf("right ");

    go(100,100);
    xbee.printf("forward ");

    pid_control('r',100);
    xbee.printf("right ");

    go(100,125);
    xbee.printf("forward ");

}

void recieve_thread(){

   while(1) {

      if(uart.readable()){

            char recv = uart.getc();

            pc.putc(recv);

            pc.printf("\r\n");

      }

   }

}

void send_thread(){

    char s[21];

    sprintf(s,"image_classification");

    uart.puts(s);

    pc.printf("send\r\n");

    wait(0.5);
}

void go(int speed, int distance){

    encoder0.reset();

    car.goStraight(speed);

    while(encoder0.get_cm()<distance) wait_ms(50);

    car.stop();
}

void xbee_rx_interrupt(void)

{

  xbee.attach(NULL, Serial::RxIrq); // detach interrupt

  queue.call(&xbee_rx);

}


void xbee_rx(void)

{

  char buf[100] = {0};


  while(xbee.readable()){

    for (int i=0; ; i++) {

      char recv = xbee.getc();

      if (recv == '\r') {

        break;

      }

      buf[i] = pc.putc(recv);

    }


    wait(0.1);

  }

  xbee.attach(xbee_rx_interrupt, Serial::RxIrq); // reattach interrupt

}

void reply_messange(char *xbee_reply, char *messange){

  xbee_reply[0] = xbee.getc();

  xbee_reply[1] = xbee.getc();

  xbee_reply[2] = xbee.getc();

  if(xbee_reply[1] == 'O' && xbee_reply[2] == 'K'){

    pc.printf("%s\r\n", messange);

    xbee_reply[0] = '\0';

    xbee_reply[1] = '\0';

    xbee_reply[2] = '\0';

  }

}


void check_addr(char *xbee_reply, char *messenger){

  xbee_reply[0] = xbee.getc();

  xbee_reply[1] = xbee.getc();

  xbee_reply[2] = xbee.getc();

  xbee_reply[3] = xbee.getc();

  pc.printf("%s = %c%c%c\r\n", messenger, xbee_reply[1], xbee_reply[2], xbee_reply[3]);

  xbee_reply[0] = '\0';

  xbee_reply[1] = '\0';

  xbee_reply[2] = '\0';

  xbee_reply[3] = '\0';

}