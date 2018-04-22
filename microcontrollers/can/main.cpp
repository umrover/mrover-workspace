#include "mbed.h"

 

//Ticker ticker;

//DigitalOut led1(LED1);

DigitalOut led2(PD_8);

Serial dbg(USBTX, USBRX);

CAN can1(PA_11, PA_12);

//CAN can2(p30, p29);

char counter = 0;

 
/*
void send() {

    printf("send()\n");

    if(can1.write(CANMessage(1337, &counter, 1))) {

        printf("wloop()\n");

        counter++;

        printf("Message sent: %d\n", counter);

    } 

    led1 = !led1;

}
*/
 

int main() {

  //  printf("main()\n");
	dbg.printf("start\r\n");
  // ticker.attach(&send, 1);
	led2 = true;
	wait(2);
	led2 = false;

  CANMessage msg;

    while(1) {

        printf("loop\n\r");

        if(can1.read(msg)) {

            printf("Message received: %d\n\r", msg.data[0]);
			dbg.printf("Message received: %d\n\r", msg.data[0]);

            led2 = !led2;

        } 

        wait(0.2);

    }

}
