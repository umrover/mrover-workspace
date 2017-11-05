#include <Handler.cpp>

CAN can(p30, p29);

int main() {
  CANMessage msg;
  DriverHandler driver();
  while(1) {
    if(can.read(msg) && driver.correct_channel(msg.data)) {
      printf("Message received: %d\n", msg.data[0]);
      driver.format(msg.data);
    }
    wait(0.2);
  }
}
