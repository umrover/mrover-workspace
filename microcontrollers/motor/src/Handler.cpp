#include "mbed.h"

class Handler {
public:
  virtual bool correct_channel(int[] &data);
  virtual void format(int[] &data) = 0;
private:
  const int channel_id;
}

static int toTalonValue(const double amount) {
  if (amount > -0.1 && amount < 0.1) {
    return 1513;
  }
  if (amount > 0) {
    return 1539 + ((2037 - 1539) * amount);
  }
  if (amount < 0) {
    return 1487 + ((1487 - 989) * amount);
  }
  return 1513;
}

class DriveHandler : public Handler {
public:
  virtual bool correct_channel(int[] &data){
    return data[0] == channel_id;
  }
  virtual void format(int[] &data){
    int leftTalon = toTalonValue(data[1]);
    int rightTalon = toTalonValue(data[2]);
    device.baud(9600);
    device.printf("#0P" + leftTalon + "#1P" + leftTalon);
    device.printf("#2P" + rightTalon + "#3P" + rightTalon);
  }

private:
  const int channel_id = 0;
  Serial device(p9, p10);  // tx, rx //WHAT ARE THE ACTUAL PINS??? TODO
}
