import lcm

lcm_ = lcm.LCM()


def mosfet_callback(channel, msg):
    print("call back received")


def main():
    lcm_.subscribe("/mosfet_cmd", mosfet_callback)

    while True:
        lcm_.handle()
        print('lcm received')


if __name__ == "__main__":
    main()
