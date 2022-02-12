import lcm
from exlcm import example_t

if __name__ == '__main__':
    # m = follium.Map()

    lc = lcm.LCM()
    subscription =lc.subscribe("",)

def my_handler(channel,data):
    msg = example_t.decode(data)





