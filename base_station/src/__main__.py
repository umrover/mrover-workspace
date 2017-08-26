from rover_common import heartbeatlib


def connection_state_changed(c):
    if c:
        print("Connection established.")
    else:
        print("Disconnected.")


def main():
    hb = heartbeatlib.BaseStationHeartbeater(connection_state_changed)

    try:
        hb.loop()
    except KeyboardInterrupt:
        pass
