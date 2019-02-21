import array
import usb
import time


# From https://github.com/chris-blay/android-open-accessory-bridge
# and AOAv1 docs.


class Android:
    def __init__(self,
                 uncfg_vendor_id, cfg_vendor_id,
                 uncfg_product_id, cfg_product_id,
                 manufacturer, model, description, version, uri, serial):
        self.uncfg_vendor_id = int(uncfg_vendor_id)
        self.cfg_vendor_id = int(cfg_vendor_id)
        self.uncfg_product_id = int(uncfg_product_id)
        self.cfg_product_id = int(cfg_product_id)
        self.device = self._configure_and_open(
            str(manufacturer), str(model), str(description),
            str(version), str(uri), str(serial))
        self.tx, self.rx = self._detect_endpoints()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()

    def _detect_device(self, attempts_left=5):
        unconfigured_device = usb.core.find(
            idVendor=self.uncfg_vendor_id, idProduct=self.uncfg_product_id)
        configured_device = usb.core.find(
            idVendor=self.cfg_vendor_id, idProduct=self.cfg_product_id)
        if configured_device:
            return configured_device, True
        elif unconfigured_device:
            return unconfigured_device, False
        elif attempts_left:
            time.sleep(1)
            return self._detect_device(attempts_left - 1)
        else:
            raise usb.core.USBError('Device not connected')

    def _configure_device(self, device, *args):
        '''
        Tells the connected device to enter accessory mode
        '''
        buf = device.ctrl_transfer(0xc0, 51, data_or_wLength=2)
        assert len(buf) == 2 and (buf[0] | buf[1] << 8) == 2

        for i, data in enumerate(args):
            assert(
                device.ctrl_transfer(
                    0x40, 52, wIndex=i, data_or_wLength=data) == len(data))
        assert(device.ctrl_transfer(0x40, 53) == 0)
        usb.util.dispose_resources(device)

    def _configure_and_open(self, *args):
        device, cfg = self._detect_device()
        if not cfg:
            self._configure_device(device, *args)
        else:
            # Bring app to foreground
            device.reset()
            time.sleep(1)

        # Now, the configured version should show up
        attempts_left = 5
        while attempts_left > 0:
            device, cfg = self._detect_device()
            if cfg:
                return device
            time.sleep(1)
            attempts_left -= 1
        raise usb.core.USBError('Device not configured')

    def _detect_endpoints(self):
        assert self.device
        cfg = self.device.get_active_configuration()
        interface = cfg[(0, 0)]

        def first_out(endpoint):
            return (usb.util.endpoint_direction(endpoint.bEndpointAddress)
                    == usb.util.ENDPOINT_OUT)

        def first_in(endpoint):
            return (usb.util.endpoint_direction(endpoint.bEndpointAddress)
                    == usb.util.ENDPOINT_IN)

        tx = usb.util.find_descriptor(interface, custom_match=first_out)
        rx = usb.util.find_descriptor(interface, custom_match=first_in)
        assert tx and rx
        return tx, rx

    def write(self, data, timeout=None):
        assert self.device and self.tx and data
        size = len(data)
        size_bytes = array.array('B', [
            (size & 0x0000ff00) >> 8,
            (size & 0x000000ff)])
        data_bytes = array.array('B', data)
        while True:
            try:
                bytes_written = self.tx.write(size_bytes, timeout=timeout)
            except usb.core.USBError as e:
                if e.errno == 110:  # Timeout
                    continue
                else:
                    raise e
            else:
                assert bytes_written == 2
                break
        assert self.tx.write(data_bytes, timeout=timeout) == size

    def read(self, timeout=None):
        assert self.device and self.rx
        try:
            size_bytes = self.rx.read(2, timeout=timeout)
            size = (size_bytes[0] << 8) | size_bytes[1]
            return self.rx.read(size, timeout=timeout)
        except usb.core.USBError as e:
            if e.errno == 110:  # timeout
                return None
            else:
                raise e

    def close(self):
        assert self.device and self.tx
        self.tx.write(array.array('B', [0, 0]))
        usb.util.dispose_resources(self.device)
        self.device = None
        self.tx = None
        self.rx = None
