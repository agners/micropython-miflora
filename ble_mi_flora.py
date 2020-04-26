# BLE Class which allows to connect to Mi Flora BLE plant sensors
# Based on the ble_temperature_central.py example.

import bluetooth
import random
import struct
import time
import micropython
import machine

from micropython import const

_IRQ_CENTRAL_CONNECT = const(1 << 0)
_IRQ_CENTRAL_DISCONNECT = const(1 << 1)
_IRQ_GATTS_WRITE = const(1 << 2)
_IRQ_GATTS_READ_REQUEST = const(1 << 3)
_IRQ_SCAN_RESULT = const(1 << 4)
_IRQ_SCAN_COMPLETE = const(1 << 5)
_IRQ_PERIPHERAL_CONNECT = const(1 << 6)
_IRQ_PERIPHERAL_DISCONNECT = const(1 << 7)
_IRQ_GATTC_SERVICE_RESULT = const(1 << 8)
_IRQ_GATTC_CHARACTERISTIC_RESULT = const(1 << 9)
_IRQ_GATTC_DESCRIPTOR_RESULT = const(1 << 10)
_IRQ_GATTC_READ_RESULT = const(1 << 11)
_IRQ_GATTC_WRITE_STATUS = const(1 << 12)
_IRQ_GATTC_NOTIFY = const(1 << 13)
_IRQ_GATTC_INDICATE = const(1 << 14)
_IRQ_ALL = const(0xFFFF)

_ADV_IND = const(0x00)
_ADV_DIRECT_IND = const(0x01)
_ADV_SCAN_IND = const(0x02)
_ADV_NONCONN_IND = const(0x03)

# Mi Flora root service
_ENV_FLORA_UUID = bluetooth.UUID(0xfe95)

# Mi Flora data service
_ENV_FLORA_DATA_UUID = bluetooth.UUID('00001204-0000-1000-8000-00805f9b34fb')

# Mi Flora charactristics
_FLORA_CMD_CHAR = bluetooth.UUID("00001a00-0000-1000-8000-00805f9b34fb")
_FLORA_DATA_CHAR = bluetooth.UUID("00001a01-0000-1000-8000-00805f9b34fb")
_FLORA_FIRM_CHAR = bluetooth.UUID("00001a02-0000-1000-8000-00805f9b34fb")

# Advertising payloads are repeated packets of the following form:
#   1 byte data length (N + 1)
#   1 byte type (see constants below)
#   N bytes type-specific data

_ADV_TYPE_FLAGS = const(0x01)
_ADV_TYPE_NAME = const(0x09)
_ADV_TYPE_UUID16_COMPLETE = const(0x3)
_ADV_TYPE_UUID32_COMPLETE = const(0x5)
_ADV_TYPE_UUID128_COMPLETE = const(0x7)
_ADV_TYPE_UUID16_MORE = const(0x2)
_ADV_TYPE_UUID32_MORE = const(0x4)
_ADV_TYPE_UUID128_MORE = const(0x6)

def decode_field(payload, adv_type):
    i = 0
    result = []
    while i + 1 < len(payload):
        if payload[i + 1] in adv_type:
            result.append(payload[i + 2 : i + payload[i] + 1])
        i += 1 + payload[i]
    return result


def decode_name(payload):
    n = decode_field(payload, [_ADV_TYPE_NAME])
    return str(n[0], "utf-8") if n else ""


def decode_services(payload):
    services = []
    for u in decode_field(payload, [_ADV_TYPE_UUID16_COMPLETE, _ADV_TYPE_UUID16_MORE]):
        services.append(bluetooth.UUID(struct.unpack("<h", u)[0]))
    for u in decode_field(payload, [_ADV_TYPE_UUID32_COMPLETE, _ADV_TYPE_UUID32_MORE]):
        services.append(bluetooth.UUID(struct.unpack("<d", u)[0]))
    for u in decode_field(payload, [_ADV_TYPE_UUID128_COMPLETE, _ADV_TYPE_UUID128_MORE]):
        services.append(bluetooth.UUID(u))
    return services


class BLEMiFloraCentral:
    def __init__(self, ble):
        self._ble = ble
        self._ble.irq(handler=self._irq)

        # Cached name and address from a successful scan.
        self._devices = {}

        self._reset()

    def _reset(self):
        # Active device...
        self._addr_type = None
        self._addr = None

        self._connecting = False

        # Cached value (if we have one)
        self._value = None

        # Callbacks for completion of various operations.
        # These reset back to None after being invoked.
        self._scan_callback = None

        # Persistent callback for when new data is notified from the device.
        self._notify_callback = None

        # Connected device.
        self._conn_handle = None
        self._start_handle = None
        self._end_handle = None

        self._characteristics_handle = { }
        self._read_status = None
        self._read_data = None
        self._write_status = None


    def _irq(self, event, data):
        if event == _IRQ_SCAN_RESULT:
            addr_type, addr, adv_type, rssi, adv_data = data
            if adv_type in (_ADV_IND, _ADV_DIRECT_IND,) and _ENV_FLORA_UUID in decode_services(
                adv_data
            ):
                # Note: addr buffer is owned by caller so need to copy it.
                device_tuple = (addr_type, bytes(addr))

                # Found a Mi Flora device, remember it
                self._devices[device_tuple] = decode_name(adv_data) or "?"

        elif event == _IRQ_SCAN_COMPLETE:
            self._scanning = False
            if self._scan_callback:
                if self._addr:
                    # Found a device during the scan (and the scan was explicitly stopped).
                    self._scan_callback(self._devices)
                    self._scan_callback = None
                else:
                    # Scan timed out.
                    self._scan_callback(None, None, None)

        elif event == _IRQ_PERIPHERAL_CONNECT:
            # Connect successful.
            print("_IRQ_PERIPHERAL_CONNECT")
            conn_handle, addr_type, addr, = data
            if addr_type == self._addr_type and addr == self._addr:
                self._conn_handle = conn_handle
                self._connecting = False

        elif event == _IRQ_PERIPHERAL_DISCONNECT:
            # Disconnect (either initiated by us or the remote end).
            conn_handle, _, _, = data

            print("_IRQ_PERIPHERAL_DISCONNECT")
            # Somehow we get sometimes a disconnect event altough we never got
            # a connect... It seems that connecting aborted somehow.
            self._connecting = False

            if conn_handle == self._conn_handle:
                # If it was initiated by us, it'll already be reset.
                self._reset()

        elif event == _IRQ_GATTC_SERVICE_RESULT:
            # Connected device returned a service.
            conn_handle, start_handle, end_handle, uuid = data
            print("_IRQ_GATTC_SERVICE_RESULT", uuid)
            if conn_handle == self._conn_handle and uuid == _ENV_FLORA_DATA_UUID:
                self._start_handle = start_handle
                self._end_handle = end_handle

        elif event == _IRQ_GATTC_CHARACTERISTIC_RESULT:
            # Connected device returned a characteristic.
            conn_handle, def_handle, value_handle, properties, uuid = data
            print("_IRQ_GATTC_CHARACTERISTIC_RESULT", uuid)
            if conn_handle == self._conn_handle:
                self._characteristics_handle[uuid] = value_handle

        elif event == _IRQ_GATTC_READ_RESULT:
            # A read completed successfully.
            conn_handle, value_handle, char_data = data
            print("_IRQ_GATTC_READ_RESULT", char_data)
            if conn_handle == self._conn_handle:
                self._read_data = char_data

        elif event == _IRQ_GATTC_WRITE_STATUS:
            conn_handle, value_handle, status = data
            print("_IRQ_GATTC_WRITE_STATUS", status)

            self._write_status = status

    # Returns true if we've successfully connected and discovered characteristics.
    def is_connected(self):
        return self._conn_handle is not None

    def is_connecting(self):
        return self._connecting

    def discover_service(self, service_uuid):
        print("service_discover", service_uuid)
        self._ble.gattc_discover_service_by_uuid(self._conn_handle, service_uuid)

        while not self.is_service_discovered():
            machine.idle()

        return True

    def is_service_discovered(self):
        return self._start_handle is not None and self._end_handle is not None

    def discover_characteristic(self, chrs_uuid):
        print("start_characteristic_discover", chrs_uuid, chrs_uuid in self._characteristics_handle)
	self._ble.gattc_discover_characteristic_by_uuid(
	    self._conn_handle, self._start_handle, self._end_handle, chrs_uuid
	)

        while not chrs_uuid in self._characteristics_handle:
            machine.idle()

        print("start_characteristic_discover found", self._characteristics_handle[chrs_uuid])

    # Find a device advertising the environmental sensor service.
    def scan(self, duration_ms=20000, callback=None):
        self._scan_callback = callback
        self._scanning = True
        self._ble.gap_scan(duration_ms, 30000, 11250)

    def is_scan_finished(self):
        return not self._scanning

    def get_devices(self):
        return self._devices

    # Connect to the specified device (otherwise use cached address from a scan).
    def connect(self, addr_type=None, addr=None):
        self._addr_type = addr_type
        self._addr = addr
        if self._addr_type is None or self._addr is None:
            return False
        self._ble.gap_connect(self._addr_type, self._addr, 5000)
        self._connecting = True

        while self._connecting:
            machine.idle()

        return self.is_connected()

    # Disconnect from current device.
    def disconnect(self):
        if self._conn_handle is None:
            return

        print("disconnecting")
        self._ble.gap_disconnect(self._conn_handle)

        while self.is_connected():
            machine.idle()
        print("disconnecting done!")

        return True

    def write(self, value_handle, value, mode=0):
        self._ble.gattc_write(self._conn_handle, value_handle, value, mode)

        while self._write_status is None:
            machine.idle()

        print("Write done, status =", self._write_status)
        return self._write_status == 0

    def read(self, value_handle):
        self._read_data = None
        self._ble.gattc_read(self._conn_handle, value_handle)

        while self._read_data is None:
            machine.idle()

        return self._read_data

    # Issues an (asynchronous) write which then will trigger a read
    def read_sensor_values(self):
        if not self.is_connected():
            return

        if not self.is_service_discovered():
            self.discover_service(_ENV_FLORA_DATA_UUID)

        if not _FLORA_CMD_CHAR in self._characteristics_handle:
            self.discover_characteristic(_FLORA_CMD_CHAR)

        time.sleep_ms(300)
        print("Writing read sensor command...")
        value = struct.pack("<h", 0x1fa0)
        if not self.write(self._characteristics_handle[_FLORA_CMD_CHAR], value, 1):
            raise Exception("Write failed")

        if not _FLORA_DATA_CHAR in self._characteristics_handle:
            self.discover_characteristic(_FLORA_DATA_CHAR)

        time.sleep_ms(1000)
        self.read(self._characteristics_handle[_FLORA_DATA_CHAR])

        (temp, pad, lux, moisture, cond) =  struct.unpack("<hBIBH", self._read_data)
        temp = float(temp) / 10
        return (temp, lux, moisture, cond)


    def read_firmware(self):
        if not self.is_connected():
            return

        if not self.is_service_discovered():
            self.discover_service(_ENV_FLORA_DATA_UUID)

        if not _FLORA_FIRM_CHAR in self._characteristics_handle:
            self.discover_characteristic(_FLORA_FIRM_CHAR)

        time.sleep_ms(300)
        print("Reading firmware...")
        self.read(self._characteristics_handle[_FLORA_FIRM_CHAR])

        battery, padding, firmware_version = struct.unpack("BB5s", self._read_data)
        return (battery, firmware_version)

    def value(self):
        return self._value

def scan(central):
    print("Scanning for Mi Flora devices...")
    central.scan(duration_ms=5000)

    # Wait for devices
    while not central.is_scan_finished():
        time.sleep_ms(100)

    print("Scan finished, devices found:")
    for device_key, device_name in central.get_devices().items():
        addr_type, addr = device_key
        print("MAC: ", "".join('{:02x}:'.format(x) for x in addr)[:-1])

    return central.get_devices()

def demo():
    ble = bluetooth.BLE()
    ble.active(True)
    central = BLEMiFloraCentral(ble)

    devices = scan(central)

    # Use hardcoded list:
    #devices = {
    #        (0, b'\x80\xea\xca\x88\xd5\x82'): "Device 1",
    #        (0, b'\x80\xea\xca\x88\xd4\x69'): "Device 2",
    #        }

    # Now get sensor values for each device
    for device_key, device_name in devices.items():
        addr_type, addr = device_key

        # Connetion is not always successful, so try multiple times...
        print("Connecting to ", "".join('{:02x}:'.format(x) for x in addr)[:-1])
        while not central.connect(addr_type, addr):
            print("Connection failed, retrying")

        print("Connected")

        (temperature, light, moisture, conductivity) = central.read_sensor_values()
        print("Temperature: {0}°C".format(temperature))
        print("Light: {0} lux".format(light))
        print("Moisture: {0}%".format(moisture))
        print("Conductivity: {0}uS/cm".format(conductivity))

        battery, firmware_version = central.read_firmware()
        print("Battery: {0}%".format(battery))
        print("Firmware Version: {0}".format(firmware_version))
        central.disconnect()

        time.sleep(5)

if __name__ == "__main__":
    demo()
