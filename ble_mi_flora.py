# BLE Class which allows to connect to Mi Flora BLE plant sensors
# Based on the ble_temperature_central.py example.

import bluetooth
import random
import struct
import time
import micropython

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
        self._conn_callback = None
        self._data_callback = None

        # Persistent callback for when new data is notified from the device.
        self._notify_callback = None

        # Connected device.
        self._conn_handle = None
        self._start_handle = None
        self._end_handle = None

        self._cmd_handle = None
        self._data_handle = None
        self._firm_handle = None

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
                print("_IRQ_PERIPHERAL_CONNECT, self._ble.gattc_discover_services", addr)
                self._ble.gattc_discover_services(self._conn_handle)

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
            if conn_handle == self._conn_handle and uuid == _ENV_FLORA_DATA_UUID:
                self._start_handle = start_handle
                self._end_handle = end_handle

        elif event == _IRQ_GATTC_CHARACTERISTIC_RESULT:
            # Connected device returned a characteristic.
            conn_handle, def_handle, value_handle, properties, uuid = data
            if conn_handle == self._conn_handle:
                if uuid == _FLORA_CMD_CHAR:
                    self._cmd_handle = value_handle
                if uuid == _FLORA_DATA_CHAR:
                    self._data_handle = value_handle
                if uuid == _FLORA_FIRM_CHAR:
                    self._firm_handle = value_handle

                # We've finished connecting and discovering device, fire the connect callback.
                if self._conn_callback and self.is_characteristic_discovered():
                    self._conn_callback()

        elif event == _IRQ_GATTC_READ_RESULT:
            # A read completed successfully.
            conn_handle, value_handle, char_data = data
            print("_IRQ_GATTC_READ_RESULT", char_data)
            if conn_handle == self._conn_handle:
                if value_handle == self._data_handle:
                    self._decode_data(char_data)
                    if self._data_callback:
                        self._data_callback(self._temperature, self._lux, self._moisture, self._conductivity)
                        self._data_callback = None
                elif value_handle == self._firm_handle:
                    self._decode_firm(char_data)
                    if self._battery_callback:
                        self._battery_callback(self._battery, self._firmware_version)
                        self._battery_callback = None

        elif event == _IRQ_GATTC_WRITE_STATUS:
            conn_handle, value_handle, status = data
            print("_IRQ_GATTC_WRITE_STATUS", status)

            if status == 0:
                self._ble.gattc_read(self._conn_handle, self._data_handle)
            elif self._data_callback:
                self._data_callback(None, None, None, None)

    # Returns true if we've successfully connected and discovered characteristics.
    def is_connected(self):
        return self._conn_handle is not None

    def is_connecting(self):
        return self._connecting

    def is_service_discovered(self):
        return self._start_handle is not None and self._end_handle is not None

    def is_characteristic_discovered(self):
	return (self._firm_handle is not None and self._cmd_handle is not None
                and self._data_handle is not None)

    def start_characteristic_discover(self):
        print("start_characteristic_discover")
	self._ble.gattc_discover_characteristics(
	    self._conn_handle, self._start_handle, self._end_handle
	)

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
    def connect(self, addr_type=None, addr=None, callback=None):
        self._addr_type = addr_type
        self._addr = addr
        self._conn_callback = callback
        if self._addr_type is None or self._addr is None:
            return False
        print(self._conn_handle)
        print(self._ble.gap_connect(self._addr_type, self._addr, 5000))
        self._connecting = True
        return True

    # Disconnect from current device.
    def disconnect(self):
        if not self._conn_handle:
            return
        self._ble.gap_disconnect(self._conn_handle)

    # Issues an (asynchronous) write which then will trigger a read
    def read_sensor_values(self, callback):
        if not self.is_connected():
            return

        self._data_callback = callback
        value = struct.pack("<h", 0x1fa0)
        self._ble.gattc_write(self._conn_handle, self._cmd_handle, value, 1)

    def read_firmware(self, callback):
        if not self.is_connected():
            return

        self._battery_callback = callback
        self._ble.gattc_read(self._conn_handle, self._firm_handle)

    # Sets a callback to be invoked when the device notifies us.
    def on_notify(self, callback):
        self._notify_callback = callback

    def _decode_data(self, data):
        temp, padding, lux, moisture, conductivity = struct.unpack("<hBIBH", data)
        print(temp, lux, moisture, conductivity)
        self._temperature = float(temp) / 10
        self._lux = lux
        self._moisture = moisture
        self._conductivity = conductivity

    def _decode_firm(self, data):
        battery, padding, firmware_version = struct.unpack("BB5s", data)
        print("Battery", battery, firmware_version)
        self._battery = battery
        self._firmware_version = firmware_version

    def value(self):
        return self._value


def demo():
    ble = bluetooth.BLE()
    ble.active(True)
    central = BLEMiFloraCentral(ble)

    print("Scanning for Mi Flora devices...")
    central.scan(duration_ms=5000)

    # Wait for devices
    while not central.is_scan_finished():
        time.sleep_ms(100)

    print("Scan finished, devices found:")
    for device_key, device_name in central.get_devices().items():
        addr_type, addr = device_key
        print("MAC: ", "".join('{:02x}:'.format(x) for x in addr)[:-1])

    # Now get sensor values for each device
    for device_key, device_name in central.get_devices().items():
        addr_type, addr = device_key

        # Connetion is not always successful, so try multiple times...
        for attempt in range(5):
            if not attempt == 0:
                print("Connection unsuccessful (attempt {0})".format(attempt + 1))
                central.disconnect()
                while central.is_connected():
                    time.sleep_ms(100)

                time.sleep_ms(1000)

            print("Connecting to ", "".join('{:02x}:'.format(x) for x in addr)[:-1])
            central.connect(addr_type, addr)

            # Wait for connection...
            while not central.is_connected() and central.is_connecting():
                time.sleep_ms(100)

            if not central.is_connected():
                continue

            print("Connected")

            # Wait for service discovery...
            while not central.is_service_discovered() and central.is_connected():
                time.sleep_ms(100)
                print(".")

            if not central.is_connected():
                continue
            print("Service discovered main loop")

            # Work around bug in ESP IDF NimBLE implementation:
            # Wait for service discovery to complete before triggering charactristic
            # discovery.
            # https://github.com/espressif/esp-idf/issues/4913
            time.sleep_ms(1500)

            central.start_characteristic_discover()

            while not central.is_characteristic_discovered() and central.is_connected():
                time.sleep_ms(100)
                print(".")

            if not central.is_connected():
                continue

            print("Charactristics we are looking for discovered")

            # Work around same issue here...
            time.sleep_ms(3000)

            # Explicitly issue reads, using "print" as the callback.
            print("Reading values")

            sensor_values_received = False
            def sensor_values(temperature, light, moisture, conductivity):
                print("Temperature: {0}°C".format(temperature))
                print("Light: {0} lux".format(light))
                print("Moisture: {0}%".format(moisture))
                print("Conductivity: {0}uS/cm".format(conductivity))
                nonlocal sensor_values_received
                sensor_values_received = True

            central.read_sensor_values(callback=sensor_values)

            firmware_values_received = False
            def firmware_values(battery, firmware_version):
                print("Battery: {0}%".format(battery))
                print("Firmware Version: {0}".format(firmware_version))
                nonlocal firmware_values_received
                firmware_values_received = True

            central.read_firmware(callback=firmware_values)

            while ((not sensor_values_received or not firmware_values_received)
                   and central.is_connected()):
                time.sleep_ms(100)
                print(".")

            if not central.is_connected():
                continue
            central.disconnect()

            # This is important otherwise the BLE stack won't allow us to
            # connect to another device.
            while central.is_connected():
                time.sleep_ms(100)

            print("Disconnected")

            # Successful attempt, break retry looop
            break


if __name__ == "__main__":
    demo()
