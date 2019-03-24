import sys
import argparse
from intelhex import IntelHex
from bluepy.btle import Scanner, DefaultDelegate, Peripheral, ADDR_TYPE_RANDOM
from struct import pack, unpack
from enum import Enum


class NotificationDelegate(DefaultDelegate):
    def __init__(self, recorder):
        DefaultDelegate.__init__(self)
        self.recorder = recorder

    def handleNotification(self, cHandle, data):
        self.recorder.notification(cHandle, data)

class Recorder():
    RECORDER_SERVICE_UUID = "8965c8d0-b543-9695-6414-789670ac1cf7"
    RECORDER_CONTROL_UUID ="8e921d16-a929-4c0c-41ed-8e40fa37013f"
    RECORDER_DATA_UUID =   "8c817150-dfc8-f45d-7d8e-e8a539c4a101"
    RECORDER_STATUS_UUID = "dfc54332-8dda-7f88-2699-630483c2fd99"
    peripheral = None
    service = None
    control_char = None
    status_char = None
    data_char = None
    readout_in_progress = False
    remaining_packets = 0

    def __init__(self, peripheral):
        self.peripheral = peripheral
        self.service = peripheral.getServiceByUUID(self.RECORDER_SERVICE_UUID)
        characteristics = self.service.getCharacteristics()
        for c in characteristics:
            if c.uuid == self.RECORDER_CONTROL_UUID:
                self.control_char = c
            elif c.uuid == self.RECORDER_DATA_UUID:
                self.data_char = c
            elif c.uuid == self.RECORDER_STATUS_UUID:
                self.status_char = c
        if (self.service is None or
            self.control_char is None or
            self.data_char is None or
            self.status_char is None):
                raise Exception("Could not find all characteristics in recorder service on %s" % (peripheral))

        print("Initialized recorder application")
        self.delegate = NotificationDelegate(self)
        self.peripheral.withDelegate(self.delegate)

    def get_status(self):
        status = self.status_char.read()
        flags, recorder_size, max_connection_time, max_connectable_time, sensor_update_rate, last_temperature_update, src_version = unpack("<IHHHHII", status)
        print("Flags: %8X buffer used: %d connection time: %d s, connectable time: %d s, sensor rate: %d s, recent measurement age: %d s, git hash %8x" % (flags, recorder_size, max_connection_time, max_connectable_time, sensor_update_rate, last_temperature_update, src_version))
        return flags, recorder_size, max_connection_time, max_connectable_time, sensor_update_rate, last_temperature_update, src_version

    def write_readout_command(self, size):
        data = pack("<BH", 1, size)
        self.control_char.write(data, True)
        print("Requested to read %d data points" % (size))

    def set_measurement_interval(self, interval):
        data = pack("<BH", 4, interval)
        self.control_char.write(data, True)

    def reset_storage(self):
        data = pack("<B", 5)
        self.control_char.write(data, True)

    def reset_to_ota(self):
        data = pack("<B", 6)
        self.control_char.write(data, True)

    def register_indication(self, valHandle, enabled):
        data = pack("<H", 2 if enabled else 0)
        self.peripheral.writeCharacteristic(valHandle + 1, data)

    def read(self, size):
        self.register_indication(self.data_char.getHandle(), True)
        self.remaining_packets = size
        self.write_readout_command(size)
        print("Starting readout")
        self.readout_in_progress = True
        while self.readout_in_progress:
            if self.peripheral.waitForNotifications(10) == False:
                raise Exception("Upload failed with timeout")

    def notification(self, handle, data):
        if handle == self.data_char.getHandle():
            unpacked = unpack("<10H", data)
            for d in unpacked:
                if d == 0x8003:
                    print("Start measurement")
                elif d == 0x8002:
                    print("Stop measurement")
                elif d == 0x8000:
                    print("()")
                elif (d & 0xF000) == 0x9000:
                    print("Measurement interval: %d s" % (d & 0x0FFF))
                elif (d & 0xC000) == 0xC000:
                    print("Time elapsed: %d s" % (d & 0x3FFF))
                else:
                    if d & 0x4000:
                        print("negative temperature")
                    else:
                        print("%f °C" % (d / 100))
                
            self.remaining_packets = self.remaining_packets - 10
            if self.remaining_packets <= 0:
                self.readout_in_progress = False
            sys.stdout.flush()
        else:
            print("\nReceived notification for %d with %s" % (handle, data.hex()))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--scan", help="Scans for Bluetooth low energy devices", action="store_true")
    parser.add_argument("--device", help="Connects to given bluetooth device")
    parser.add_argument("--read", help="Read given number of measurements from given device", type=int)
    parser.add_argument("--set_measurement_interval", help="take a measurement every X seconds", type=int)
    parser.add_argument("--reset_storage", help="Empty the measurement buffer", action="store_true")
    parser.add_argument("--reset_to_bootloader", help="Reset thermometer (losing all data) to bootloader for firmware update", action="store_true")
    args = parser.parse_args()

    if args.scan:
        scanner = Scanner()
        devices = scanner.scan(5.0)

        for dev in devices:
            print("Device %s (%s), RSSI=%d dB, connectable=%r" % (dev.addr, dev.addrType, dev.rssi, dev.connectable))
            for (adtype, desc, value) in dev.getScanData():
                print("  %s = %s" % (desc, value))

    if args.device:
        peripheral = Peripheral(args.device, ADDR_TYPE_RANDOM)
        recorder = Recorder(peripheral)
        recorder.get_status()

    if args.device and args.read:
        recorder.read(args.read)

    if args.device and args.reset_storage:
        recorder.reset_storage()
        print("Reset device storage")

    if args.device and args.reset_to_bootloader:
        recorder.reset_to_ota()
        print("Device should be in OTA mode now")

    if args.device and args.set_measurement_interval:
        recorder.set_measurement_interval(args.set_measurement_interval)
        print("Measurement interval: %d s" % (args.set_measurement_interval))


if __name__ == "__main__":
    main()
