import pigpio
import nrf24
from os import environ as env
import struct
import datetime
import pickle

# Raspberry Pi 3 B+ Pin Connections:
# Pin 24 - CSN
# Pin 22 - CE
# Pin 18 - IRQ

# Get access to the pi's GPIO pins
pi = pigpio.pi(env.get('PIGPIO_HOST', 'localhost'), env.get('PIGPIO_PORT', 8888))
if not pi.connected:
    print("Not connected to Raspberry PI...goodbye.")
    exit()

# Set up NRF24L01 module for reading. The default SPI peripheral to use is correct.
radio = nrf24.NRF24(pi,ce=25, data_rate=nrf24.RF24_DATA_RATE.RATE_1MBPS, channel=115, payload_size=nrf24.RF24_PAYLOAD.DYNAMIC)

# The reading pipe must be set to the same channel as the RX and TX channels of
# the sender, but it seems that the writing pipe can be set to anything
radio.open_reading_pipe(nrf24.RF24_RX_ADDR.P0, [0x01, 0x01, 0x01, 0x01, 0x01])
radio.set_address_bytes(5)

# Create arrays to store the captured data by device
device_ids = range(256)
timestamps_by_device = {device_id: [] for device_id in device_ids}
wind_values_by_device = {device_id: [] for device_id in device_ids}

# Get the start datetime to use in the filename of the log
start_datetime = datetime.datetime.now()

print("Recording started.")
print("Press Ctrl+C to stop logging, save data, and exit.")
try:
    while True:
        while radio.data_ready():
            receive_time = datetime.datetime.now()
            report_packet_flattened_struct = radio.get_payload()

            # Unpack the data, which is a uint8 ('B') and a uint16 ('h')
            report_packet = struct.unpack('=Bhh', report_packet_flattened_struct)

            # The radio payload is a wind report packet consisting of device ID and data
            device_id = report_packet[0]
            wind_measurement_raw = report_packet[1]

            timestamps_by_device[device_id].append(receive_time)
            wind_values_by_device[device_id].append(wind_measurement_raw)
except KeyboardInterrupt:
    # Print a line to skip where ^C was printed
    print()

    # Make the time of the log clear from the filename
    output_filename = start_datetime.strftime("%Y_%m_%d_%H_%M_%S_%p")
    output_filename += "_wind_data_log.pkl"

    output_file = open(output_filename, "wb")
    data_to_write = [start_datetime, timestamps_by_device, wind_values_by_device]
    pickle.dump(data_to_write, output_file)
    output_file.close()

    print("Saved data as " + output_filename)
