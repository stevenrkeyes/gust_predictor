"""
Listens for wind data on the radio and publishes it to the socket.
"""

import datetime
import struct
from os import environ as env

import nrf24
import pigpio
import zmq

import settings


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

# Set up the socket for publishing
context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind("tcp://*:" + str(settings.wind_data_port))

while True:
    while radio.data_ready():
        receive_time = datetime.datetime.now()
        report_packet_flattened_struct = radio.get_payload()

        # Unpack the data, which is a uint8 ('B') and a uint16 ('h')
        report_packet = struct.unpack('=Bhh', report_packet_flattened_struct)

        # The radio payload is a wind report packet consisting of device ID and data
        device_id = report_packet[0]
        wind_measurement_raw = report_packet[1]

        data_to_send = (receive_time, device_id, wind_measurement_raw)
        socket.send_pyobj(data_to_send)
