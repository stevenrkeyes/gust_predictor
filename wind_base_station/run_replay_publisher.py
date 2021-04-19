"""
Plays back a log file, publishing data as if it is live data from the radio.
"""

import sys
import datetime
import pickle

import zmq

import settings


def _load_data(log_filename):
    # The log files consist of pickled python objects
    log_file = open(log_filename, "rb")
    data_from_file = pickle.load(log_file)
    log_file.close()

    # The following Python objects are in the data in the following hardcoded order
    log_start_datetime = data_from_file[0]
    timestamps_by_device = data_from_file[1]
    wind_values_by_device = data_from_file[2]

    return log_start_datetime, timestamps_by_device, wind_values_by_device


# Set up the socket for publishing
context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind("tcp://*:" + str(settings.wind_data_port))

# The log file must be passed as an argument to this script
log_filename = sys.argv[-1]
log_start_datetime, timestamps_by_device, wind_values_by_device = _load_data(log_filename)

# The values will be replayed one at a time in order of timestamp, so gather them all into one sorted array
messages_to_send = []
for device_id in timestamps_by_device.keys():
    device_timestamps = timestamps_by_device[device_id]
    device_wind_values = wind_values_by_device[device_id]
    for (timestamp, wind_value) in zip(device_timestamps, device_wind_values):
        message_to_send = [timestamp, device_id, wind_value]
        messages_to_send.append(message_to_send)
# Sort by timestamp
messages_to_send.sort(key=lambda message: message[0])

# Use the same timestamps relative to the start time, but shift the start time so that it starts now.
player_start_datetime = datetime.datetime.now()
log_to_player_timedelta = player_start_datetime - log_start_datetime
for message_to_send in messages_to_send:
    message_to_send[0] += log_to_player_timedelta

print("Starting Replay")
for message_to_send in messages_to_send:
    # Wait until the current time has just reached the message timestamp, then send the message
    while datetime.datetime.now() < message_to_send[0]:
        pass
    socket.send_pyobj(message_to_send)
