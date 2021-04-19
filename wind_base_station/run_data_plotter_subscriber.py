"""
Plots any published wind data.
"""

from matplotlib import pyplot as plt
from matplotlib import animation as animation
import zmq

import settings

context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.connect("tcp://localhost:" + str(settings.wind_data_port))
socket.subscribe("")

figure = plt.figure()
wind_subplot = figure.add_subplot(1, 1, 1)

# Data buffers to be updated by the received radio packets and plotted live
device_ids = range(256)
timestamps_by_device = {device_id: [] for device_id in device_ids}
wind_values_by_device = {device_id: [] for device_id in device_ids}

def _animate(i):
    # Get any new data available until there is no more
    messages_available = True
    while messages_available:
        try:
            received_data = socket.recv_pyobj(flags=zmq.NOBLOCK)
            (timestamp, device_id, wind_value_raw) = received_data
            timestamps_by_device[device_id].append(timestamp)
            wind_values_by_device[device_id].append(wind_value_raw)
        except zmq.ZMQError:
            messages_available = False
    # Clear the plot and plot all the data
    wind_subplot.clear()
    wind_subplot.set_xlabel("Time")
    wind_subplot.set_ylabel("Measurement (Raw Value)")
    for device_id in device_ids:
        timestamps = timestamps_by_device[device_id]
        if len(timestamps) > 0:
            wind_values = wind_values_by_device[device_id]
            wind_subplot.plot(timestamps, wind_values, label=str(device_id))
    plt.legend(title="Device ID")

animation_object = animation.FuncAnimation(figure, _animate, interval=50)
print("Starting plot")
plt.show()
