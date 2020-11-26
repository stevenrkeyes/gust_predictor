from matplotlib import pyplot as plt
from matplotlib import animation as animation

def run_data_plotter(connection):
    figure = plt.figure()
    wind_subplot = figure.add_subplot(1, 1, 1)

    # Data buffers to be updated by the received radio packets and plotted live
    device_ids = range(256)
    timestamps_by_device = {device_id: [] for device_id in device_ids}
    wind_values_by_device = {device_id: [] for device_id in device_ids}

    def animate(i):
        # Check if there is new data to plot
        if connection.poll():
            # Read all of it
            while connection.poll():
                received_data = connection.recv()
                (timestamp, device_id, wind_value_raw) = received_data
                timestamps_by_device[device_id].append(timestamp)
                wind_values_by_device[device_id].append(wind_value_raw)
            wind_subplot.clear()
            for device_id in device_ids:
                timestamps = timestamps_by_device[device_id]
                if len(timestamps) > 0:
                    wind_values = wind_values_by_device[device_id]
                    wind_subplot.plot(timestamps, wind_values, label=str(device_id))
            plt.legend()

    animation_object = animation.FuncAnimation(figure, animate, interval=50)
    print("Starting plot")
    plt.show()
