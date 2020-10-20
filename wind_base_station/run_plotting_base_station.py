from multiprocessing import Process, Pipe

from radio_listener import run_radio_listener
from data_plotter import run_data_plotter

# The plotting base station consists of a radio listening process sending data
# to a plotting process through a pipe
parent_connection, child_connection = Pipe()
sensor_reading_subprocess = Process(target=run_radio_listener, args=(child_connection,))

print("Starting Reading")
sensor_reading_subprocess.start()

run_data_plotter(parent_connection)

# End the subprocess when the main program ends
sensor_reading_subprocess.join()
