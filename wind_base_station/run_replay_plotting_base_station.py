from multiprocessing import Process, Pipe
import sys

from data_log_player import run_data_log_player
from data_plotter import run_data_plotter


# The log file must be passed as an argument to this script
log_filename = sys.argv[-1]

# The replay plotting base station consists of a data log replaying process sending data
# to a plotting process through a pipe. The data log replaying process mimics a live radio
# listener.
parent_connection, child_connection = Pipe()
data_log_player_subprocess = Process(target=run_data_log_player, args=(child_connection, log_filename))

print("Starting Replay")
data_log_player_subprocess.start()

run_data_plotter(parent_connection)

# End the subprocess when the main program ends
data_log_player_subprocess.join()
