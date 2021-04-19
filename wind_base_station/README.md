# Wind Base Station

## Setup
`pipenv --python 3 install`

## Usage
Once you have some remote sensors set up and running, you can start the base station, either to show a live plot or to save the data for plotting later. In either case, you must start pigpiod after powering on the computer.

`sudo pigpiod`

### Plot data live
`./start_radio_relay_and_plotter.sh`

### Save data
`pipenv run python run_data_logger_base_station.py`

### Replay saved data
`pipenv run python run_replay_publisher.py <name_of_log_file>`

If you'd like to display this data as well, run:

`pipenv run python run_data_plotter_subscriber.py`
