# Wind Base Station

## Setup
`pipenv --python 3 install`

## Usage
Once you have some remote sensors set up and running, you can start the base station, either to show a live plot or to save the data for plotting later. In either case, you must start pigpiod after powering on the computer.

`sudo pigpiod`

### Plot data live
`pipenv run python run_plotting_base_station.py`

### Save data
`pipenv run python run_data_logger_base_station.py`

### Replay saved data
`pipenv run python run_replay_plotting_base_station.py <name_of_log_file>`
