#!/bin/bash

echo "Starting radio relay publisher and plotting subscriber"

# Run the following in a subshell with SIGINT mapped to kill everything in the subshell so that they all close when the
# user enters ctrl-C
(
  trap 'kill 0' SIGINT
  pipenv run python run_radio_relay_publisher.py &
  pipenv run python run_data_plotter_subscriber.py
)
