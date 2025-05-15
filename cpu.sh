#!/bin/bash

echo "Timestamp,PID,CPU%,MEM%,CMD" > cpu_log.csv

while true; do
  timestamp=$(date +"%Y-%m-%d %H:%M:%S")
  ps -C planner_server,controller_server -o pid,%cpu,%mem,cmd --no-headers | \
  while read pid cpu mem cmd; do
    echo "$timestamp,$pid,$cpu,$mem,$cmd" >> cpu_log.csv
    done
      sleep 1
    done
