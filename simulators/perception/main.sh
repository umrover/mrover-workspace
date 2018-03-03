#!/bin/bash

gzserver "$1" &
GZSERVER_PID=$!
gzweb &
GZWEB_PID=$!

wait $GZSERVER_PID $GZWEB_PID
