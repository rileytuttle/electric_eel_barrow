#!/bin/bash

EEB_USER=pi
EEB_HOME=/home/pi/electric_eel_barrow

EXIT_CODE=1
while [ $EXIT_CODE -ne 0 ]; do
# while [ 1 ]; do
    sudo -H -u $EEB_USER python3 $EEB_HOME/ez_cart_main.py >> $EEB_HOME/logs/main_log.txt
    EXIT_CODE=$?
done
