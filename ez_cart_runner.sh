#!/bin/bash

EEB_HOME=/home/pi/electric_eel_barrow
EEB_USER=pi

EXIT_CODE=1
while [ $EXIT_CODE -ne 0 ]; do
# while [ 1 ]; do
    sudo -H -u $EEB_USER python3 $EEB_HOME/ez_cart_main.py
    EXIT_CODE=$?
    echo "exit($EXIT_CODE)"
done
