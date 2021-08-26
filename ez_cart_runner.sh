#!/bin/bash

EEB_HOME=/home/rileytuttle/electric_eel_barrow
EEB_USER=rileytuttle

EXIT_CODE=1
# while [ $EXIT_CODE -ne 0 ]; do
while [ 1 ]; do
    sudo -H -u $EEB_USER python3 $EEB_HOME/ez_cart_main.py
    EXIT_CODE=$?
done
