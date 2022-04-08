#!/bin/bash

while [[ -z $(bluetoothctl paired-devices | grep "Wireless Controller") ]]; do
    /electric_eel_barrow/connect_ds4.expect
done
