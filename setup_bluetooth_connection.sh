#!/bin/bash


# start dbus
/etc/init.d/dbus stat
# start bluetooth daemon
/usr/lib/bluetooth/bluetoothd
# add user to the bluetooth group
sudo usermod -G bluetooth root

# the we set up bluetooth connection for all gamepads we know about
# once I take the above section out I can make it loop until something is successful
MAC_ADDRESSES=("XX:XX:XX:XX:XX:XX", "XX:XX:XX:XX:XX:XX")

for mac_address in ${MAC_ADDRESSES[@]}; do
    bluetoothctl disconnect $mac_address
    bluetoothctl trust $mac_address
    bluetoothctl pair $mac_address
    bluetoothctl connect $mac_address
done
