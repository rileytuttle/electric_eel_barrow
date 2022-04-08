#!/bin/bash


# start dbus
echo "starting dbus"
/etc/init.d/dbus start
# start bluetooth daemon
echo "starting bluetooth daemon"
/usr/lib/bluetooth/bluetoothd --debug &
# add user to the bluetooth group
echo "adding root to group: bluetooth"
sudo usermod -G bluetooth root

# the we set up bluetooth connection for all gamepads we know about
# once I take the above section out I can make it loop until something is successful
# MAC_ADDRESSES=("D0:27:88:69:58:9A", "XX:XX:XX:XX:XX:XX")

# for mac_address in ${MAC_ADDRESSES[@]}; do
#     echo "setting up device $mac_address"
#     bluetoothctl disconnect $mac_address
#     bluetoothctl trust $mac_address
#     bluetoothctl pair $mac_address
#     bluetoothctl connect $mac_address
# done
