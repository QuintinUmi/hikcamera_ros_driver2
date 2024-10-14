#!/bin/bash

# Restart NTP
restart_ntp() {
    echo "Restart NTP"
    sudo timedatectl set-ntp true
    echo "Completed."
}

trap restart_ntp EXIT SIGINT SIGTERM


sudo timedatectl set-ntp true
timedatectl  status

echo "Waiting for ntp..."
sleep 2

sudo timedatectl set-ntp false
timedatectl  status
sleep 2

echo "Waiting for ntp close..."

sudo ptpd -S -i eno1 -C
