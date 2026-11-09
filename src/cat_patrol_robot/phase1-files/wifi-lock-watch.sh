#!/bin/bash
dbus-monitor --session "type='signal',interface='org.gnome.SessionManager',member='PropertiesChanged'" | \
while read line; do
    if echo "$line" | grep -q "SessionIsActive"; then
        read nextline
        read value
        if echo "$value" | grep -q "boolean true"; then
            sleep 1
            nmcli con up "TP-Link-Bot-Net-High"
        elif echo "$value" | grep -q "boolean false"; then
            nmcli con down "TP-Link-Bot-Net-High"
        fi
    fi
done
