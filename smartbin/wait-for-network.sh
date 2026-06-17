#!/bin/bash

TARGET_SSID=TheBlutotDevice
until [ "$(iwgetid -r)" = "$TARGET_SSID" ]; do
	sleep 3
done

wait-for-network