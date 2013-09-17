#!/bin/sh

dtc -O dtb -o BB-SMART-FM-00A0.dtbo -b 0 -@ BB-SMART-FM-00A0.dts && \
cp BB-SMART-FM-00A0.dtbo /lib/firmware/


