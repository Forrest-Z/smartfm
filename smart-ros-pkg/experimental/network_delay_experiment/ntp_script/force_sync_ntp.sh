#!/bin/bash

# This script syncs the clock with the golfcart-clone. It actually stops the
# chrony daemon and sync the clock with ntpdate, then restarts the daemon.
# This is a bit harsh but it's usefull when the offset is so large that it
# would take chrony hours to converge...

offset=`ntpdate -q golfcart-clone | awk '$1 ~ ntpdate { offset=$10 } END { print offset }'`
echo offset is $offset. Synchronizing...

if [ `id -u` -eq 0 ]
then

(
service chrony stop
ntpdate golfcart-clone
service chrony start
) > /dev/null 2>&1

else
        echo "You must have administrator privileges (sudo) to sync! Aborting."
        exit 1
fi

offset=`ntpdate -q golfcart-clone | awk '$1 ~ ntpdate { offset=$10 } END { print offset }'`
echo ... offset is now $offset.
exit 0
