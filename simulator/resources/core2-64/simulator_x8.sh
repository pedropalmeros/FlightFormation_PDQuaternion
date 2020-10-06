#! /bin/bash

if [ -f /proc/xenomai/version ];then
	EXEC=./SimpleFleet_simulator_rt
else
	EXEC=./SimpleFleet_simulator_nrt
fi

. $FLAIR_ROOT/flair-src/scripts/distribution_specific_hack.sh

$EXEC -n x8 -a 127.0.0.1 -p 9000 -x simulator_x8.xml -o 10 -m $FLAIR_ROOT/flair-src/models -s $FLAIR_ROOT/flair-src/models/indoor_flight_arena.xml -t x8
