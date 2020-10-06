#! /bin/bash

if [ -f /proc/xenomai/version ];then
	EXEC=./MultiAgent_simulator_rt
else
	EXEC=./MultiAgent_simulator_nrt
fi

. $FLAIR_ROOT/flair-src/scripts/distribution_specific_hack.sh

$EXEC -n x4 -a 127.0.0.1 -p 9000 -x simulator_x4.xml -o 10 -m $FLAIR_ROOT/flair-src/models -s $FLAIR_ROOT/flair-src/models/indoor_flight_arena.xml -t x4
