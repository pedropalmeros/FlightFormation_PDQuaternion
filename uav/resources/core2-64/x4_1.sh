#! /bin/bash

if [ -f /proc/xenomai/version ];then
	EXEC=./SimpleFleet_rt
else
	EXEC=./SimpleFleet_nrt
fi

. $FLAIR_ROOT/flair-src/scripts/distribution_specific_hack.sh

$EXEC -n x4_1 -a 127.0.0.1 -p 9000 -l ./ -x setup_x4.xml -t x4_simu1 -b 127.255.255.255:20010 -d 21000
