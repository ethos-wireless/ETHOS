#!/bin/bash

cd /opt/ethos-power/ETHOS-power && chmod a+x build.sh && ./build.sh && \
cd /opt/ethos-power/ && \
# ./pmd_test /dev/ttyUSB0 1 && \
./ETHOS-power/build/ETHOS-power -t 60000 |& tee power.log
sleep infinity