#!/bin/bash
cp -R /opt/oai-ethos-dev-patches-Gigabyte/openair2/LAYER2/NR_MAC_gNB/main.c /opt/oai-ran-dev-ethos-GIGABYTE/openair2/LAYER2/NR_MAC_gNB/main.c
cp -R /opt/oai-ethos-dev-patches-Gigabyte/openair2/LAYER2/NR_MAC_gNB/ethos_RIC_shm.c /opt/oai-ran-dev-ethos-GIGABYTE/openair2/LAYER2/NR_MAC_gNB/ethos_RIC_shm.c
cp -R /opt/oai-ethos-dev-patches-Gigabyte/openair2/LAYER2/NR_MAC_gNB/ethos_RIC_shm.h /opt/oai-ran-dev-ethos-GIGABYTE/openair2/LAYER2/NR_MAC_gNB/ethos_RIC_shm.h
cp -R /opt/oai-ethos-dev-patches-Gigabyte/openair2/LAYER2/NR_MAC_gNB/gNB_scheduler_dlsch.c /opt/oai-ran-dev-ethos-GIGABYTE/openair2/LAYER2/NR_MAC_gNB/gNB_scheduler_dlsch.c
cp -R /opt/oai-ethos-dev-patches-Gigabyte/openair2/LAYER2/NR_MAC_gNB/gNB_scheduler.c /opt/oai-ran-dev-ethos-GIGABYTE/openair2/LAYER2/NR_MAC_gNB/gNB_scheduler.c
cp -R /opt/oai-ethos-dev-patches-Gigabyte/openair2/LAYER2/NR_MAC_gNB/gNB_scheduler_ulsch.c /opt/oai-ran-dev-ethos-GIGABYTE/openair2/LAYER2/NR_MAC_gNB/gNB_scheduler_ulsch.c
cp -R /opt/oai-ethos-dev-patches-Gigabyte/openair2/LAYER2/NR_MAC_gNB/nr_mac_gNB.h /opt/oai-ran-dev-ethos-GIGABYTE/openair2/LAYER2/NR_MAC_gNB/nr_mac_gNB.h

cp -R /opt/oai-ethos-dev-patches-Gigabyte/openair2/NR_PHY_INTERFACE/NR_IF_Module.c /opt/oai-ran-dev-ethos-GIGABYTE/openair2/NR_PHY_INTERFACE/NR_IF_Module.c

CMAKE_FILE="/opt/oai-ran-dev-ethos-GIGABYTE/CMakeLists.txt"

# Define the substring to match and the line to insert
MATCH_SUBSTRING="gNB_scheduler_dlsch"
# INSERT_LINE='${NR_GNB_MAC_DIR}/ethos_RIC_shm.c'

sed -i "/${MATCH_SUBSTRING}/a \\
\$\{NR_GNB_MAC_DIR\}\/ethos_RIC_shm.c" "$CMAKE_FILE"

echo "Updated CMakeLists.txt."

cd /opt/oai-ran-dev-ethos-GIGABYTE
/bin/sh oaienv && \
cd cmake_targets && \
mkdir -p log && \
./build_oai -c -w AERIAL --gNB --ninja --cmake-opt -DCMAKE_C_FLAGS="-Werror" --cmake-opt -DCMAKE_CXX_FLAGS="-Werror"

cp /opt/oai-ran-dev-ethos-GIGABYTE/cmake_targets/ran_build/build/nr-softmodem /opt/oai-gnb/bin/nr-softmodem-RL
