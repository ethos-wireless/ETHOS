#! /bin/bash

serverVendorAndModel=$(cat /sys/devices/virtual/dmi/id/board_vendor)
serverVendorAndModel+="-"
serverVendorAndModel+=$(cat /sys/devices/virtual/dmi/id/board_name)
echo $serverVendorAndModel
case $serverVendorAndModel in
"Dell Inc.-06V45N")
        argument="P5G_FXN_R750"
    ;;
"GIGABYTE-MU71-SU0-00")
        argument="P5G_FXN"
        cp ./docker/Dockerfile.gNB.aerial.ubuntu20.ethos.GIGA /home/aerial/openairinterface5g/docker/
        pushd /home/aerial/openairinterface5g/
        docker build . -f docker/Dockerfile.gNB.aerial.ubuntu20.ethos.GIGA --tag oai-gnb-aerial:ethos-dev
        popd
    ;;
    "Supermicro-G1SMH-G")
        argument="P5G_FXN_GH"
        cp ./docker/Dockerfile.gNB.aerial.ubuntu22.ethos.GH /home/aerial/openairinterface5g/docker/
        pushd /home/aerial/openairinterface5g/
        docker build . -f docker/Dockerfile.gNB.aerial.ubuntu22.ethos.GH --tag oai-gnb-aerial-ethos:dev
        popd
    ;;
*)
    echo "Unrecognized server: $serverVendorAndModel"
    exit
    ;;
esac