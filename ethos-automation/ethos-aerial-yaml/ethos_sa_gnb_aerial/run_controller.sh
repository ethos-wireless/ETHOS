#!/bin/bash

# Check if cuBB_SDK is defined, if not, use default path
cuBB_Path="${cuBB_SDK:-/opt/nvidia/cuBB}"


cd "$cuBB_Path" || exit 1
# Add gdrcopy to LD_LIBRARY_PATH
export LD_LIBRARY_PATH=/usr/local/nvidia/lib:/usr/local/nvidia/lib64:/opt/mellanox/dpdk/lib/x86_64-linux-gnu:/opt/mellanox/doca/lib/x86_64-linux-gnu:/opt/nvidia/cuBB/cuPHY-CP/external/gdrcopy/build/x86_64/

# Restart MPS
# Export variables
export CUDA_DEVICE_MAX_CONNECTIONS=8
export CUDA_MPS_PIPE_DIRECTORY=/var
export CUDA_MPS_LOG_DIRECTORY=/var

# Stop existing MPS
echo quit | nvidia-cuda-mps-control

# Start MPS
sudo -E nvidia-cuda-mps-control -d
sudo -E echo start_server -uid 0 | sudo -E nvidia-cuda-mps-control

# export CUDA_VISIBLE_DEVICES=0
# export CUDA_MPS_ACTIVE_THREAD_PERCENTAGE=60        # Only allow cuBB to use 60% of SMs
# export CUDA_MPS_PINNED_DEVICE_MEM_LIMIT_MB=2048    # (Optional) Limit pinned memory

# Start cuphycontroller_scf
# Check if an argument is provided
if [ $# -eq 0 ]; then
    # No argument provided, use default value
    argument="P5G_SCF_FXN"
else
    # Argument provided, use it
    argument="$1"
fi

sed -i "s/ nic:.*/ nic: 0000:b5:00.0/" ${cuBB_SDK}/cuPHY-CP/cuphycontroller/config/cuphycontroller_P5G_FXN.yaml
sed -i "s/ dst_mac_addr:.*/ dst_mac_addr: 6c:ad:ad:00:09:40/" ${cuBB_SDK}/cuPHY-CP/cuphycontroller/config/cuphycontroller_P5G_FXN.yaml
# sed -i "s/ dst_mac_addr:.*/ dst_mac_addr: 6c:ad:ad:00:0b:fc/" ${cuBB_SDK}/cuPHY-CP/cuphycontroller/config/cuphycontroller_P5G_FXN.yaml

sed -i "s/ mps_sm_pusch:.*/ mps_sm_pusch: 108/" ${cuBB_SDK}/cuPHY-CP/cuphycontroller/config/cuphycontroller_P5G_FXN.yaml      # pusch default mps_sm_pusch: 108
sed -i "s/ mps_sm_pdsch:.*/ mps_sm_pdsch: 82/" ${cuBB_SDK}/cuPHY-CP/cuphycontroller/config/cuphycontroller_P5G_FXN.yaml      # pdsch default mps_sm_pdsch: 82
#   mps_sm_pusch: 108
#   mps_sm_pucch: 16
#   mps_sm_prach: 16
#   mps_sm_pdsch: 82
#   mps_sm_pdcch: 28
#   mps_sm_pbch: 14
#   mps_sm_srs: 16
#   pdsch_fallback: 0
sed -i "s/ mps_sm_pucch:.*/ mps_sm_pucch: 16/" ${cuBB_SDK}/cuPHY-CP/cuphycontroller/config/cuphycontroller_P5G_FXN.yaml      # pucch default mps_sm_pucch: 16
sed -i "s/ mps_sm_prach:.*/ mps_sm_prach: 16/" ${cuBB_SDK}/cuPHY-CP/cuphycontroller/config/cuphycontroller_P5G_FXN.yaml      # prach default mps_sm_prach: 16
sed -i "s/ mps_sm_pdcch:.*/ mps_sm_pdcch: 28/" ${cuBB_SDK}/cuPHY-CP/cuphycontroller/config/cuphycontroller_P5G_FXN.yaml      # pdcch default mps_sm_pdcch: 28
sed -i "s/ mps_sm_pbch:.*/ mps_sm_pbch: 14/" ${cuBB_SDK}/cuPHY-CP/cuphycontroller/config/cuphycontroller_P5G_FXN.yaml      # pbch default mps_sm_pbch: 14
sed -i "s/ mps_sm_srs:.*/ mps_sm_srs: 16/" ${cuBB_SDK}/cuPHY-CP/cuphycontroller/config/cuphycontroller_P5G_FXN.yaml      # srs default mps_sm_srs: 16
# sed -i "s/ pdsch_fallback:.*/ pdsch_fallback: 0/" ${cuBB_SDK}/cuPHY-CP/cuphycontroller/config/cuphycontroller_P5G_FXN.yaml      # pdsch_fallback default pdsch_fallback: 0

# echo $cuBB_Path
#sudo -E nsys profile --gpu-metrics-device=all -o cuphyProfile --force-overwrite=true "$cuBB_Path"/build/cuPHY-CP/cuphycontroller/examples/cuphycontroller_scf P5G_FXN 
sudo -E "$cuBB_Path"/build/cuPHY-CP/cuphycontroller/examples/cuphycontroller_scf P5G_FXN
#sudo ./build/cuPHY-CP/gt_common_libs/nvIPC/tests/pcap/pcap_collect
#sudo mv nvipc.pcap /var/log/aerial/
#sleep infinity
