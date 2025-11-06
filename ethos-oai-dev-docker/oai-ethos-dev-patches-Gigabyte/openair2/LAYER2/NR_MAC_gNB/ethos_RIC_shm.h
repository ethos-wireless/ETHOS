#ifndef ETHOS_RIC_SHM_H
#define ETHOS_RIC_SHM_H

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <string.h>
#include <pthread.h>
#include "nr_mac_gNB.h"

#include "common/utils/nr/nr_common.h"
#include "common/ran_context.h"
#include "executables/softmodem-common.h"

#define STATE_SHM_NAME_PREFIX "/nrmac_STATE_"
#define STATE_SHM_SIZE sizeof(shared_memory_nr_STATE_t)
#define ACTION_SHM_NAME_PREFIX "/nrmac_ACTION_"
#define ACTION_SHM_SIZE sizeof(shared_memory_nr_ACTION_t)
#define SHM_COND_NAME_PREFIX "/shm_cond_"
#define SHM_COND_SIZE sizeof(shared_memory_Condition_t)
#define MUTEX_SIZE sizeof (pthread_mutex_t)

#define SHM_COND_NUM 3  // 0 is for STATE, 1 is for ACTION, 2 is for PROFILNG 
#define UE_NUM_SHM_BLOCKS 8

// bool rl_shm_init = false;
// bool rl_new_read = false;
#pragma pack(1)
typedef struct {
    pthread_mutex_t mutex;              // assume 40 bytes-Intel 48 bytes-ARM
    bool is_new;                        // 1 byte
} shared_memory_Condition_t;            // 41 bytes struct
#pragma pack()

#pragma pack(1)
typedef struct {
    //----mutex and overall UE info --- 78 bytes
    pthread_mutex_t mutex;              // assume 40 bytes-Intel 48 bytes-ARM
    uid_t UE_id;                        // 4 bytes - unsigned int
    uint16_t UE_rnti;                   // 2 bytes
    double state_time_us;                 // 8 bytes    //get_time_meas_us ()             // may not need
    double rtt_interval_us;                 // 8 bytes    //get_time_meas_us ()
    double action_read_us;                 // 8 bytes    //get_time_meas_us ()
    frame_t frame;                      // 4 bytes - uint32_t
    sub_frame_t slot;                   // 4 bytes - uint32_t 
    //----DLSCH---- 26 bytes ---
    uint8_t dl_ri;                         // 1 byte
    uint8_t dl_cqi;                        // 1 byte
    uint64_t dl_total_bytes;            // 8 bytes   stats->dl.total_bytes
    float dl_throughput;                   // 4 bytes
    float dl_bler;                         // 4 bytes
    uint32_t dl_total_waiting_bytes;       // 4 bytes  sched_ctrl->num_total_bytes  total bytes waiting in the DL RLC buffer
    uint16_t dl_rbSize;                     // 2 bytes
    uint8_t dl_mcs;                        // 1 byte
    uint8_t dl_nrOfLayers;                  // 1 byte
    //----ULSCH---- 30 bytes ---
    uint8_t ul_ri;                         // 1 byte
    uint8_t ul_cqi;                        // 1 byte
    uint64_t ul_total_bytes;            // 8 bytes   stats->dl.total_bytes
    float ul_throughput;                   // 4 bytes
    float ul_bler;                         // 4 bytes
    uint32_t ul_total_waiting_bytes;       // 4 bytes  sched_ctrl->num_total_bytes  total bytes waiting in the DL RLC buffer
    uint16_t ul_rbSize;                     // 2 bytes
    int ul_snr;                             // 4 byte
    uint8_t ul_mcs;                        // 1 byte
    uint8_t ul_nrOfLayers;                  // 1 byte
} shared_memory_nr_STATE_t;             // 134 bytes total struct
#pragma pack()
// int shm_state_fds[UE_NUM_SHM_BLOCKS];
// shared_memory_nr_STATE_t *shared_states[UE_NUM_SHM_BLOCKS];
#pragma pack(1)
typedef struct {
    //----mutex and overall UE info --- 70 bytes
    pthread_mutex_t mutex;              // assume 40 bytes-Intel 48 bytes-ARM
    uid_t UE_id;                        // 4 bytes - unsigned int
    uint16_t UE_rnti;                   // 2 bytes
    double state_time_us;                 // 8 bytes    //get_time_meas_us ()             // may not need
    double action_ready_us;                 // 8 bytes
    frame_t frame;                      // 4 bytes - uint32_t
    sub_frame_t slot;                   // 4 bytes - uint32_t
    // --- DLSCH --- 2 bytes ---
    uint8_t dl_MCS_value;                  // 1 byte
    uint8_t dl_layer_use;                  // 1 byte
    float dl_rbWeight;                  // 4 bytes
    // --- ULSCH --- 2 bytes ---
    uint8_t ul_MCS_value;                  // 1 byte
    uint8_t ul_layer_use;                  // 1 byte
    float ul_rbWeight;                  // 4 bytes
} shared_memory_nr_ACTION_t;            // 82 bytes total struct
#pragma pack()

// int shm_action_fds[UE_NUM_SHM_BLOCKS];
// shared_memory_nr_ACTION_t *current_action_buf[UE_NUM_SHM_BLOCKS];
// uint16_t UE_rnti_list[UE_NUM_SHM_BLOCKS];

/* ----- struct lists for shared memory pointers ----- */
shared_memory_nr_STATE_t *shared_states[UE_NUM_SHM_BLOCKS]; // shared memory pointers for UE states
shared_memory_nr_ACTION_t *shared_actions[UE_NUM_SHM_BLOCKS]; // shared memory pointers for UE actions
shared_memory_Condition_t *shared_conditions[SHM_COND_NUM]; // shared memory pointers for UE shm condition 



// typedef struct {
//   /// scheduling control info
//   // last element always NULL
//   pthread_mutex_t mutex;
//   NR_UE_info_t *list[MAX_MOBILES_PER_GNB+1];
//   // bitmap of CSI-RS already scheduled in current slot
//   int sched_csirs;
//   uid_allocator_t uid_allocator;
// } NR_UEs_t;

// #define UE_iterator(BaSe, VaR) NR_UE_info_t ** VaR##pptr=BaSe, *VaR; while ((VaR=*(VaR##pptr++)))

void init_nrmac_RL_shm();
void init_nrmac_STATE_shm();
void init_nrmac_ACTION_shm();
void init_shm_cond();

void write_nrmac_STATE_shm(int block_index, 
    uid_t UE_id, uint16_t UE_rnti, double state_time_us, double rtt_interval_us, double action_read_us, 
    frame_t frame, sub_frame_t slot, uint8_t dl_ri, uint8_t dl_cqi, uint64_t dl_total_bytes, float dl_throughput, 
    float dl_bler, uint32_t dl_total_waiting_bytes, uint16_t dl_rbSize, uint8_t dl_mcs, uint8_t dl_nrOfLayers,
    uint8_t ul_ri, uint8_t ul_cqi, uint64_t ul_total_bytes, float ul_throughput, 
    float ul_bler, uint32_t ul_total_waiting_bytes, uint16_t ul_rbSize, int ul_snr, uint8_t ul_mcs, uint8_t ul_nrOfLayers);
void read_nrmac_ACTION_shm(int block_index);
void write_nrmac_shm_cond(int block_index, bool cond);
void read_nrmac_shm_cond(int block_index, bool* cond);

void write_gnb_ue_states(frame_t frame, 
                        sub_frame_t slot, 
                        NR_UE_info_t **UE_list);
bool read_gnb_ue_actions(NR_UE_info_t **UE_list);
int find_index(uint16_t target);
double get_action_ready_us_of_current_ue(uint16_t ue_rnti);
uint8_t get_dl_mcs_of_current_ue(uint16_t ue_rnti);
uint8_t get_dl_nrOfLayers_of_current_ue(uint16_t ue_rnti);
float get_dl_rbWeight_of_current_ue(uint16_t ue_rnti);
uint8_t get_ul_mcs_of_current_ue(uint16_t ue_rnti);
uint8_t get_ul_nrOfLayers_of_current_ue(uint16_t ue_rnti);
float get_ul_rbWeight_of_current_ue(uint16_t ue_rnti);
void assign_gnb_ue_action(NR_UE_info_t **UE_list);      // only used for debugging purpose (not used in current OAI)
bool update_with_rl(frame_t frame, 
                    sub_frame_t slot, 
                    NR_UE_info_t **UE_list);
void print_raw_data(void *ptr, size_t size, char *note);
char* get_all_action_buffers();             // only used for debugging purpose (not used in current OAI)


typedef struct{
    time_stats_t timer;              // the latency of one R/W round of shm IPC (all UEs)
    uint64_t tries;
    uint64_t num_hit;
    uint64_t num_read_none;
    uint64_t num_read_late;
    float hit_rate;
}rw_stats_t;                    // shm read and write stats, measure the hit rate of the rw

typedef struct{
    frame_t frame;
    sub_frame_t slot;
    time_stats_t timer;          // the latency of one complete loop with RL and grabbing the results (all UE slot level)
    uint64_t rdtsc_now;      // the rdtsc value (CPU cycle count) of the timer
    double time_now_us;     // the time in us of the timer
}valid_loop_timer_t;


extern rw_stats_t rw_shm_stats;
extern valid_loop_timer_t loop_ctrl_timer;

extern valid_loop_timer_t state_time_timer;         // use its rdtsc_now to measure the time of the state write (overall UE)
extern valid_loop_timer_t rtt_interval_timer;       // use its timer to measure the RTT time (ETHOS W+R in OAI, regardless of blocking or non-blocking)
extern valid_loop_timer_t action_read_timer;        // use its timer to measure the action read time (ETHOS R in OAI), for further calc "action_ready_us" + "action_read"

void print_rw_stats(frame_t frame, sub_frame_t slot);
void print_loop_ctrl_timer(frame_t frame, sub_frame_t slot);

// typedef struct{
//     frame_t last_frame;                                 // used external -- update every slot
//     sub_frame_t last_slot;                              // used external -- update every slot
//     double last_measure_time_us;                        // used internal -- update every state_write
//     uint8_t ul_nrOfLayers;                              // used external -- update every slot
//     uint32_t moving_averaged_dl_total_waiting_bytes;    // used external -- update every slot
//     uint32_t moving_averaged_ul_total_waiting_bytes;    // used external -- update every slot
// }ue_metric_reg_t;

// update the external used metric every slot in gNB_scheduler after ulsch and dlsch
void update_ethos_ue_metric_reg(frame_t frame, sub_frame_t slot, NR_UE_info_t **UE_list);


#endif // ETHOS_RIC_SHM_H

// static inline void print_rw_stats(frame_t frame, sub_frame_t slot){
//     if ((slot == 0) && (frame & 127) == 0) {
//         cpu_meas_enabled = 1;
//         char log_buffer[512];
//         print_meas_log(&rw_shm_stats.timer,"update_with_rl: shm_proc",NULL,NULL, log_buffer, sizeof(log_buffer));
//         LOG_I(NR_MAC, "%s", log_buffer);
//     }
// }

// static inline void print_loop_ctrl_timer(frame_t frame, sub_frame_t slot){
//     if ((slot == 0) && (frame & 127) == 0) {
//         cpu_meas_enabled = 1;
//         char log_buffer[512];
//         print_meas_log(&loop_ctrl_timer.timer,"update_with_rl: external_controller_loop",NULL,NULL, log_buffer, sizeof(log_buffer));
//         LOG_I(NR_MAC, "%s", log_buffer);
//     }
// }
