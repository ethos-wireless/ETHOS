#include "LAYER2/NR_MAC_gNB/ethos_RIC_shm.h"

bool rl_shm_init = false;
shared_memory_nr_ACTION_t current_action_buf[UE_NUM_SHM_BLOCKS];
uint16_t UE_rnti_list[UE_NUM_SHM_BLOCKS];

void init_nrmac_STATE_shm() {
    LOG_D(NR_MAC,"ethos init_nrmac_STATE_shm.\n");
    for (int i = 0; i < UE_NUM_SHM_BLOCKS; i++) {
        char shm_name[256];
        snprintf(shm_name, sizeof(shm_name), "%s%d", STATE_SHM_NAME_PREFIX, i);
        LOG_D(NR_MAC,"ethos shm_open %s.\n", shm_name);
        int shm_fd = shm_open(shm_name, O_CREAT | O_RDWR, 0666);
        if (shm_fd == -1) {
            LOG_E(NR_MAC,"ethos shm_open error %s.\n", shm_name);
            exit(EXIT_FAILURE);
        }

        LOG_I(NR_MAC,"ethos ftruncat STATE_shm %s STATE_SHM_SIZE %zu.\n", shm_name, STATE_SHM_SIZE);
        if (ftruncate(shm_fd, STATE_SHM_SIZE) == -1) {
            LOG_E(NR_MAC,"ethos ftruncate error %s.\n", shm_name);
            exit(EXIT_FAILURE);
        }

        shared_memory_nr_STATE_t *shared_state = mmap(NULL, STATE_SHM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
        if (shared_state == MAP_FAILED) {
            LOG_E(NR_MAC,"ethos mmap error %s.\n", shm_name);
            exit(EXIT_FAILURE);
        }

        pthread_mutexattr_t attr;
        pthread_mutexattr_init(&attr);
        pthread_mutexattr_setpshared(&attr, PTHREAD_PROCESS_SHARED);
        pthread_mutexattr_setrobust(&attr, PTHREAD_MUTEX_ROBUST);
        pthread_mutex_init(&shared_state->mutex, &attr);
        pthread_mutexattr_destroy(&attr);

        pthread_mutex_lock(&shared_state->mutex);
        shared_state->UE_id = 0;
        shared_state->UE_rnti = 0;
        shared_state->state_time_us = 0.0;
        shared_state->rtt_interval_us = 0.0;
        shared_state->action_read_us = 0.0;
        shared_state->frame = 0;
        shared_state->slot = 0;
        shared_state->dl_ri = 0;
        shared_state->dl_cqi = 0;
        shared_state->dl_total_bytes = 0;
        shared_state->dl_throughput = 0.0f;
        shared_state->dl_bler = 0.0f;
        shared_state->dl_total_waiting_bytes = 0;
        shared_state->dl_rbSize = 0;
        shared_state->dl_mcs = 0;
        shared_state->dl_nrOfLayers = 0;
        shared_state->ul_ri = 0;
        shared_state->ul_cqi = 0;
        shared_state->ul_total_bytes = 0;
        shared_state->ul_throughput = 0.0f;
        shared_state->ul_bler = 0.0f;
        shared_state->ul_total_waiting_bytes = 0;
        shared_state->ul_rbSize = 0;
        shared_state->ul_snr = 0;
        shared_state->ul_mcs = 0;
        shared_state->ul_nrOfLayers = 0;
        pthread_mutex_unlock(&shared_state->mutex);

        LOG_D(NR_MAC, "ethos /nrmac_STATE_ Shared memory block %d mutex size: %zu bytes.", i, MUTEX_SIZE );
        print_raw_data(&shared_state->mutex, MUTEX_SIZE, "STATE_mutex");

        shared_states[i] = shared_state;
        LOG_D(NR_MAC,"ethos /nrmac_STATE_%d mapped to shared_states[%d].\n", i, i);
    }

    LOG_D(NR_MAC,"ethos /nrmac_STATE_ Shared memory blocks initialized.\n");
}

void init_nrmac_ACTION_shm() {
    LOG_D(NR_MAC,"ethos init_nrmac_ACTION_shm.\n");
    for (int i = 0; i < UE_NUM_SHM_BLOCKS; i++) {
        char shm_name[256];
        snprintf(shm_name, sizeof(shm_name), "%s%d", ACTION_SHM_NAME_PREFIX, i);
        LOG_D(NR_MAC,"ethos shm_open %s.\n", shm_name);
        int shm_fd = shm_open(shm_name, O_CREAT | O_RDWR, 0666);
        if (shm_fd == -1) {
            LOG_E(NR_MAC,"ethos shm_open error %s.\n", shm_name);
            exit(EXIT_FAILURE);
        }

        LOG_I(NR_MAC,"ethos ftruncat ACTION_shm %s ACTION_SHM_SIZE %zu.\n", shm_name, ACTION_SHM_SIZE);
        if (ftruncate(shm_fd, ACTION_SHM_SIZE) == -1) {
            LOG_E(NR_MAC,"ethos ftruncate error %s.\n", shm_name);
            exit(EXIT_FAILURE);
        }

        shared_memory_nr_ACTION_t *shared_state = mmap(NULL, ACTION_SHM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
        if (shared_state == MAP_FAILED) {
            LOG_E(NR_MAC,"ethos mmap error %s.\n", shm_name);
            exit(EXIT_FAILURE);
        }

        pthread_mutexattr_t attr;
        pthread_mutexattr_init(&attr);
        pthread_mutexattr_setpshared(&attr, PTHREAD_PROCESS_SHARED);
        pthread_mutexattr_setrobust(&attr, PTHREAD_MUTEX_ROBUST);
        pthread_mutex_init(&shared_state->mutex, &attr);
        pthread_mutexattr_destroy(&attr);

        pthread_mutex_lock(&shared_state->mutex);
        shared_state->UE_id = 0;
        shared_state->UE_rnti = 0;
        shared_state->state_time_us = 0.0;
        shared_state->action_ready_us = 0.0;
        shared_state->frame = 0;
        shared_state->slot = 0;
        shared_state->dl_MCS_value = 0;
        shared_state->dl_layer_use = 0;
        shared_state->ul_MCS_value = 0;
        shared_state->ul_layer_use = 0;
        pthread_mutex_unlock(&shared_state->mutex);

        LOG_D(NR_MAC, "ethos /nrmac_ACTION_ Shared memory block %d mutex size: %zu bytes.", i, MUTEX_SIZE );
        print_raw_data(&shared_state->mutex, MUTEX_SIZE, "ACTION_mutex");

        shared_actions[i] = shared_state;
        LOG_D(NR_MAC,"ethos /nrmac_ACTION_%d mapped to shared_actions[%d].\n", i, i);

        LOG_D(NR_MAC,"ethos initializing current_action_buf[%d].\n", i);
        current_action_buf[i].UE_id = 0;
        current_action_buf[i].UE_rnti = 0;
        current_action_buf[i].state_time_us = 0.0;
        current_action_buf[i].action_ready_us = 0.0;
        current_action_buf[i].frame = 0;
        current_action_buf[i].slot = 0;
        current_action_buf[i].dl_MCS_value = 0;
        current_action_buf[i].dl_layer_use = 0;
        current_action_buf[i].ul_MCS_value = 0;
        current_action_buf[i].ul_layer_use = 0;
    }

    LOG_D(NR_MAC,"ethos /nrmac_ACTION_ Shared memory blocks initialized.\n");
}

void init_shm_cond() {
    LOG_D(NR_MAC,"ethos init_shm_cond.\n");
    for (int i = 0; i < SHM_COND_NUM; i++) {
        char shm_name[256];
        snprintf(shm_name, sizeof(shm_name), "%s%d", SHM_COND_NAME_PREFIX, i);
        LOG_D(NR_MAC,"ethos shm_open %s.\n", shm_name);
        int shm_fd = shm_open(shm_name, O_CREAT | O_RDWR, 0666);
        if (shm_fd == -1) {
            LOG_E(NR_MAC,"ethos shm_open error %s.\n", shm_name);
            exit(EXIT_FAILURE);
        }

        LOG_I(NR_MAC,"ethos ftruncat shm_COND %s SHM_COND_SIZE %zu.\n", shm_name, SHM_COND_SIZE);
        if (ftruncate(shm_fd, SHM_COND_SIZE) == -1) {
            LOG_E(NR_MAC,"ethos ftruncate error %s.\n", shm_name);
            exit(EXIT_FAILURE);
        }

        shared_memory_Condition_t *shared_state = mmap(NULL, SHM_COND_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
        if (shared_state == MAP_FAILED) {
            LOG_E(NR_MAC,"ethos mmap error %s.\n", shm_name);
            exit(EXIT_FAILURE);
        }

        pthread_mutexattr_t attr;
        pthread_mutexattr_init(&attr);
        pthread_mutexattr_setpshared(&attr, PTHREAD_PROCESS_SHARED);
        pthread_mutexattr_setrobust(&attr, PTHREAD_MUTEX_ROBUST);
        pthread_mutex_init(&shared_state->mutex, &attr);
        pthread_mutexattr_destroy(&attr);

        pthread_mutex_lock(&shared_state->mutex);
        shared_state->is_new = false;
        pthread_mutex_unlock(&shared_state->mutex);

        shared_conditions[i] = shared_state;
        LOG_D(NR_MAC,"ethos /shm_cond_%d mapped to shared_conditions[%d].\n", i, i);
    }

    LOG_D(NR_MAC,"ethos /shm_cond_ Shared memory blocks initialized.\n");
}

void init_nrmac_RL_shm() {
    LOG_D(NR_MAC,"ethos init_nrmac_RL_shm.\n");
    init_nrmac_STATE_shm();
    init_nrmac_ACTION_shm();
    init_shm_cond();
}

void write_nrmac_STATE_shm(int block_index, 
    uid_t UE_id, uint16_t UE_rnti, double state_time_us, double rtt_interval_us, double action_read_us, 
    frame_t frame, sub_frame_t slot, 
    uint8_t dl_ri, uint8_t dl_cqi, uint64_t dl_total_bytes, float dl_throughput, 
    float dl_bler, uint32_t dl_total_waiting_bytes, uint16_t dl_rbSize, uint8_t dl_mcs, uint8_t dl_nrOfLayers,
    uint8_t ul_ri, uint8_t ul_cqi, uint64_t ul_total_bytes, float ul_throughput, 
    float ul_bler, uint32_t ul_total_waiting_bytes, uint16_t ul_rbSize, int ul_snr, uint8_t ul_mcs, uint8_t ul_nrOfLayers) {

    LOG_D(NR_MAC,"ethos write_nrmac_STATE_shm.\n");

    if (block_index < 0 || block_index >= UE_NUM_SHM_BLOCKS) {
        LOG_E(NR_MAC,"ethos Invalid block index.\n");
        return;
    }

    char shm_name[256];
    snprintf(shm_name, sizeof(shm_name), "%s%d", STATE_SHM_NAME_PREFIX, block_index);
    LOG_D(NR_MAC,"ethos shm_open %s.\n", shm_name);

    shared_memory_nr_STATE_t *shared_state = shared_states[block_index];
    if (shared_state == NULL) {
        LOG_E(NR_MAC,"ethos shared_state is NULL for block index %d.\n", block_index);
        return;
    }
    start_meas(&rw_shm_stats.timer);
    pthread_mutex_lock(&shared_state->mutex);
    shared_state->UE_id = UE_id;
    shared_state->UE_rnti = UE_rnti;
    shared_state->state_time_us = state_time_us;
    shared_state->rtt_interval_us = rtt_interval_us;
    shared_state->action_read_us = action_read_us;
    shared_state->frame = frame;
    shared_state->slot = slot;
    shared_state->dl_ri = dl_ri;
    shared_state->dl_cqi = dl_cqi;
    shared_state->dl_total_bytes = dl_total_bytes;
    shared_state->dl_throughput = dl_throughput;
    shared_state->dl_bler = dl_bler;
    shared_state->dl_total_waiting_bytes = dl_total_waiting_bytes;
    shared_state->dl_rbSize = dl_rbSize;
    shared_state->dl_mcs = dl_mcs;
    shared_state->dl_nrOfLayers = dl_nrOfLayers;
    shared_state->ul_ri = ul_ri;
    shared_state->ul_cqi = ul_cqi;
    shared_state->ul_total_bytes = ul_total_bytes;
    shared_state->ul_throughput = ul_throughput;
    shared_state->ul_bler = ul_bler;
    shared_state->ul_total_waiting_bytes = ul_total_waiting_bytes;
    shared_state->ul_rbSize = ul_rbSize;
    shared_state->ul_snr = ul_snr;
    shared_state->ul_mcs = ul_mcs;
    shared_state->ul_nrOfLayers = ul_nrOfLayers;
    pthread_mutex_unlock(&shared_state->mutex);
    stop_meas(&rw_shm_stats.timer);

    LOG_D(NR_MAC,"ethos Written to shared STATE memory block %d.\n", block_index);
}

void read_nrmac_ACTION_shm(int block_index) {
    LOG_D(NR_MAC,"ethos read_nrmac_ACTION_shm.\n");
    if (block_index < 0 || block_index >= UE_NUM_SHM_BLOCKS) {
        LOG_E(NR_MAC,"ethos Invalid block index.\n");
        return;
    }

    char shm_name[256];
    snprintf(shm_name, sizeof(shm_name), "%s%d", ACTION_SHM_NAME_PREFIX, block_index);
    LOG_D(NR_MAC,"ethos shm_open %s.\n", shm_name);

    shared_memory_nr_ACTION_t *shared_state = shared_actions[block_index];
    if (shared_state == NULL) {
        LOG_E(NR_MAC,"ethos shared_state is NULL for block index %d.\n", block_index);
        return;
    }

    memcpy(&current_action_buf[block_index], shared_state, ACTION_SHM_SIZE);
    LOG_D(NR_MAC,"ethos Read from shared ACTION memory block %d.\n", block_index);
}

void write_nrmac_shm_cond(int block_index, bool cond) {
    LOG_D(NR_MAC,"ethos write_nrmac_shm_cond.\n");
    if (block_index < 0 || block_index >= SHM_COND_NUM) {
        LOG_E(NR_MAC,"ethos Invalid block index.\n");
        return;
    }

    char shm_name[256];
    snprintf(shm_name, sizeof(shm_name), "%s%d", SHM_COND_NAME_PREFIX, block_index);
    LOG_D(NR_MAC,"ethos shm_open %s.\n", shm_name);

    shared_memory_Condition_t *shared_state = shared_conditions[block_index];
    if (shared_state == NULL) {
        LOG_E(NR_MAC,"ethos shared_state is NULL for block index %d.\n", block_index);
        return;
    }

    pthread_mutex_lock(&shared_state->mutex);
    shared_state->is_new = cond;
    pthread_mutex_unlock(&shared_state->mutex);
    LOG_D(NR_MAC,"ethos Written to and Unlock shared CONDITION memory block %d, is_new %d.\n", block_index, shared_state->is_new);
}

void read_nrmac_shm_cond(int block_index, bool* cond) {
    LOG_D(NR_MAC,"ethos read_nrmac_shm_cond.\n");
    if (block_index < 0 || block_index >= SHM_COND_NUM) {
        LOG_E(NR_MAC,"ethos Invalid block index.\n");
        return;
    }

    char shm_name[256];
    snprintf(shm_name, sizeof(shm_name), "%s%d", SHM_COND_NAME_PREFIX, block_index);
    LOG_D(NR_MAC,"ethos shm_open %s.\n", shm_name);

    shared_memory_Condition_t *shared_state = shared_conditions[block_index];
    if (shared_state == NULL) {
        LOG_E(NR_MAC,"ethos shared_state is NULL for block index %d.\n", block_index);
        return;
    }

    LOG_D(NR_MAC,"ethos Read from shared CONDITION memory (shared_state) block %d, is_new %d.\n", block_index, shared_state->is_new);
    *cond = shared_state->is_new;
    LOG_D(NR_MAC,"ethos updated shared CONDITION memory block %d.\n", block_index);
    LOG_D(NR_MAC,"ethos Read from shared CONDITION memory (shared_state) block %d, *cond input is_new %d.\n", block_index, *cond);
    LOG_D(NR_MAC,"ethos Read from shared CONDITION memory block %d.\n", block_index);
}

void write_gnb_ue_states(frame_t frame, 
                        sub_frame_t slot, 
                        NR_UE_info_t **UE_list) {
    LOG_D(NR_MAC,"ethos write_gnb_ue_states.\n"); 
    write_nrmac_shm_cond(0, false);
    write_nrmac_shm_cond(2, false);
    uint8_t UE_count = 0;
    UE_iterator(UE_list, UE) {
        if (UE_count > UE_NUM_SHM_BLOCKS) {
            LOG_E(NR_MAC,"ethos Invalid block index.\n");
            break;
        }
        uint64_t cpucyclestamp = rdtsc_oai();
        double state_time_us = (double)cpucyclestamp;
        double rtt_interval_us = (double)get_time_meas_us(&rtt_interval_timer.timer);
        double action_read_us = (double)get_time_meas_us(&action_read_timer.timer);
        NR_UE_sched_ctrl_t *sched_ctrl = &UE->UE_sched_ctrl;
        NR_mac_stats_t *stats = &UE->mac_stats;
        uid_t UE_id = UE->uid;
        uint16_t UE_rnti = UE->rnti;
        double thpt_ts_us = (((double) cpucyclestamp) / get_cpu_freq_GHz())/1000.0;
        double thpt_interval_us = thpt_ts_us - UE->ethos_metric_reg.last_timestamp_us;
        UE->ethos_metric_reg.last_timestamp_us = thpt_ts_us;
        double dl_thpt_result = (double)(stats->dl.total_bytes - UE->ethos_metric_reg.last_dl_total_bytes)*8.0/thpt_interval_us;
        UE->ethos_metric_reg.last_dl_total_bytes = stats->dl.total_bytes;
        double ul_thpt_result = (double)(stats->ul.total_bytes - UE->ethos_metric_reg.last_ul_total_bytes)*8.0/thpt_interval_us;
        UE->ethos_metric_reg.last_ul_total_bytes = stats->ul.total_bytes;
        uint8_t dl_cqi = sched_ctrl->CSI_report.cri_ri_li_pmi_cqi_report.wb_cqi_1tb;
        uint8_t dl_ri = sched_ctrl->CSI_report.cri_ri_li_pmi_cqi_report.ri+1;
        float dl_bler = sched_ctrl->dl_bler_stats.bler;
        float dl_throughput = (float)dl_thpt_result;
        uint64_t dl_total_bytes = stats->dl.total_bytes;
        uint32_t dl_total_waiting_bytes = sched_ctrl->num_total_bytes;
        uint16_t dl_rbSize = UE->ethos_metric_reg.dl_rbSize_log;
        UE->ethos_metric_reg.dl_rbSize_log = 0;
        uint8_t dl_mcs = sched_ctrl->dl_bler_stats.mcs;
        uint8_t dl_nrOfLayers = sched_ctrl->sched_pdsch.nrOfLayers;
        uint8_t ul_cqi = sched_ctrl->CSI_report.ul_cqi;
        uint8_t ul_ri = sched_ctrl->srs_feedback.ul_ri + 1;
        float ul_bler = sched_ctrl->ul_bler_stats.bler;
        float ul_throughput = (float)ul_thpt_result;
        uint64_t ul_total_bytes = stats->ul.total_bytes;
        uint32_t ul_total_waiting_bytes = (uint32_t)(sched_ctrl->estimated_ul_buffer);
        uint16_t ul_rbSize = (uint16_t)(stats->NPRB);
        stats->NPRB = 0;
        int ul_snr = sched_ctrl->pusch_snrx10;
        uint8_t ul_mcs = sched_ctrl->ul_bler_stats.mcs;
        uint8_t ul_nrOfLayers = UE->ethos_metric_reg.ul_nrOfLayers;
        write_nrmac_STATE_shm(UE_count, UE_id, UE_rnti, state_time_us, rtt_interval_us, action_read_us, frame, slot, 
                              dl_ri, dl_cqi, dl_total_bytes, dl_throughput, dl_bler, dl_total_waiting_bytes, dl_rbSize, dl_mcs, dl_nrOfLayers,
                              ul_ri, ul_cqi, ul_total_bytes, ul_throughput, ul_bler, ul_total_waiting_bytes, ul_rbSize, ul_snr, ul_mcs, ul_nrOfLayers);
        LOG_D(NR_MAC,"ethos write_gnb_ue_states written the %u UE. \n", UE_count); 
        UE_count++;
    }
    write_nrmac_shm_cond(0, true);
    write_nrmac_shm_cond(2, true);
}

bool read_gnb_ue_actions(NR_UE_info_t **UE_list) {
    LOG_D(NR_MAC,"ethos read_gnb_ue_actions.\n"); 
    bool is_action_new = false;
    read_nrmac_shm_cond(1, &is_action_new);
    if (is_action_new){
        start_meas(&action_read_timer.timer);
        for (uint8_t UE_count = 0;UE_count<UE_NUM_SHM_BLOCKS;UE_count++){
            read_nrmac_ACTION_shm(UE_count);
            UE_rnti_list[UE_count] = current_action_buf[UE_count].UE_rnti;
        }
        write_nrmac_shm_cond(1, false);
        stop_meas(&action_read_timer.timer);
        return true;
    }
    return false;
}

int find_index(uint16_t target) {
    LOG_D(NR_MAC,"ethos find_index %d.\n",target);
    for (int i = 0; i < UE_NUM_SHM_BLOCKS; i++) {
        if (UE_rnti_list[i] == target) {
            return i;
        }
    }
    return -1;
}

double get_action_ready_us_of_current_ue(uint16_t ue_rnti){
    LOG_D(NR_MAC,"ethos get_action_ready_us_of_current_ue.\n");
    int idx = find_index(ue_rnti);
    if (idx >= 0){
        return current_action_buf[idx].action_ready_us;
    }
    else {
        return 0;
    }
}

uint8_t get_dl_mcs_of_current_ue(uint16_t ue_rnti){
    LOG_D(NR_MAC,"ethos get_dl_mcs_of_current_ue.\n");
    int idx = find_index(ue_rnti);
    if (idx >= 0){
        return current_action_buf[idx].dl_MCS_value;
    }
    else {
        return 0;
    }
}

uint8_t get_dl_nrOfLayers_of_current_ue(uint16_t ue_rnti) {
    LOG_D(NR_MAC,"ethos get_dl_nrOfLayers_of_current_ue.\n");
    int idx = find_index(ue_rnti);
    if (idx >= 0){
        return current_action_buf[idx].dl_layer_use;
    }
    else {
        return 0;
    }
}

float get_dl_rbWeight_of_current_ue(uint16_t ue_rnti){
    LOG_D(NR_MAC,"ethos get_dl_rbWeight_of_current_ue.\n");
    int idx = find_index(ue_rnti);
    if (idx >= 0){
        return current_action_buf[idx].dl_rbWeight;
    }
    else {
        return 0;
    }
}

uint8_t get_ul_mcs_of_current_ue(uint16_t ue_rnti){
    LOG_D(NR_MAC,"ethos get_ul_mcs_of_current_ue.\n");
    int idx = find_index(ue_rnti);
    if (idx >= 0){
        return current_action_buf[idx].ul_MCS_value;
    }
    else {
        return 0;
    }
}

uint8_t get_ul_nrOfLayers_of_current_ue(uint16_t ue_rnti) {
    LOG_D(NR_MAC,"ethos get_ul_nrOfLayers_of_current_ue.\n");
    int idx = find_index(ue_rnti);
    if (idx >= 0){
        return current_action_buf[idx].ul_layer_use;
    }
    else {
        return 0;
    }
}

float get_ul_rbWeight_of_current_ue(uint16_t ue_rnti){
    LOG_D(NR_MAC,"ethos get_ul_rbWeight_of_current_ue.\n");
    int idx = find_index(ue_rnti);
    if (idx >= 0){
        return current_action_buf[idx].ul_rbWeight;
    }
    else {
        return 0;
    }
}

void assign_gnb_ue_action(NR_UE_info_t **UE_list) {
    LOG_D(NR_MAC,"ethos assign_gnb_ue_action.\n");
    UE_iterator(UE_list, UE) {
        int idx = find_index(UE->rnti);
        if (idx >= 0){
            NR_UE_sched_ctrl_t *sched_ctrl = &UE->UE_sched_ctrl;
            NR_sched_pdsch_t *sched_pdsch = &sched_ctrl->sched_pdsch;
            sched_pdsch->mcs = current_action_buf[idx].dl_MCS_value;
            sched_pdsch->nrOfLayers = current_action_buf[idx].dl_layer_use;
            NR_sched_pusch_t *sched_pusch = &sched_ctrl->sched_pusch;
            sched_pusch->mcs = current_action_buf[idx].ul_MCS_value;
            sched_pusch->nrOfLayers = current_action_buf[idx].ul_layer_use;
        }
        else {
            NR_UE_sched_ctrl_t *sched_ctrl = &UE->UE_sched_ctrl;
            NR_sched_pdsch_t *sched_pdsch = &sched_ctrl->sched_pdsch;
            sched_pdsch->mcs = current_action_buf[0].dl_MCS_value;
            sched_pdsch->nrOfLayers = current_action_buf[0].dl_layer_use;
            NR_sched_pusch_t *sched_pusch = &sched_ctrl->sched_pusch;
            sched_pusch->mcs = current_action_buf[0].ul_MCS_value;
            sched_pusch->nrOfLayers = current_action_buf[0].ul_layer_use;
        }   
    }
}

bool update_with_rl(frame_t frame, 
                    sub_frame_t slot, 
                    NR_UE_info_t **UE_list) {
    LOG_D(NR_MAC,"ethos update_with_rl.\n");    
    state_time_timer.rdtsc_now = rdtsc_oai();
    state_time_timer.time_now_us = (((double) state_time_timer.rdtsc_now) / get_cpu_freq_GHz())/1000.0;
    start_meas(&rtt_interval_timer.timer);
    if (rl_shm_init == false) {
        init_nrmac_RL_shm();
        rl_shm_init = true;
    }
    write_gnb_ue_states(frame, slot, UE_list);
    bool new_read=read_gnb_ue_actions(UE_list);
    stop_meas(&rtt_interval_timer.timer);
    return new_read;
}

void print_raw_data(void *ptr, size_t size, char *note) {
    LOG_D(NR_MAC, "\n--- Raw Shared Memory Data (Hex) of %s ---", note);
    unsigned char *byte_ptr = (unsigned char *)ptr;
    char hex_line[128];
    int pos = 0;
    for (size_t i = 0; i < size; i++) {
        pos += snprintf(hex_line + pos, sizeof(hex_line) - pos, "%02X ", byte_ptr[i]);
        if ((i + 1) % 16 == 0 || i == size - 1) {
            LOG_D(NR_MAC, "Raw_Mem_Line_%zu %s ", i, hex_line);
            pos = 0;
        }
    }
}

char* get_all_action_buffers() {
    char *log = (char *)malloc(UE_NUM_SHM_BLOCKS * 100);
    if (!log) return NULL;
    log[0] = '\0';
    for (uint8_t i = 0; i < UE_NUM_SHM_BLOCKS; i++) {
        char buffer[100];
        snprintf(buffer, sizeof(buffer), 
                 "UE_id:%u RNTI:%04x Frame:%u Slot:%u dl_MCS:%u dl_Layers:%u ul_MCS:%u ul_Layers:%u ; ",
                 current_action_buf[i].UE_id,
                 current_action_buf[i].UE_rnti,
                 current_action_buf[i].frame,
                 current_action_buf[i].slot,
                 current_action_buf[i].dl_MCS_value,
                 current_action_buf[i].dl_layer_use,
                 current_action_buf[i].ul_MCS_value,
                 current_action_buf[i].ul_layer_use);
        strncat(log, buffer, UE_NUM_SHM_BLOCKS * 100 - strlen(log) - 1);
    }
    return log;
}

rw_stats_t rw_shm_stats;
valid_loop_timer_t loop_ctrl_timer;

void print_rw_stats(frame_t frame, sub_frame_t slot){
    if ((slot == 0) && (frame & 127) == 0) {
        cpumeas(CPUMEAS_ENABLE);
        char log_buffer[512];
        print_meas_log(&rw_shm_stats.timer,"update_with_rl: shm_proc",NULL,NULL, log_buffer, sizeof(log_buffer));
        LOG_I(NR_MAC, "%s", log_buffer);
        reset_meas(&rw_shm_stats.timer);
    }
}

void print_loop_ctrl_timer(frame_t frame, sub_frame_t slot){
    if ((slot == 0) && (frame & 127) == 0) {
        cpumeas(CPUMEAS_ENABLE);
        char log_buffer[512];
        print_meas_log(&loop_ctrl_timer.timer,"update_with_rl: external_controller_loop",NULL,NULL, log_buffer, sizeof(log_buffer));
        LOG_I(NR_MAC, "%s", log_buffer);
    }
}

void update_ethos_ue_metric_reg(frame_t frame, sub_frame_t slot, NR_UE_info_t **UE_list){
    UE_iterator(UE_list, UE) {
        UE->ethos_metric_reg.last_frame = frame;
        UE->ethos_metric_reg.last_slot = slot;
        if (UE->UE_sched_ctrl.sched_pusch.nrOfLayers > 0) {
            UE->ethos_metric_reg.ul_nrOfLayers = UE->UE_sched_ctrl.sched_pusch.nrOfLayers;
        }
        UE->ethos_metric_reg.moving_averaged_dl_total_waiting_bytes = 0.95 * UE->ethos_metric_reg.moving_averaged_dl_total_waiting_bytes + 0.05 * UE->UE_sched_ctrl.num_total_bytes;
        UE->ethos_metric_reg.moving_averaged_ul_total_waiting_bytes = 0.95 * UE->ethos_metric_reg.moving_averaged_ul_total_waiting_bytes + 0.05 * (uint32_t)(UE->UE_sched_ctrl.estimated_ul_buffer);
    }
}

valid_loop_timer_t state_time_timer;
valid_loop_timer_t rtt_interval_timer;
valid_loop_timer_t action_read_timer;
