#include <TrackerScheduler.h>


TrackerScheduler::TrackerScheduler()
    : active_(true){
    // cpu_tracker_ = CpuTracker();
    // gpu_tracker_ = GpuTracker();
    // pmd_tracker_ = PmdTracker(2,"","/dev/ttyUSB0",115200u);
    cpu_result_ = {"cpu_power",0,0.0,std::chrono::high_resolution_clock::now(),std::chrono::seconds(1)};
    gpu_result_ = {"gpu_power",0,0.0,std::chrono::high_resolution_clock::now(),std::chrono::seconds(1)};
    ru_result_ = {"oru_power",0,0.0,std::chrono::high_resolution_clock::now(),std::chrono::seconds(1)};
    InitializePowerShm();
    Enable();
}

TrackerScheduler::~TrackerScheduler() {
    Disable();
    UnlinkShm();
}


std::string TrackerScheduler::GetTimestampUs (std::chrono::time_point<std::chrono::high_resolution_clock> time_now) const {
    
    using namespace std::chrono;

    // auto now = system_clock::now();
    auto duration_since_epoch = time_now.time_since_epoch();
    auto micros = duration_cast<microseconds>(duration_since_epoch) % 1000000;
    
    std::time_t time_point = high_resolution_clock::to_time_t(time_now);
    std::tm tm_time = *std::localtime(&time_point);

    std::ostringstream oss;
    oss << std::put_time(&tm_time, "%Y-%m-%d %H:%M:%S")
        << "." << std::setw(6) << std::setfill('0') << micros.count(); // Display microseconds
    return oss.str();
}

void TrackerScheduler::PrintMeasuredPower(std::chrono::duration<uint64_t,std::milli> interval){
    using namespace std::chrono;
    auto next_time_point = high_resolution_clock::now();
    uint32_t call_count = 0;
    float energy_before = cpu_tracker_.GetTrackEnergyUj();
    if (energy_before < 0) {
        return;
    }
    while (active_) {
        auto now = high_resolution_clock::now();
        // auto elapsed = duration_cast<microseconds>(now - next_time_point).count();
        // Calculate the time to sleep to ensure 1ms interval
        next_time_point += interval;  // Move to the next 1ms interval
        std::this_thread::sleep_until(next_time_point);


        auto readStartTime = high_resolution_clock::now();
        float energy_after = cpu_tracker_.GetTrackEnergyUj();
        if (energy_after < 0) {
            return;
        }
        float energy_diff_uj = energy_after - energy_before;
        float energy_diff_joules = energy_diff_uj / 1e6;
        std::chrono::milliseconds milliInterval = std::chrono::duration_cast<std::chrono::milliseconds>(interval);
        uint64_t milliCount = milliInterval.count();
        float cpu_power_watts = energy_diff_joules / (milliCount*1e-3);


        float gpu_power_watts = (gpu_tracker_.GetDeviceTrackPowerMw(0) / 1e-3);

        std::string timestamp = GetTimestampUs(now);

        auto elapseReadDuration = duration_cast<microseconds>(high_resolution_clock::now() - readStartTime).count();

        std::cout << std::fixed << std::setprecision(6)
                  << "Timestamp: " << timestamp << ", "
                  << "Call " << ++call_count << " - read time: "
                  << elapseReadDuration << " us, CPU Power: "
                  << cpu_power_watts << " W, CPU Energy: "
                  << energy_diff_joules << " Joules, GPU power: "
                  << gpu_power_watts << "W"
                  << std::endl;

        energy_before = energy_after;
    }
    
}

void TrackerScheduler::MeasureCpuPower(std::chrono::duration<uint64_t,std::milli> interval) {

    using namespace std::chrono;
    uint32_t call_count = 0;
    auto call_previous_func_time = high_resolution_clock::now();
    auto next_time_point = call_previous_func_time+interval;
    float energy_previous = cpu_tracker_.GetTrackEnergyUj();
    std::this_thread::sleep_until(next_time_point);
    float energy_now = cpu_tracker_.GetTrackEnergyUj();
    float energy_diff_uj = 0;

    while (active_) {
        // cpu calc power with enerygy and actual interval
        auto call_func_time = high_resolution_clock::now();
        next_time_point += interval;  // Move to the next interval
        energy_now = cpu_tracker_.GetTrackEnergyUj();
        energy_diff_uj = energy_now - energy_previous;
        auto after_func_time = high_resolution_clock::now();
        std::chrono::microseconds actual_interval_us = 
            std::chrono::duration_cast<std::chrono::microseconds>(call_func_time-call_previous_func_time);
        uint64_t actual_interval_count_us = actual_interval_us.count();
        float cpu_power_watts = energy_diff_uj / static_cast<float>(actual_interval_count_us);
        call_previous_func_time = call_func_time;
        energy_previous = energy_now;
        // check whether exec time exceeds interval
        std::chrono::microseconds func_exec_duration =
            std::chrono::duration_cast<std::chrono::microseconds>(after_func_time-call_func_time);
        uint64_t func_exec_duration_us_count = func_exec_duration.count();
        uint64_t interval_us_count = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(interval).count());
        if (func_exec_duration_us_count > interval_us_count) {
            std::cout << __PRETTY_FUNCTION__ << ": func_exec_duration_us_count exceed interval - func_exec_duration_us_count: "
            << func_exec_duration_us_count << " us , interval: "
            << interval_us_count << " us" << std::endl;
        }

        call_count++;
        // result format : 
        // {std::string dev_type; uint32_t call_cnt;float power_result;std::time_t measure_timestamp;std::chrono::duration<uint64_t,std::micro> current_interval;
        cpu_result_ = {"cpu_power",call_count,cpu_power_watts,call_func_time,actual_interval_us};

        // // Print results for debugging
        // std::string timestamp = GetTimestampUs(call_func_time);
        // std::cout << std::fixed << std::setprecision(6)
        //           << "Measured Type: " << cpu_result_.dev_type << " , "
        //           << "Func Timestamp: " << GetTimestampUs(cpu_result_.measure_timestamp) << " , "
        //           << "Call " << cpu_result_.call_cnt << " , " 
        //           << "Exec Time: " << cpu_result_.current_interval_us.count() << " us ,"
        //           << "CPU Power: " << cpu_result_.power_result << " W , "
        //           << std::endl;

        std::this_thread::sleep_until(next_time_point);
    }

}

void TrackerScheduler::MeasureGpuPower(std::chrono::duration<uint64_t,std::milli> interval) {
    using namespace std::chrono;
    uint32_t call_count = 0;
    auto call_previous_func_time = high_resolution_clock::now();
    auto next_time_point = call_previous_func_time+interval;
    float power_now_mw = gpu_tracker_.GetDeviceTrackPowerMw(0);
    std::this_thread::sleep_until(next_time_point);

    while (active_) {
        // gpu directly call power and get actual interval
        auto call_func_time = high_resolution_clock::now();
        next_time_point += interval;  // Move to the next interval
        power_now_mw = gpu_tracker_.GetDeviceTrackPowerMw(0);
        float power_now_watts = (power_now_mw * static_cast<float>(1e-3));
        auto after_func_time = high_resolution_clock::now();
        std::chrono::microseconds actual_interval_us = 
            std::chrono::duration_cast<std::chrono::microseconds>(call_func_time-call_previous_func_time);
        // uint64_t actual_interval_count_us = actual_interval_us.count();
        call_previous_func_time = call_func_time;
        // check whether exec time exceeds interval
        std::chrono::microseconds func_exec_duration =
            std::chrono::duration_cast<std::chrono::microseconds>(after_func_time-call_func_time);
        uint64_t func_exec_duration_us_count = func_exec_duration.count();
        uint64_t interval_us_count = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(interval).count());
        if (func_exec_duration_us_count > interval_us_count) {
            std::cout << __PRETTY_FUNCTION__  << ": func_exec_duration_us_count exceed interval - func_exec_duration_us_count: "
            << func_exec_duration_us_count << " us , interval: "
            << interval_us_count << " us" << std::endl;
        }

        call_count++;

        // result format : 
        // {std::string dev_type; uint32_t call_cnt;float power_result;std::time_t measure_timestamp;std::chrono::duration<uint64_t,std::micro> current_interval;
        gpu_result_ = {"gpu_power",call_count,power_now_watts,call_func_time,actual_interval_us};

        // // Print results for debugging
        // std::string timestamp = GetTimestampUs(call_func_time);
        // std::cout << std::fixed << std::setprecision(6)
        //           << "Measured Type: " << gpu_result_.dev_type << " , "
        //           << "Func Timestamp: " << GetTimestampUs(gpu_result_.measure_timestamp) << " , "
        //           << "Call Idx" << gpu_result_.call_cnt << " , " 
        //           << "Exec Time: " << gpu_result_.current_interval_us.count() << " us ,"
        //           << "GPU Power: " << gpu_result_.power_result << " W , "
        //           << std::endl;

        std::this_thread::sleep_until(next_time_point);
    }
}

void TrackerScheduler::MeasureRuPower(std::chrono::duration<uint64_t,std::milli> interval) {
    using namespace std::chrono;
    uint32_t call_count = 0;
    auto call_previous_func_time = high_resolution_clock::now();
    auto next_time_point = call_previous_func_time+interval;
    float power_now_watts = pmd_tracker_.GetPmdPower()[0];
    std::this_thread::sleep_until(next_time_point);

    while (active_) {
        // gpu directly call power and get actual interval
        auto call_func_time = high_resolution_clock::now();
        next_time_point += interval;  // Move to the next interval
        power_now_watts = pmd_tracker_.GetPmdPower()[0];
        // float power_now_watts = (power_now_mw * static_cast<float>(1e-3));
        auto after_func_time = high_resolution_clock::now();
        std::chrono::microseconds actual_interval_us = 
            std::chrono::duration_cast<std::chrono::microseconds>(call_func_time-call_previous_func_time);
        // uint64_t actual_interval_count_us = actual_interval_us.count();
        call_previous_func_time = call_func_time;
        // check whether exec time exceeds interval
        std::chrono::microseconds func_exec_duration =
            std::chrono::duration_cast<std::chrono::microseconds>(after_func_time-call_func_time);
        uint64_t func_exec_duration_us_count = func_exec_duration.count();
        uint64_t interval_us_count = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(interval).count());
        if (func_exec_duration_us_count > interval_us_count) {
            std::cout << __PRETTY_FUNCTION__  << ": func_exec_duration_us_count exceed interval - func_exec_duration_us_count: "
            << func_exec_duration_us_count << " us , interval: "
            << interval_us_count << " us" << std::endl;
        }

        call_count++;

        // result format : 
        // {std::string dev_type; uint32_t call_cnt;float power_result;std::time_t measure_timestamp;std::chrono::duration<uint64_t,std::micro> current_interval;
        if (power_now_watts > 0){
            ru_result_ = {"oru_power",call_count,power_now_watts,call_func_time,actual_interval_us};
        }
        std::this_thread::sleep_until(next_time_point);
    }
}

void TrackerScheduler::PrintAllTrackerResults() const {
    std::cout << std::fixed << std::setprecision(6) 
                << "-----------------------------------------------------------  " 
                << "Current Timestamp: " << GetTimestampUs(std::chrono::high_resolution_clock::now())
                << "  -----------------------------------------------------------" << std::endl;
    std::cout << std::fixed << std::setprecision(6)
                << "Measured Type: " << cpu_result_.dev_type << " , "
                << "Func Timestamp: " << GetTimestampUs(cpu_result_.measure_timestamp) << " , "
                << "CPU Power: " << std::setw(12) << cpu_result_.power_result << " W , "
                << "Interval: " << std::setw(12) << cpu_result_.current_interval_us.count() << " us ,"
                << "Call: " << std::setw(12) << cpu_result_.call_cnt << " , " 
                << std::endl;
    std::cout << std::fixed << std::setprecision(6)
                << "Measured Type: " << gpu_result_.dev_type << " , "
                << "Func Timestamp: " << GetTimestampUs(gpu_result_.measure_timestamp) << " , "
                << "GPU Power: " << std::setw(12) << gpu_result_.power_result << " W , "
                << "Interval: " << std::setw(12) << gpu_result_.current_interval_us.count() << " us ,"
                << "Call: " << std::setw(12) << gpu_result_.call_cnt << " , " 
                << std::endl; 
    std::cout << std::fixed << std::setprecision(6)
                << "Measured Type: " << ru_result_.dev_type << " , "
                << "Func Timestamp: " << GetTimestampUs(ru_result_.measure_timestamp) << " , "
                << "RU Power: " << std::setw(12) << ru_result_.power_result << " W , "
                << "Interval: " << std::setw(12) << ru_result_.current_interval_us.count() << " us ,"
                << "Call: " << std::setw(12) << ru_result_.call_cnt << " , " 
                << std::endl; 
    std::cout << "------------------------------------------------------------------------------------"
                << "-----------------------------------------------------------------------------------" << std::endl;
}

void TrackerScheduler::PeriodicalPrintResults(std::chrono::duration<uint64_t,std::milli> interval) {
    using namespace std::chrono;
    auto next_time_point = high_resolution_clock::now();
    while (active_) {
        next_time_point += interval;  // Move to the next interval
        PrintAllTrackerResults();
        WriteAllTrackerResultsToShm();
        std::this_thread::sleep_until(next_time_point);
    }
}

std::vector<SchedulerMeasuredResult> TrackerScheduler::GetAllTrackerResults() const {
    std::vector<SchedulerMeasuredResult> vector_results = {cpu_result_, gpu_result_,ru_result_};
    return vector_results; 
}

// ------------- shared memory function implementation-----
void TrackerScheduler::WriteAllTrackerResultsToShm() {
    int written_out = WritePowerShm(cpu_result_.power_result, gpu_result_.power_result, ru_result_.power_result, 
                cpu_result_.call_cnt, gpu_result_.call_cnt, ru_result_.call_cnt);
    std::cout << "point TW done" << written_out << std::endl;
}

int TrackerScheduler::InitializePowerShm() {
    int shm_fd = shm_open(POWER_SHM_NAME, O_CREAT | O_RDWR, 0666);
    if (shm_fd == -1) {
        std::cerr << "Error opening shared memory POWER_SHM_NAME " << POWER_SHM_NAME << std::endl;
        return 1;
    }

    if (ftruncate(shm_fd, POWER_SHM_SIZE) == -1) {
        std::cerr << "Error setting shared memory size POWER_SHM_SIZE " << POWER_SHM_SIZE << std::endl;
        return 1;
    }

    auto *shm_data = (Shared_Memory_PowerMeasurement_t *)mmap(
        nullptr, POWER_SHM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (shm_data == MAP_FAILED) {
        std::cerr << "Error mapping shared memory POWER_SHM_NAME " << POWER_SHM_NAME << " POWER_SHM_SIZE " << POWER_SHM_SIZE << std::endl;
        return 1;
    }

    // Initialize mutex
    pthread_mutexattr_t attr;
    pthread_mutexattr_init(&attr);
    pthread_mutexattr_setpshared(&attr, PTHREAD_PROCESS_SHARED);
    pthread_mutex_init(&shm_data->mutex, &attr);
    pthread_mutexattr_destroy(&attr);
    // new (&shm_data->mutex) std::mutex();
    // Initialize shared memory values
    shm_data->timestamp_us = 0;
    shm_data->cpu_power = 0.0f;
    shm_data->gpu_power = 0.0f;
    shm_data->ru_power = 0.0f;
    shm_data->cpu_call_cnt = 0;
    shm_data->gpu_call_cnt = 0;
    shm_data->ru_call_cnt = 0;
    std::cout << "Shared memory initialized successfully." << std::endl;

    munmap(shm_data, POWER_SHM_SIZE);
    close(shm_fd);
    return 0;
}

int TrackerScheduler::WritePowerShm(float cpu_power, float gpu_power, float ru_power, uint32_t cpu_call, uint32_t gpu_call, uint32_t ru_call) {
    int shm_fd = shm_open(POWER_SHM_NAME, O_RDWR, 0666);
    if (shm_fd == -1) {
        std::cerr << "Error opening shared memory POWER_SHM_NAME " << POWER_SHM_NAME << std::endl;
        return 1;
    }

    auto *shm_data = (Shared_Memory_PowerMeasurement_t *)mmap(
        nullptr, POWER_SHM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (shm_data == MAP_FAILED) {
        std::cerr << "Error mapping shared memory POWER_SHM_SIZE " << POWER_SHM_SIZE << std::endl;
        return 1;
    }

    std::cout << "Shared memory writer started POWER_SHM_NAME " << POWER_SHM_NAME << std::endl;

    pthread_mutex_lock(&shm_data->mutex);
    // std::lock_guard<std::mutex> lock(shm_data->mutex);
    // Update power measurements --- watts
    shm_data->timestamp_us = static_cast<uint64_t>(GetTimestampUs());
    shm_data->cpu_power = static_cast<float>(cpu_power);
    shm_data->gpu_power = static_cast<float>(gpu_power);
    shm_data->ru_power = static_cast<float>(ru_power);
    shm_data->cpu_call_cnt = static_cast<uint32_t>(cpu_call);
    shm_data->gpu_call_cnt = static_cast<uint32_t>(gpu_call);
    shm_data->ru_call_cnt = static_cast<uint32_t>(ru_call);
    pthread_mutex_unlock(&shm_data->mutex);

    std::cout << POWER_SHM_NAME << " written at: " << GetFormattedTime() << " | "
                << "CPU: " << shm_data->cpu_power << "W, "
                << "GPU: " << shm_data->gpu_power << "W, "
                << "RU: " << shm_data->ru_power << "W, "
                << "CPU_call: " << shm_data->cpu_call_cnt << ", "
                << "GPU_call: " << shm_data->gpu_call_cnt << ", "
                << "RU_call: " << shm_data->ru_call_cnt << " " << std::endl;
    // Cleanup
    // std::cout << "point 1 done" << std::endl;
    // munmap(shm_data, POWER_SHM_SIZE);
    // std::cout << "point 2 done" << std::endl;
    // close(shm_fd);
    // std::cout << "point 3 done" << std::endl;
    if (munmap(shm_data, POWER_SHM_SIZE) == -1) {
        perror("munmap failed");
    }
    if (close(shm_fd) == -1) {
        perror("close failed");
    }
    return 0;
}

PowerMeasurementResult TrackerScheduler::ReadPowerShm(){
    int shm_fd = shm_open(POWER_SHM_NAME, O_RDWR, 0666);
    if (shm_fd == -1) {
        throw std::runtime_error(std::string("Error opening shared memory ")+POWER_SHM_NAME);
    }

    auto *shm_data = (Shared_Memory_PowerMeasurement_t *)mmap(
        nullptr, POWER_SHM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (shm_data == MAP_FAILED) {
        close(shm_fd);
        throw std::runtime_error(std::string("Error mapping shared memory ") + std::to_string(POWER_SHM_SIZE));
    }

    // Read and return shared memory data
    PowerMeasurementResult result;
    pthread_mutex_lock(&shm_data->mutex);
    {
        // std::lock_guard<std::mutex> lock(shm_data->mutex);
        result.timestamp_us = shm_data->timestamp_us;
        result.cpu_power = shm_data->cpu_power;
        result.gpu_power = shm_data->gpu_power;
        result.ru_power = shm_data->ru_power;
        result.cpu_call_cnt = shm_data->cpu_call_cnt;
        result.gpu_call_cnt = shm_data->gpu_call_cnt;
        result.ru_call_cnt = shm_data->ru_call_cnt;
    }
    pthread_mutex_unlock(&shm_data->mutex);

    munmap(shm_data, POWER_SHM_SIZE);
    close(shm_fd);
    return result;
}


void TrackerScheduler::UnlinkShm(){
    shm_unlink(POWER_SHM_NAME);
}

// ----------------- shm above ----
