
#ifndef TRACKERSCHEDULER_H
#define TRACKERSCHEDULER_H

#include <stdio.h>
#include <chrono>
#include <thread>
#include <mutex>
#include <atomic>
#include <memory>
#include <string>
#include <iostream>
#include <fstream>
#include <chrono>
#include <thread>
#include <iomanip>
#include <ctime>
#include <sstream>

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <pthread.h>

#include <Tracker.h>
#include <CpuTracker.h>
#include <GpuTracker.h>
#include <PmdTracker.h>

// Define shared memory
#define POWER_SHM_NAME "/power_measurement_shm"
#define POWER_SHM_SIZE sizeof(Shared_Memory_PowerMeasurement_t)

struct Shared_Memory_PowerMeasurement_t {
    pthread_mutex_t mutex;          // Mutex - assume 48 bytes (GH), 40 bytes (Intel)
    uint64_t timestamp_us;     // 8 bytes Timestamp in microseconds
    float cpu_power;           // 4 bytes
    float gpu_power;           // 4 bytes
    float ru_power;            // 4 bytes
    uint32_t cpu_call_cnt;     // 4 bytes
    uint32_t gpu_call_cnt;     // 4 bytes
    uint32_t ru_call_cnt;      // 4 bytes
};                                          // In total 80 bytes (GH), 72 bytes (Intel)

struct PowerMeasurementResult {
    uint64_t timestamp_us;
    float cpu_power;
    float gpu_power;
    float ru_power;
    uint32_t cpu_call_cnt;
    uint32_t gpu_call_cnt;
    uint32_t ru_call_cnt;
};

struct SchedulerMeasuredResult {
    std::string dev_type;
    uint32_t call_cnt;
    float power_result;
    std::chrono::time_point<std::chrono::high_resolution_clock> measure_timestamp;
    std::chrono::duration<uint64_t,std::micro> current_interval_us;
};

class TrackerScheduler {
public:
    TrackerScheduler();
    virtual ~TrackerScheduler();

    void PrintMeasuredPower(std::chrono::duration<uint64_t,std::milli> interval);
    
    void MeasureCpuPower(std::chrono::duration<uint64_t,std::milli> interval);
    void MeasureGpuPower(std::chrono::duration<uint64_t,std::milli> interval);
    void MeasureRuPower(std::chrono::duration<uint64_t,std::milli> interval);

    std::vector<SchedulerMeasuredResult> GetAllTrackerResults() const;
    void PrintAllTrackerResults() const;
    void PeriodicalPrintResults(std::chrono::duration<uint64_t,std::milli> interval);

    std::string GetTimestampUs(std::chrono::time_point<std::chrono::high_resolution_clock> time_now) const;

    void Disable() {active_ = false;}
    void Enable() {active_ = true;}

private:
    // std::atomic<bool> active_;
    bool active_;

    CpuTracker cpu_tracker_;
    // std::thread cpu_thread_;
    SchedulerMeasuredResult cpu_result_;

    GpuTracker gpu_tracker_;
    // std::thread gpu_thread_;
    SchedulerMeasuredResult gpu_result_;

    PmdTracker pmd_tracker_;
    // std::thread ru_thread_;
    SchedulerMeasuredResult ru_result_;

    // --- EHTOS - shared memory related functions and members ---
    Shared_Memory_PowerMeasurement_t power_shm_;
    PowerMeasurementResult power_read_result_;

    void WriteAllTrackerResultsToShm();
    int InitializePowerShm();
    int WritePowerShm(float cpu_power, float gpu_power, float ru_power, uint32_t cpu_call, uint32_t gpu_call, uint32_t ru_call);
    PowerMeasurementResult ReadPowerShm();
    void UnlinkShm();
    
    // Get the current timestamp in microseconds
    inline uint64_t GetTimestampUs() {
        return std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::high_resolution_clock::now().time_since_epoch())
            .count();
    }

    // Get the current timestamp in formated string
    inline std::string GetFormattedTime() {
        auto now = std::chrono::system_clock::now();
        std::time_t now_time = std::chrono::system_clock::to_time_t(now);
        std::tm *timeinfo = std::localtime(&now_time);
        auto duration_since_epoch = now.time_since_epoch();
        auto micros = std::chrono::duration_cast<std::chrono::microseconds>(duration_since_epoch) % 1000000;
        std::ostringstream oss;
        oss << std::put_time(timeinfo, "%Y-%m-%d %H:%M:%S")<< "." << std::setw(6) << std::setfill('0') << micros.count();
        return oss.str();
    }
    // --------------- shared memory above------------------//
};


#endif /* TRACKERSCHEDULER_H */