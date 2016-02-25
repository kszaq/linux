
#ifndef __AMLOGIC_THERMAL_H__
#define __AMLOGIC_THERMAL_H__

struct record_buf {
    int idx;
    int max;
    unsigned long cool_flag;
    unsigned int *op;
};

struct cpu_stat_monitor {
    unsigned int total_cpu_freq;
    unsigned int total_gpu_freq;
    unsigned int total_cpu_cores;
    unsigned int total_gpu_cores;
    unsigned int avg_cpu_freq;
    unsigned int avg_gpu_freq;
    unsigned int avg_cpu_cores;
    unsigned int avg_gpu_cores;
    unsigned int filter_temp;
};

struct aml_virtual_thermal {
    unsigned int freq;
    unsigned int temp_time[4];
};

struct aml_virtual_thermal_device {
    int count;
    struct aml_virtual_thermal *thermal;
};

struct temp_trip{
    unsigned int temperature;
    unsigned int cpu_upper_freq;
    unsigned int cpu_lower_freq;
    int cpu_upper_level;
    int cpu_lower_level;
    unsigned int gpu_upper_freq;
    unsigned int gpu_lower_freq;
    int gpu_upper_level;
    int gpu_lower_level;
    int cpu_core_num;
    int cpu_core_upper;
    int gpu_core_num;
    int gpu_core_upper;
};

struct amlogic_thermal_platform_data {
    const char *name;
    struct temp_trip *tmp_trip;
    unsigned int temp_trip_count;
    unsigned int temp_valid;
    unsigned int current_temp;
    unsigned int idle_interval;
    unsigned int trim_flag;
    unsigned int virtual_thermal_en;
    unsigned int keep_mode;
    unsigned int keep_mode_threshold;
    unsigned int keep_mode_ini_state[4];
    unsigned int keep_mode_cur_state[4];
    unsigned int keep_mode_max_state[4];
    unsigned int keep_mode_min_state[4];
    unsigned int keep_mode_max_range[4];
    unsigned int keep_mode_min_range[4];
    unsigned int keep_min_exist;
    unsigned int freq_sample_period;
    struct record_buf op_buf;
    struct cpu_stat_monitor monitor;
    struct thermal_zone_device *therm_dev;
    struct thermal_cooling_device *cpu_cool_dev;
    struct thermal_cooling_device *gpu_cool_dev;
    struct thermal_cooling_device *cpucore_cool_dev;
    struct thermal_cooling_device *gpucore_cool_dev;
    enum thermal_device_mode mode;
    struct mutex lock;
    struct delayed_work thermal_work;
};

struct temp_level{
    unsigned int temperature;
    int cpu_high_freq;
    int cpu_low_freq;
    int gpu_high_freq;
    int gpu_low_freq;
    int cpu_core_num;
    int gpu_core_num;
};

struct freq_trip_table {
    unsigned int freq_state;
};

void *thermal_alloc(size_t len);
extern int thermal_debug_enable;
extern int high_temp_protect;
extern atomic_t freq_update_flag;

void thermal_atomic_set(atomic_t *a, int);
void thermal_lock(struct mutex *lock);
void thermal_unlock(struct mutex *lock);
void keep_mode_set_mode(struct amlogic_thermal_platform_data *);
void keep_mode_bind(struct  amlogic_thermal_platform_data *, unsigned long , int );
void keep_mode_work(struct amlogic_thermal_platform_data *, int);
void keep_mode_update_threshold(struct amlogic_thermal_platform_data *, int );
void keep_mode_temp_level_init(struct amlogic_thermal_platform_data *, struct temp_level *);
void *aml_get_cdevdata(struct thermal_cooling_device *cdev);
void aml_set_cdev_update(struct thermal_cooling_device *cdev, bool update);
void aml_cdev_lockop(struct thermal_cooling_device *cdev, bool lock);
void aml_cdev_get_cur_state(struct thermal_cooling_device *cdev, unsigned long *ret);
int gpu_get_freq_level(int freq);

#endif /* __AMLOGIC_THERMAL_H__ */
