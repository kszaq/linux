/*
 * amlogic_thermal.c - Samsung amlogic thermal (Thermal Management Unit)
 *
 *  Copyright (C) 2011 Samsung Electronics
 *  Donggeun Kim <dg77.kim@samsung.com>
 *  Amit Daniel Kachhap <amit.kachhap@linaro.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <linux/module.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/workqueue.h>
#include <linux/sysfs.h>
#include <linux/kobject.h>
#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/thermal.h>
#include <linux/cpufreq.h>
#include <linux/cpu_cooling.h>
#include <linux/of.h>
#include <linux/amlogic/saradc.h>
#include <linux/random.h>
#include <linux/gpu_cooling.h>
#include <linux/cpucore_cooling.h>
#include <linux/gpucore_cooling.h>
#include <linux/thermal_core.h>
#include <linux/version.h>
#if LINUX_VERSION_CODE > KERNEL_VERSION(3, 10, 33)
#include <linux/amlogic/aml_thermal_hw.h>
#else
#include <mach/thermal.h>
#endif
#include <linux/version.h>
#include "amlogic_thermal.h"

#define DBG_VIRTUAL        0
#define MIN_TEMP           (-273)
int thermal_debug_enable = 0;
int high_temp_protect    = 0;
atomic_t freq_update_flag;
EXPORT_SYMBOL(thermal_debug_enable);
EXPORT_SYMBOL(high_temp_protect);
EXPORT_SYMBOL(freq_update_flag);

#define THERMAL_DBG(format,args...) \
    if (thermal_debug_enable) { \
        printk("[THERMAL]"format, ##args);     \
    }

static struct device *dbg_dev;

#define THERMAL_ERR(format, args...)            \
    {if (dbg_dev)                    \
        dev_err(dbg_dev, format, ##args);    \
    }

#define THERMAL_INFO(format, args...)            \
    {if (dbg_dev)                    \
        dev_info(dbg_dev, format, ##args);    \
    }

static struct aml_virtual_thermal_device cpu_virtual_thermal = {};
static struct aml_virtual_thermal_device gpu_virtual_thermal = {};
static unsigned int report_interval[4] = {};
static int (*gpu_freq_level)(int ) = NULL;

/* CPU Zone information */
#define PANIC_ZONE      4
#define WARN_ZONE       3
#define MONITOR_ZONE    2
#define SAFE_ZONE       1

#define GET_ZONE(trip) (trip + 2)
#define GET_TRIP(zone) (zone - 2)

static void amlogic_unregister_thermal(struct amlogic_thermal_platform_data *pdata);
static int amlogic_register_thermal(struct amlogic_thermal_platform_data *pdata, struct platform_device *pdev);

void thermal_lock(struct mutex *lock)
{
    mutex_lock(lock);
}
EXPORT_SYMBOL(thermal_lock);

void thermal_unlock(struct mutex *lock)
{
    mutex_unlock(lock);
}
EXPORT_SYMBOL(thermal_unlock);

/* Get mode callback functions for thermal zone */
static int amlogic_get_mode(struct thermal_zone_device *thermal,
        enum thermal_device_mode *mode)
{
    struct  amlogic_thermal_platform_data *pdata= thermal->devdata;

    if (pdata)
        *mode = pdata->mode;
    return 0;
}

/* Set mode callback functions for thermal zone */
static int amlogic_set_mode(struct thermal_zone_device *thermal,
        enum thermal_device_mode mode)
{
    struct  amlogic_thermal_platform_data *pdata= thermal->devdata;
    struct cpucore_cooling_device *cpucore_device =NULL;
    struct gpucore_cooling_device *gpucore_device = NULL;
    if(!pdata)
        return -EINVAL;

    //mutex_lock(&pdata->therm_dev->lock);

    if (mode == THERMAL_DEVICE_ENABLED){
        pdata->therm_dev->polling_delay = pdata->idle_interval;
        if(pdata->cpucore_cool_dev){
            cpucore_device=pdata->cpucore_cool_dev->devdata;
            cpucore_device->stop_flag=0;
        }
        if(pdata->gpucore_cool_dev){
            gpucore_device=pdata->gpucore_cool_dev->devdata;
            gpucore_device->stop_flag=0;
        }
        if (pdata->keep_mode) {                     // start work
            schedule_delayed_work(&pdata->thermal_work, msecs_to_jiffies(100));
        }
    }
    else{
        pdata->therm_dev->polling_delay = 0;
        if (pdata->keep_mode) {
            cancel_delayed_work_sync(&pdata->thermal_work);
            keep_mode_set_mode(pdata);
        }
        if(pdata->cpucore_cool_dev)
            pdata->cpucore_cool_dev->ops->set_cur_state(pdata->cpucore_cool_dev,(0|CPU_STOP));
        if(pdata->gpucore_cool_dev)
            pdata->gpucore_cool_dev->ops->set_cur_state(pdata->gpucore_cool_dev,(0|GPU_STOP));
    }

    //mutex_unlock(&pdata->therm_dev->lock);

    pdata->mode = mode;
    thermal_zone_device_update(pdata->therm_dev);
    THERMAL_INFO("thermal polling set for duration=%d msec\n",
            pdata->therm_dev->polling_delay);
    return 0;
}

/* Get trip type callback functions for thermal zone */
static int amlogic_get_trip_type(struct thermal_zone_device *thermal, int trip,
        enum thermal_trip_type *type)
{
    if(trip < thermal->trips-1)
        *type = THERMAL_TRIP_ACTIVE;
    else if(trip == thermal->trips-1)
        *type = THERMAL_TRIP_CRITICAL;
    else
        return -EINVAL;
    return 0;
}

/* Get trip temperature callback functions for thermal zone */
static int amlogic_get_trip_temp(struct thermal_zone_device *thermal, int trip,
        unsigned long *temp)
{
    struct  amlogic_thermal_platform_data *pdata= thermal->devdata;

    if(trip > pdata->temp_trip_count ||trip<0)
        return  -EINVAL;
    mutex_lock(&pdata->lock);
    *temp =pdata->tmp_trip[trip].temperature;
    /* convert the temperature into millicelsius */
    mutex_unlock(&pdata->lock);

    return 0;
}

static int amlogic_set_trip_temp(struct thermal_zone_device *thermal, int trip,
        unsigned long temp)
{
    struct  amlogic_thermal_platform_data *pdata= thermal->devdata;

    if(trip > pdata->temp_trip_count ||trip<0)
        return  -EINVAL;
    mutex_lock(&pdata->lock);
    pdata->tmp_trip[trip].temperature=temp;
    /* convert the temperature into millicelsius */
    mutex_unlock(&pdata->lock);
    return 0;
}

/* Get critical temperature callback functions for thermal zone */
static int amlogic_get_crit_temp(struct thermal_zone_device *thermal,
        unsigned long *temp)
{
    int ret;
    /* Panic zone */
    ret =amlogic_get_trip_temp(thermal, thermal->trips-1, temp);

    return ret;
}

int gpu_get_freq_level(int freq)
{
    if (gpu_freq_level)
        return gpu_freq_level(freq);
    else
        return -1;
}

/* Bind callback functions for thermal zone */
static int amlogic_bind(struct thermal_zone_device *thermal,
        struct thermal_cooling_device *cdev)
{
    int ret = 0, i;
    struct  amlogic_thermal_platform_data *pdata= thermal->devdata;
    int id;
    char type[THERMAL_NAME_LENGTH];
    unsigned long max;

    if (!sscanf(cdev->type, "thermal-%7s-%d", type,&id))
        return -EINVAL;
    if(!strcmp(type,"cpufreq")){
        /* Bind the thermal zone to the cpufreq cooling device */
        for (i = 0; i < pdata->temp_trip_count; i++) {
            if(pdata->tmp_trip[0].cpu_upper_level==THERMAL_CSTATE_INVALID)
            {
                ret = -EINVAL;
                goto out;
            }
            if (thermal_zone_bind_cooling_device(thermal, i, cdev,
                        pdata->tmp_trip[i].cpu_upper_level,
                        pdata->tmp_trip[i].cpu_lower_level)) {
                THERMAL_ERR("error binding cdev inst %d\n", i);
                ret = -EINVAL;
                goto out;
            }
        }
        if (pdata->keep_mode) {
            cdev->ops->get_max_state(cdev, &max);
            keep_mode_bind(pdata, max, 0);
        }
    }

    if(!strcmp(type,"gpufreq")){
        struct gpufreq_cooling_device *gpufreq_dev=
            (struct gpufreq_cooling_device *)cdev->devdata;
        /* Bind the thermal zone to the cpufreq cooling device */
        for (i = 0; i < pdata->temp_trip_count; i++) {
            if(!gpufreq_dev->get_gpu_freq_level){
                ret = -EINVAL;
                THERMAL_ERR("invalidate pointer %p\n",gpufreq_dev->get_gpu_freq_level);
                goto out;
            } else {
                gpu_freq_level = gpufreq_dev->get_gpu_freq_level;
            }
            pdata->tmp_trip[i].gpu_lower_level=gpufreq_dev->get_gpu_freq_level(pdata->tmp_trip[i].gpu_upper_freq);
            pdata->tmp_trip[i].gpu_upper_level=gpufreq_dev->get_gpu_freq_level(pdata->tmp_trip[i].gpu_lower_freq);
            if(pdata->tmp_trip[0].gpu_lower_level==THERMAL_CSTATE_INVALID)
            {
                ret = -EINVAL;
                goto out;
            }
            if (thermal_zone_bind_cooling_device(thermal, i, cdev,
                        pdata->tmp_trip[i].gpu_upper_level,
                        pdata->tmp_trip[i].gpu_lower_level)) {
                THERMAL_ERR("error binding cdev inst %d\n", i);
                ret = -EINVAL;
                goto out;
            }
        }
        pdata->gpu_cool_dev=cdev;
        if (pdata->keep_mode) {
            cdev->ops->get_max_state(cdev, &max);
            keep_mode_bind(pdata, max, 1);
        }
    }

    if(!strcmp(type,"cpucore")){
        /* Bind the thermal zone to the cpufreq cooling device */
        struct cpucore_cooling_device *cpucore_dev=
            (struct cpucore_cooling_device *)cdev->devdata;
        for (i = 0; i < pdata->temp_trip_count; i++) {
            if(pdata->tmp_trip[0].cpu_core_num==THERMAL_CSTATE_INVALID)
            {
                ret = -EINVAL;
                goto out;
            }
            if(pdata->tmp_trip[i].cpu_core_num !=-1)
                pdata->tmp_trip[i].cpu_core_upper=cpucore_dev->max_cpu_core_num-pdata->tmp_trip[i].cpu_core_num;
            else
                pdata->tmp_trip[i].cpu_core_upper=pdata->tmp_trip[i].cpu_core_num;
            if (thermal_zone_bind_cooling_device(thermal, i, cdev,
                        pdata->tmp_trip[i].cpu_core_upper,
                        pdata->tmp_trip[i].cpu_core_upper)) {
                THERMAL_ERR("error binding cdev inst %d\n", i);
                ret = -EINVAL;
                goto out;
            }
        }
        if (pdata->keep_mode) {
            cdev->ops->get_max_state(cdev, &max);
            keep_mode_bind(pdata, max, 2);
        }
    }

    if(!strcmp(type,"gpucore")){
        /* Bind the thermal zone to the cpufreq cooling device */
        struct gpucore_cooling_device *gpucore_dev=
            (struct gpucore_cooling_device *)cdev->devdata;
        for (i = 0; i < pdata->temp_trip_count; i++) {
            if(pdata->tmp_trip[0].cpu_core_num==THERMAL_CSTATE_INVALID)
            {
                ret = -EINVAL;
                goto out;
            }
            if(pdata->tmp_trip[i].gpu_core_num != -1)
                pdata->tmp_trip[i].gpu_core_upper=gpucore_dev->max_gpu_core_num-pdata->tmp_trip[i].gpu_core_num;
            else
                pdata->tmp_trip[i].gpu_core_upper=pdata->tmp_trip[i].gpu_core_num;

            if (thermal_zone_bind_cooling_device(thermal, i, cdev,
                        pdata->tmp_trip[i].gpu_core_upper,
                        pdata->tmp_trip[i].gpu_core_upper)) {
                THERMAL_ERR("error binding cdev inst %d\n", i);
                ret = -EINVAL;
                goto out;
            }
        }
        pdata->gpucore_cool_dev=cdev;
        if (pdata->keep_mode) {
            cdev->ops->get_max_state(cdev, &max);
            keep_mode_bind(pdata, max, 3);
        }
    }
    return ret;
out:
    return ret;
}

/* Unbind callback functions for thermal zone */
static int amlogic_unbind(struct thermal_zone_device *thermal,
        struct thermal_cooling_device *cdev)
{
    int i;
    if(thermal && cdev){
        struct  amlogic_thermal_platform_data *pdata= thermal->devdata;
        for (i = 0; i < pdata->temp_trip_count; i++) {
            if (thermal_zone_unbind_cooling_device(thermal, i, cdev)) {
                THERMAL_ERR(" error  %d \n", i);
                return -EINVAL;
            }
            return 0;
        }
    }else{
        return -EINVAL;
    }
    return -EINVAL;
}
#define ABS(a) ((a) > 0 ? (a) : -(a))

void *thermal_alloc(size_t len)
{
    return kzalloc(len, GFP_KERNEL);
}
EXPORT_SYMBOL(thermal_alloc);

static void thermal_work(struct work_struct *work)
{
    struct amlogic_thermal_platform_data *pdata;
    int cpu_freq = cpufreq_quick_get(0);

    pdata = container_of((struct delayed_work *)work, struct amlogic_thermal_platform_data, thermal_work);
    if (pdata->temp_valid)
        keep_mode_work(pdata, cpu_freq);
    if (pdata->mode == THERMAL_DEVICE_ENABLED) {             // no need to do this work again if thermal disabled
        schedule_delayed_work(&pdata->thermal_work, msecs_to_jiffies(100));
    }
}

static int aml_virtaul_thermal_probe(struct platform_device *pdev, struct amlogic_thermal_platform_data *pdata)
{
    int ret, len, cells;
    struct property *prop;
    void *buf;

    if (!of_property_read_bool(pdev->dev.of_node, "use_virtual_thermal")) {
        pdata->virtual_thermal_en = 0;
        return 0;
    }

    ret = of_property_read_u32(pdev->dev.of_node,
            "freq_sample_period",
            &pdata->freq_sample_period);
    if (ret) {
        pdata->freq_sample_period = 30;
    }
    ret = of_property_read_u32_array(pdev->dev.of_node,
            "report_time",
            report_interval, sizeof(report_interval) / sizeof(u32));
    if (ret) {
        goto error;
    }
    /*
     * read cpu_virtal
     */
    prop = of_find_property(pdev->dev.of_node, "cpu_virtual", &len);
    if (!prop) {
        goto error;
    }
    cells = len / sizeof(struct aml_virtual_thermal);
    buf = kzalloc(len, GFP_KERNEL);
    if (!buf) {
        THERMAL_ERR("%s, no memory\n", __func__);
        return -ENOMEM;
    }
    ret = of_property_read_u32_array(pdev->dev.of_node,
            "cpu_virtual",
            buf, len/sizeof(u32));
    if (ret) {
        kfree(buf);
        goto error;
    }
    cpu_virtual_thermal.count   = cells;
    cpu_virtual_thermal.thermal = buf;

    /*
     * read gpu_virtal
     */
    prop = of_find_property(pdev->dev.of_node, "gpu_virtual", &len);
    if (!prop) {
        goto error;
    }
    cells = len / sizeof(struct aml_virtual_thermal);
    buf = kzalloc(len, GFP_KERNEL);
    if (!buf) {
        return -ENOMEM;
    }
    ret = of_property_read_u32_array(pdev->dev.of_node,
            "gpu_virtual",
            buf, len/sizeof(u32));
    if (ret) {
        kfree(buf);
        goto error;
    }
    gpu_virtual_thermal.count   = cells;
    gpu_virtual_thermal.thermal = buf;

    pdata->virtual_thermal_en = 1;
    return 0;

error:
    pdata->virtual_thermal_en = 0;
    return -1;
}

static void aml_virtual_thermal_remove(struct amlogic_thermal_platform_data *pdata)
{
    kfree(cpu_virtual_thermal.thermal);
    kfree(gpu_virtual_thermal.thermal);
    pdata->virtual_thermal_en = 0;
}

static int check_freq_level(struct aml_virtual_thermal_device *dev, unsigned int freq)
{
    int i = 0;

    if (freq >= dev->thermal[dev->count-1].freq) {
        return dev->count - 1;
    }
    for (i = 0; i < dev->count - 1; i++) {
        if (freq > dev->thermal[i].freq && freq <= dev->thermal[i + 1].freq) {
            return i + 1;
        }
    }
    return 0;
}

static int check_freq_level_cnt(unsigned int cnt)
{
    int i;

    if (cnt >= report_interval[3]) {
        return  3;
    }
    for (i = 0; i < 3; i++) {
        if (cnt >= report_interval[i] && cnt < report_interval[i + 1]) {
            return i;
        }
    }
    return 0;
}

static unsigned long aml_cal_virtual_temp(struct amlogic_thermal_platform_data *pdata)
{
    static unsigned int cpu_freq_level_cnt  = 0, gpu_freq_level_cnt  = 0;
    static unsigned int last_cpu_freq_level = 0, last_gpu_freq_level = 0;
    static unsigned int cpu_temp = 40, gpu_temp = 40;                   // default set to 40 when at homescreen
    unsigned int curr_cpu_avg_freq,   curr_gpu_avg_freq;
    int curr_cpu_freq_level, curr_gpu_freq_level;
    int cnt_level, level_diff;
    int temp_update = 0, final_temp;

    /*
     * CPU temp
     */
    if (atomic_read(&freq_update_flag)) {
        curr_cpu_avg_freq = pdata->monitor.avg_cpu_freq;
        curr_cpu_freq_level = check_freq_level(&cpu_virtual_thermal, curr_cpu_avg_freq);
        level_diff = curr_cpu_freq_level - last_cpu_freq_level;
        if (ABS(level_diff) <= 1) {  // freq change is not large
            cpu_freq_level_cnt++;
            cnt_level = check_freq_level_cnt(cpu_freq_level_cnt);
            cpu_temp  = cpu_virtual_thermal.thermal[curr_cpu_freq_level].temp_time[cnt_level];
        } else {                                                // level not match
            cpu_temp = cpu_virtual_thermal.thermal[curr_cpu_freq_level].temp_time[0];
            cpu_freq_level_cnt = 0;
        }
        last_cpu_freq_level = curr_cpu_freq_level;

        curr_gpu_avg_freq = pdata->monitor.avg_gpu_freq;
        curr_gpu_freq_level = check_freq_level(&gpu_virtual_thermal, curr_gpu_avg_freq);
        level_diff = curr_gpu_freq_level - last_gpu_freq_level;
        if (ABS(level_diff) <= 1) {  // freq change is not large
            gpu_freq_level_cnt++;
            cnt_level = check_freq_level_cnt(gpu_freq_level_cnt);
            gpu_temp  = gpu_virtual_thermal.thermal[curr_gpu_freq_level].temp_time[cnt_level];
        } else {                                                // level not match
            gpu_temp = gpu_virtual_thermal.thermal[curr_gpu_freq_level].temp_time[0];
            gpu_freq_level_cnt = 0;
        }
        last_gpu_freq_level = curr_gpu_freq_level;

        atomic_set(&freq_update_flag, 0);
        temp_update = 1;
    }

    if (cpu_temp <= 0 && gpu_temp <= 0) {
        final_temp = 40;
    }
    final_temp = (cpu_temp >= gpu_temp ? cpu_temp : gpu_temp);
    return final_temp;
}

/* Get temperature callback functions for thermal zone */
static int amlogic_get_temp(struct thermal_zone_device *thermal,
        unsigned long *temp)
{
    struct amlogic_thermal_platform_data *pdata = thermal->devdata;
    int tmp;

    if (pdata->trim_flag) {
        tmp = get_cpu_temp();
        if (tmp < MIN_TEMP) {
            pdata->temp_valid = 0;
            return -EINVAL;
        }
        pdata->temp_valid = 1;
        *temp = (unsigned long)get_cpu_temp();
        pdata->current_temp = *temp;
    } else if (pdata->virtual_thermal_en) {
        *temp = aml_cal_virtual_temp(pdata);
    } else {
        *temp = 45;                     // fix cpu temperature to 45 if not trimed && disable virtual thermal
    }
    return 0;
}

/* Get the temperature trend */
static int amlogic_get_trend(struct thermal_zone_device *thermal,
        int trip, enum thermal_trend *trend)
{
    return 1;
}
/* Operation callback functions for thermal zone */
static struct thermal_zone_device_ops amlogic_dev_ops = {
    .bind = amlogic_bind,
    .unbind = amlogic_unbind,
    .get_temp = amlogic_get_temp,
    .get_trend = amlogic_get_trend,
    .get_mode = amlogic_get_mode,
    .set_mode = amlogic_set_mode,
    .get_trip_type = amlogic_get_trip_type,
    .get_trip_temp = amlogic_get_trip_temp,
    .set_trip_temp = amlogic_set_trip_temp,
    .get_crit_temp = amlogic_get_crit_temp,
};

/*
 * sysfs for keep_mode
 */
#ifdef CONFIG_CPU_FREQ_GOV_HOTPLUG              // for DEBUG
extern unsigned int max_cpu_num;
static ssize_t max_cpu_num_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", max_cpu_num);
}
#endif

static ssize_t thermal_debug_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", thermal_debug_enable);
}

static ssize_t thermal_debug_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int32_t data = simple_strtol(buf, NULL, 10);

    if (data) {
        thermal_debug_enable = 1;
    } else {
        thermal_debug_enable = 0;
    }
    return count;
}

static ssize_t keep_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct thermal_zone_device *tz = container_of(dev, struct thermal_zone_device, device);
    struct amlogic_thermal_platform_data *pdata = tz->devdata;

    return sprintf(buf, "%s\n", pdata->keep_mode ? "enabled": "disabled");
}

static ssize_t keep_mode_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct thermal_zone_device *tz = container_of(dev, struct thermal_zone_device, device);
    struct amlogic_thermal_platform_data *pdata = tz->devdata;
    if (!strncmp(buf, "enabled", sizeof("enabled") - 1)) {
        pdata->keep_mode = 1;
    } else if (!strncmp(buf, "disabled", sizeof("disabled") - 1)) {
        pdata->keep_mode = 0;
    }
    return count;
}

static ssize_t keep_mode_threshold_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct thermal_zone_device *tz = container_of(dev, struct thermal_zone_device, device);
    struct amlogic_thermal_platform_data *pdata = tz->devdata;

    return sprintf(buf, "%d\n", pdata->keep_mode_threshold);
}

static ssize_t keep_mode_threshold_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct thermal_zone_device *tz = container_of(dev, struct thermal_zone_device, device);
    struct amlogic_thermal_platform_data *pdata = tz->devdata;
    int32_t data = simple_strtol(buf, NULL, 10);

    if (data > 200) {
        THERMAL_INFO("input is %d, seems too large, invalid\n", data);
    }
    keep_mode_update_threshold(pdata, data);
    THERMAL_INFO("set keep_mode_threshold to %d\n", data);
    return count;
}

static ssize_t high_temp_protect_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", high_temp_protect);
}

static ssize_t high_temp_protect_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct thermal_zone_device *tz = container_of(dev, struct thermal_zone_device, device);
    struct amlogic_thermal_platform_data *pdata = tz->devdata;
    int32_t data = simple_strtol(buf, NULL, 10);

    high_temp_protect = data ? 1 : 0;
    if (high_temp_protect) {
        pdata->tmp_trip[1].temperature = pdata->keep_mode_threshold + 25;
    } else {
        pdata->tmp_trip[1].temperature = 260;
    }
    THERMAL_INFO("high temperature protect %s\n", high_temp_protect ? "enabled" : "disabled");
    return count;
}

static struct device_attribute amlogic_thermal_attr[] = {
#ifdef CONFIG_CPU_FREQ_GOV_HOTPLUG
    __ATTR(max_cpu_num, 0444, max_cpu_num_show, NULL),
#endif
    __ATTR(thermal_debug, 0644, thermal_debug_show, thermal_debug_store),
    __ATTR(keep_mode, 0644, keep_mode_show, keep_mode_store),
    __ATTR(keep_mode_threshold, 0644, keep_mode_threshold_show, keep_mode_threshold_store),
    __ATTR(high_temp_protect, 0644, high_temp_protect_show, high_temp_protect_store)
};

/* Register with the in-kernel thermal management */
static int amlogic_register_thermal(struct amlogic_thermal_platform_data *pdata, struct platform_device *pdev)
{
    int ret=0, j;
    struct cpumask mask_val;

    memset(&mask_val,0,sizeof(struct cpumask));
    cpumask_set_cpu(0, &mask_val);
    pdata->cpu_cool_dev= cpufreq_cooling_register(&mask_val);
    if (IS_ERR(pdata->cpu_cool_dev)) {
        THERMAL_ERR("Failed to register cpufreq cooling device\n");
        ret = -EINVAL;
        goto err_unregister;
    }
    pdata->cpucore_cool_dev = cpucore_cooling_register();
    if (IS_ERR(pdata->cpucore_cool_dev)) {
        THERMAL_ERR("Failed to register cpufreq cooling device\n");
        ret = -EINVAL;
        goto err_unregister;
    }

    pdata->therm_dev = thermal_zone_device_register(pdata->name,
                                                    pdata->temp_trip_count,
                                                    ((1 << pdata->temp_trip_count) - 1),
                                                    pdata,
                                                    &amlogic_dev_ops,
                                                    NULL,
                                                    0,
                                                    pdata->idle_interval);

    if (IS_ERR(pdata->therm_dev)) {
        THERMAL_ERR("Failed to register thermal zone device, err:%p\n", pdata->therm_dev);
        ret = -EINVAL;
        goto err_unregister;
    }

    if (pdata->keep_mode) {                                     // create sysfs for keep_mode
        for (j = 0; j < ARRAY_SIZE(amlogic_thermal_attr); j++) {
            device_create_file(&pdata->therm_dev->device, &amlogic_thermal_attr[j]);
        }
    }

    return 0;

err_unregister:
    amlogic_unregister_thermal(pdata);
    return ret;
}

/* Un-Register with the in-kernel thermal management */
static void amlogic_unregister_thermal(struct amlogic_thermal_platform_data *pdata)
{
    if (pdata->therm_dev)
        thermal_zone_device_unregister(pdata->therm_dev);
    if (pdata->cpu_cool_dev)
        cpufreq_cooling_unregister(pdata->cpu_cool_dev);

}

int get_desend(void)
{
    int i;
    unsigned int freq = CPUFREQ_ENTRY_INVALID;
    int descend = -1;
    struct cpufreq_frequency_table *table =
        cpufreq_frequency_get_table(0);

    if (!table)
        return -EINVAL;

    for (i = 0; table[i].frequency != CPUFREQ_TABLE_END; i++) {
        /* ignore invalid entries */
        if (table[i].frequency == CPUFREQ_ENTRY_INVALID)
            continue;

        /* ignore duplicate entry */
        if (freq == table[i].frequency)
            continue;

        /* get the frequency order */
        if (freq != CPUFREQ_ENTRY_INVALID && descend == -1){
            descend = !!(freq > table[i].frequency);
            break;
        }

        freq = table[i].frequency;
    }
    return descend;
}
int fix_to_freq(int freqold,int descend)
{
    int i;
    unsigned int freq = CPUFREQ_ENTRY_INVALID;
    struct cpufreq_frequency_table *table =
        cpufreq_frequency_get_table(0);

    if (!table)
        return -EINVAL;

    for (i = 0; table[i].frequency != CPUFREQ_TABLE_END; i++) {
        /* ignore invalid entry */
        if (table[i].frequency == CPUFREQ_ENTRY_INVALID)
            continue;

        /* ignore duplicate entry */
        if (freq == table[i].frequency)
            continue;
        freq = table[i].frequency;
        if(descend){
            if(freqold>=table[i+1].frequency && freqold<=table[i].frequency)
                return table[i+1].frequency;
        }
        else{
            if(freqold>=table[i].frequency && freqold<=table[i+1].frequency)
                return table[i].frequency;
        }
    }
    return -EINVAL;
}

void thermal_atomic_set(atomic_t *a, int value)
{
    atomic_set(a, 1);
}
EXPORT_SYMBOL(thermal_atomic_set);

static struct amlogic_thermal_platform_data * amlogic_thermal_init_from_dts(struct platform_device *pdev, int trim_flag)
{
    int    i = 0, ret = -1, val = 0, cells, descend, error = 0;
    struct property   *prop;
    struct temp_level *tmp_level = NULL;
    struct amlogic_thermal_platform_data *pdata = NULL;

    if(!of_property_read_u32(pdev->dev.of_node, "trip_point", &val)){
        //INIT FROM DTS
        pdata=kzalloc(sizeof(*pdata),GFP_KERNEL);
        if(!pdata){
            goto err;
        }
        memset((void* )pdata,0,sizeof(*pdata));
        ret=of_property_read_u32(pdev->dev.of_node, "#thermal-cells", &val);
        if(ret){
            dev_err(&pdev->dev, "dt probe #thermal-cells failed: %d\n", ret);
            goto err;
        }
        cells=val;

        /*
         * process for KEEP_MODE and virtual thermal
         * Logic: If virtual thermal is enabled, then ignore keep_mode
         *
         */
        pdata->trim_flag = trim_flag;
        if (!pdata->trim_flag) {                                // chip is not trimmed, use virtual thermal
            aml_virtaul_thermal_probe(pdev, pdata);
        } else if (of_property_read_bool(pdev->dev.of_node, "keep_mode")) {
            if (of_property_read_u32(pdev->dev.of_node, "keep_mode_threshold", &pdata->keep_mode_threshold)) {
                error = 1;
            }
            if (of_property_read_u32_array(pdev->dev.of_node,
                        "keep_mode_max_range",
                        pdata->keep_mode_max_range,
                        sizeof(pdata->keep_mode_max_range)/sizeof(u32))) {
                error = 1;
            }
            if (!error && pdata->trim_flag) {                  // keep mode should not used for virtual thermal right now
                THERMAL_INFO("keep_mode_max_range:   [%7d, %3d, %d, %d]\n",
                        pdata->keep_mode_max_range[0], pdata->keep_mode_max_range[1],
                        pdata->keep_mode_max_range[2], pdata->keep_mode_max_range[3]);
                pdata->keep_mode = 1;
                pdata->freq_sample_period = 5;
            }
            if (!of_property_read_u32_array(pdev->dev.of_node,
                        "keep_mode_min_range",
                        pdata->keep_mode_min_range,
                        sizeof(pdata->keep_mode_min_range)/sizeof(u32))) {
                pdata->keep_min_exist = 1;
                THERMAL_INFO("keep_mode_min_range:   [%7d, %3d, %d, %d]\n",
                        pdata->keep_mode_min_range[0], pdata->keep_mode_min_range[1],
                        pdata->keep_mode_min_range[2], pdata->keep_mode_min_range[3]);
            }
        } else {
            THERMAL_INFO("keep_mode is disabled\n");
        }
        if(pdata->keep_mode || !pdata->trim_flag){
            INIT_DELAYED_WORK(&pdata->thermal_work, thermal_work);
            schedule_delayed_work(&pdata->thermal_work, msecs_to_jiffies(100));
            atomic_set(&freq_update_flag, 0);
        }

        prop = of_find_property(pdev->dev.of_node, "trip_point", &val);
        if (!prop){
            dev_err(&pdev->dev, "read %s length error\n","trip_point");
            goto err;
        }
        if (pdata->keep_mode) {
            pdata->temp_trip_count = 2;
        } else {
            pdata->temp_trip_count=val/cells/sizeof(u32);
        }
        tmp_level=kzalloc(sizeof(*tmp_level)*pdata->temp_trip_count,GFP_KERNEL);
        pdata->tmp_trip=kzalloc(sizeof(struct temp_trip)*pdata->temp_trip_count,GFP_KERNEL);
        if(!tmp_level){
            goto err;
        }

        if (pdata->keep_mode) {     // keep mode only need one point
            keep_mode_temp_level_init(pdata, tmp_level);
        } else {
            ret=of_property_read_u32_array(pdev->dev.of_node,"trip_point",(u32 *)tmp_level,val/sizeof(u32));
            if (ret){
                dev_err(&pdev->dev, "read %s data error\n","trip_point");
                goto err;
            }
        }
        descend=get_desend();
        for (i = 0; i < pdata->temp_trip_count; i++) {
            pdata->tmp_trip[i].temperature=tmp_level[i].temperature;
            tmp_level[i].cpu_high_freq=fix_to_freq(tmp_level[i].cpu_high_freq,descend);
            pdata->tmp_trip[i].cpu_lower_level=cpufreq_cooling_get_level(0,tmp_level[i].cpu_high_freq);

            tmp_level[i].cpu_low_freq=fix_to_freq(tmp_level[i].cpu_low_freq,descend);
            pdata->tmp_trip[i].cpu_upper_level=cpufreq_cooling_get_level(0,tmp_level[i].cpu_low_freq);
            pdata->tmp_trip[i].gpu_lower_freq=tmp_level[i].gpu_low_freq;
            pdata->tmp_trip[i].gpu_upper_freq=tmp_level[i].gpu_high_freq;

            pdata->tmp_trip[i].cpu_core_num=tmp_level[i].cpu_core_num;
            pdata->tmp_trip[i].gpu_core_num=tmp_level[i].gpu_core_num;
        }

        ret= of_property_read_u32(pdev->dev.of_node, "idle_interval", &val);
        if (ret){
            dev_err(&pdev->dev, "read %s  error\n","idle_interval");
            goto err;
        }
        pdata->idle_interval=val;
        ret=of_property_read_string(pdev->dev.of_node,"device_name",&pdata->name);
        if (ret){
            ret=of_property_read_string(pdev->dev.of_node,"dev_name",&pdata->name);
        }
        if (ret){
            dev_err(&pdev->dev, "read %s  error\n","device_name");
            goto err;
        }
        pdata->mode=THERMAL_DEVICE_ENABLED;
        if(tmp_level)
            kfree(tmp_level);
        return pdata;
    }
err:
    if(tmp_level)
        kfree(tmp_level);
    if(pdata)
        kfree(pdata);
    pdata= NULL;
    return pdata;
}

static struct amlogic_thermal_platform_data * amlogic_thermal_initialize(struct platform_device *pdev, int trim_flag)
{
    struct amlogic_thermal_platform_data *pdata=NULL;
    pdata=amlogic_thermal_init_from_dts(pdev, trim_flag);
    return pdata;
}

static const struct of_device_id amlogic_thermal_match[] = {
    {
        .compatible = "amlogic, amlogic-thermal",
    },
    {},
};

#ifdef CONFIG_HIBERNATION
static int amlogic_thermal_freeze(struct device *dev)
{
    return 0;
}

static int amlogic_thermal_thaw(struct device *dev)
{
    return 0;
}

static int amlogic_thermal_restore(struct device *dev)
{
    thermal_firmware_init();

    return 0;
}

static struct dev_pm_ops amlogic_theraml_pm = {
    .freeze     = amlogic_thermal_freeze,
    .thaw       = amlogic_thermal_thaw,
    .restore    = amlogic_thermal_restore,
};
#endif

static int amlogic_thermal_probe(struct platform_device *pdev)
{
    int ret, trim_flag;
    struct amlogic_thermal_platform_data *pdata=NULL;

    device_rename(&pdev->dev, "thermal");
    dbg_dev = &pdev->dev;
    ret = thermal_firmware_init();
    if (ret < 0) {
        THERMAL_INFO("this chip is not trimmed, can't use thermal\n");
        trim_flag = 0;
        return -ENODEV;
    } else {
        THERMAL_INFO("this chip is trimmed, use thermal\n");
        trim_flag = 1;
    }

    pdata = amlogic_thermal_initialize(pdev, trim_flag);
    if (!pdata) {
        dev_err(&pdev->dev, "Failed to initialize thermal\n");
        goto err;
    }
    mutex_init(&pdata->lock);
    pdev->dev.platform_data=pdata;
    platform_set_drvdata(pdev, pdata);
    ret = amlogic_register_thermal(pdata, pdev);
    if (ret) {
        dev_err(&pdev->dev, "Failed to register thermal interface\n");
        goto err;
    }
    return 0;
err:
    platform_set_drvdata(pdev, NULL);
    return ret;
}

static int amlogic_thermal_remove(struct platform_device *pdev)
{
    struct amlogic_thermal_platform_data *pdata = platform_get_drvdata(pdev);

    aml_virtual_thermal_remove(pdata);

    amlogic_unregister_thermal(pdata);

    platform_set_drvdata(pdev, NULL);

    return 0;
}

struct platform_driver amlogic_thermal_driver = {
    .driver = {
        .name   = "amlogic-thermal",
        .owner  = THIS_MODULE,
    #ifdef CONFIG_HIBERNATION
        .pm     = &amlogic_theraml_pm,
    #endif
        .of_match_table = of_match_ptr(amlogic_thermal_match),
    },
    .probe  = amlogic_thermal_probe,
    .remove = amlogic_thermal_remove,
};

void *aml_get_cdevdata(struct thermal_cooling_device *cdev)
{
    return cdev->devdata;
}
EXPORT_SYMBOL(aml_get_cdevdata);

void aml_set_cdev_update(struct thermal_cooling_device *cdev, bool update)
{
    cdev->updated = update;
}
EXPORT_SYMBOL(aml_set_cdev_update);

void aml_cdev_lockop(struct thermal_cooling_device *cdev, bool lock)
{
    if (lock) {
        thermal_lock(&cdev->lock);
    } else {
        thermal_unlock(&cdev->lock);
    }
}
EXPORT_SYMBOL(aml_cdev_lockop);

void aml_cdev_get_cur_state(struct thermal_cooling_device *cdev, unsigned long *ret)
{
    cdev->ops->get_cur_state(cdev, ret);
}
EXPORT_SYMBOL(aml_cdev_get_cur_state);

static int __init amlogic_thermal_driver_init(void)
{
    return platform_driver_register(&(amlogic_thermal_driver));
}
late_initcall(amlogic_thermal_driver_init);
static void __exit amlogic_thermal_driver_exit(void)
{
    platform_driver_unregister(&(amlogic_thermal_driver) );
}
module_exit(amlogic_thermal_driver_exit);

MODULE_DESCRIPTION("amlogic thermal Driver");
MODULE_AUTHOR("Amlogic SH platform team");
MODULE_ALIAS("platform:amlogic-thermal");
MODULE_LICENSE("GPL");

