// SPDX-License-Identifier: GPL-2.0-only
/* sysfs_node.c
 *
 * Copyright 2020 Google LLC
 */

#include <linux/kobject.h>
#include <linux/sched.h>
#include <linux/sched/task.h>
#include <linux/sysfs.h>
#include <linux/mutex.h>

unsigned int pmu_poll_time_ms = 10;
bool pmu_poll_enabled;
extern void pmu_poll_enable(void);
extern void pmu_poll_disable(void);
static struct mutex sysfs_mutex;
static bool sysfs_node_created;

static ssize_t pmu_poll_time_show(struct kobject *kobj,
					struct kobj_attribute *attr,
					char *buf)
{
	return sysfs_emit(buf, "%u\n", pmu_poll_time_ms);
}

static ssize_t pmu_poll_time_store(struct kobject *kobj,
					struct kobj_attribute *attr,
					const char *buf, size_t count)
{
	unsigned int val;

	if (kstrtouint(buf, 0, &val))
		return -EINVAL;

	if (val < 10 || val > 1000000)
		return -EINVAL;

	pmu_poll_time_ms = val;

	return count;
}

static struct kobj_attribute pmu_poll_time_attribute = __ATTR_RW(pmu_poll_time);

static ssize_t pmu_poll_enable_show(struct kobject *kobj,
					struct kobj_attribute *attr,
					char *buf)
{
	return sysfs_emit(buf, "%s\n", pmu_poll_enabled ? "true" : "false");
}

static ssize_t pmu_poll_enable_store(struct kobject *kobj,
					struct kobj_attribute *attr,
					const char *buf, size_t count)
{
	bool enable;

	if (kstrtobool(buf, &enable))
		return -EINVAL;

	if (pmu_poll_enabled == enable)
		return count;

	if (enable)
		pmu_poll_enable();
	else
		pmu_poll_disable();

	return count;
}

static struct kobj_attribute pmu_poll_enable_attribute = __ATTR_RW(pmu_poll_enable);

static struct kobject *vendor_sched_kobj;
static struct attribute *attrs[] = {
	&pmu_poll_time_attribute.attr,
	&pmu_poll_enable_attribute.attr,
	NULL,
};
static struct attribute_group attr_group = {
	.attrs = attrs,
};

int create_sysfs_node(void)
{
    int ret;

    mutex_lock(&sysfs_mutex);
    if (sysfs_node_created) {
        ret = 0;
        goto out;
    }

    vendor_sched_kobj = kobject_create_and_add("vendor_sched", kernel_kobj);
    if (!vendor_sched_kobj) {
        ret = -ENOMEM;
        goto out;
    }

    ret = sysfs_create_group(vendor_sched_kobj, &attr_group);
    if (ret) {
        kobject_put(vendor_sched_kobj);
        goto out;
    }

    sysfs_node_created = true;
out:
    mutex_unlock(&sysfs_mutex);
    return ret;
}
