// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * CPU Microcode Update Driver for Linux
 *
 * Copyright (C) 2000-2006 Tigran Aivazian <aivazian.tigran@gmail.com>
 *	      2006	Shaohua Li <shaohua.li@intel.com>
 *	      2013-2016	Borislav Petkov <bp@alien8.de>
 *
 * X86 CPU microcode early update for Linux:
 *
 *	Copyright (C) 2012 Fenghua Yu <fenghua.yu@intel.com>
 *			   H Peter Anvin" <hpa@zytor.com>
 *		  (C) 2015 Borislav Petkov <bp@alien8.de>
 *
 * This driver allows to upgrade microcode on x86 processors.
 */

#define pr_fmt(fmt) "microcode: " fmt

#include <linux/platform_device.h>
#include <linux/stop_machine.h>
#include <linux/syscore_ops.h>
#include <linux/miscdevice.h>
#include <linux/capability.h>
#include <linux/firmware.h>
#include <linux/debugfs.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/cpu.h>
#include <linux/nmi.h>
#include <linux/fs.h>
#include <linux/mm.h>

#include <asm/microcode_intel.h>
#include <asm/cpu_device_id.h>
#include <asm/microcode_amd.h>
#include <asm/perf_event.h>
#include <asm/microcode.h>
#include <asm/processor.h>
#include <asm/cmdline.h>
#include <asm/setup.h>

#define DRIVER_VERSION	"2.2"

static struct microcode_ops	*microcode_ops;
static bool dis_ucode_ldr = true;

static struct dentry *dentry_ucode;
bool override_minrev;
bool ucode_load_same;

bool initrd_gone;

LIST_HEAD(microcode_cache);

/*
 * Synchronization.
 *
 * All non cpu-hotplug-callback call sites use:
 *
 * - microcode_mutex to synchronize with each other;
 * - cpus_read_lock/unlock() to synchronize with
 *   the cpu-hotplug-callback call sites.
 *
 * We guarantee that only a single cpu is being
 * updated at any particular moment of time.
 */
static DEFINE_MUTEX(microcode_mutex);

struct ucode_cpu_info		ucode_cpu_info[NR_CPUS];

struct cpu_info_ctx {
	struct cpu_signature	*cpu_sig;
	int			err;
};

/*
 * Those patch levels cannot be updated to newer ones and thus should be final.
 */
static u32 final_levels[] = {
	0x01000098,
	0x0100009f,
	0x010000af,
	0, /* T-101 terminator */
};

/*
 * Check the current patch level on this CPU.
 *
 * Returns:
 *  - true: if update should stop
 *  - false: otherwise
 */
static bool amd_check_current_patch_level(void)
{
	u32 lvl, dummy, i;
	u32 *levels;

	native_rdmsr(MSR_AMD64_PATCH_LEVEL, lvl, dummy);

	if (IS_ENABLED(CONFIG_X86_32))
		levels = (u32 *)__pa_nodebug(&final_levels);
	else
		levels = final_levels;

	for (i = 0; levels[i]; i++) {
		if (lvl == levels[i])
			return true;
	}
	return false;
}

static bool __init check_loader_disabled_bsp(void)
{
	static const char *__dis_opt_str = "dis_ucode_ldr";

#ifdef CONFIG_X86_32
	const char *cmdline = (const char *)__pa_nodebug(boot_command_line);
	const char *option  = (const char *)__pa_nodebug(__dis_opt_str);
	bool *res = (bool *)__pa_nodebug(&dis_ucode_ldr);

#else /* CONFIG_X86_64 */
	const char *cmdline = boot_command_line;
	const char *option  = __dis_opt_str;
	bool *res = &dis_ucode_ldr;
#endif

	/*
	 * CPUID(1).ECX[31]: reserved for hypervisor use. This is still not
	 * completely accurate as xen pv guests don't see that CPUID bit set but
	 * that's good enough as they don't land on the BSP path anyway.
	 */
	if (native_cpuid_ecx(1) & BIT(31))
		return *res;

	if (x86_cpuid_vendor() == X86_VENDOR_AMD) {
		if (amd_check_current_patch_level())
			return *res;
	}

	if (cmdline_find_option_bool(cmdline, option) <= 0)
		*res = false;

	return *res;
}

void __init load_ucode_bsp(void)
{
	unsigned int cpuid_1_eax;
	bool intel = true;

	if (!have_cpuid_p())
		return;

	cpuid_1_eax = native_cpuid_eax(1);

	switch (x86_cpuid_vendor()) {
	case X86_VENDOR_INTEL:
		if (x86_family(cpuid_1_eax) < 6)
			return;
		break;

	case X86_VENDOR_AMD:
		if (x86_family(cpuid_1_eax) < 0x10)
			return;
		intel = false;
		break;

	default:
		return;
	}

	if (check_loader_disabled_bsp())
		return;

	if (intel)
		load_ucode_intel_bsp();
	else
		load_ucode_amd_bsp(cpuid_1_eax);
}

static bool check_loader_disabled_ap(void)
{
#ifdef CONFIG_X86_32
	return *((bool *)__pa_nodebug(&dis_ucode_ldr));
#else
	return dis_ucode_ldr;
#endif
}

void load_ucode_ap(void)
{
	unsigned int cpuid_1_eax;

	if (check_loader_disabled_ap())
		return;

	cpuid_1_eax = native_cpuid_eax(1);

	switch (x86_cpuid_vendor()) {
	case X86_VENDOR_INTEL:
		if (x86_family(cpuid_1_eax) >= 6)
			load_ucode_intel_ap();
		break;
	case X86_VENDOR_AMD:
		if (x86_family(cpuid_1_eax) >= 0x10)
			load_ucode_amd_ap(cpuid_1_eax);
		break;
	default:
		break;
	}
}

static int __init save_microcode_in_initrd(void)
{
	struct cpuinfo_x86 *c = &boot_cpu_data;
	int ret = -EINVAL;

	switch (c->x86_vendor) {
	case X86_VENDOR_INTEL:
		if (c->x86 >= 6)
			ret = save_microcode_in_initrd_intel();
		break;
	case X86_VENDOR_AMD:
		if (c->x86 >= 0x10)
			ret = save_microcode_in_initrd_amd(cpuid_eax(1));
		break;
	default:
		break;
	}

	initrd_gone = true;

	return ret;
}

struct cpio_data find_microcode_in_initrd(const char *path, bool use_pa)
{
#ifdef CONFIG_BLK_DEV_INITRD
	unsigned long start = 0;
	size_t size;

#ifdef CONFIG_X86_32
	struct boot_params *params;

	if (use_pa)
		params = (struct boot_params *)__pa_nodebug(&boot_params);
	else
		params = &boot_params;

	size = params->hdr.ramdisk_size;

	/*
	 * Set start only if we have an initrd image. We cannot use initrd_start
	 * because it is not set that early yet.
	 */
	if (size)
		start = params->hdr.ramdisk_image;

# else /* CONFIG_X86_64 */
	size  = (unsigned long)boot_params.ext_ramdisk_size << 32;
	size |= boot_params.hdr.ramdisk_size;

	if (size) {
		start  = (unsigned long)boot_params.ext_ramdisk_image << 32;
		start |= boot_params.hdr.ramdisk_image;

		start += PAGE_OFFSET;
	}
# endif

	/*
	 * Fixup the start address: after reserve_initrd() runs, initrd_start
	 * has the virtual address of the beginning of the initrd. It also
	 * possibly relocates the ramdisk. In either case, initrd_start contains
	 * the updated address so use that instead.
	 *
	 * initrd_gone is for the hotplug case where we've thrown out initrd
	 * already.
	 */
	if (!use_pa) {
		if (initrd_gone)
			return (struct cpio_data){ NULL, 0, "" };
		if (initrd_start)
			start = initrd_start;
	} else {
		/*
		 * The picture with physical addresses is a bit different: we
		 * need to get the *physical* address to which the ramdisk was
		 * relocated, i.e., relocated_ramdisk (not initrd_start) and
		 * since we're running from physical addresses, we need to access
		 * relocated_ramdisk through its *physical* address too.
		 */
		u64 *rr = (u64 *)__pa_nodebug(&relocated_ramdisk);
		if (*rr)
			start = *rr;
	}

	return find_cpio_data(path, (void *)start, size, NULL);
#else /* !CONFIG_BLK_DEV_INITRD */
	return (struct cpio_data){ NULL, 0, "" };
#endif
}

void reload_early_microcode(unsigned int cpu)
{
	int vendor, family;

	vendor = x86_cpuid_vendor();
	family = x86_cpuid_family();

	switch (vendor) {
	case X86_VENDOR_INTEL:
		if (family >= 6)
			reload_ucode_intel();
		break;
	case X86_VENDOR_AMD:
		if (family >= 0x10)
			reload_ucode_amd(cpu);
		break;
	default:
		break;
	}
}

/* fake device for request_firmware */
static struct platform_device	*microcode_pdev;

#ifdef CONFIG_MICROCODE_LATE_LOADING
/*
 * Late loading dance. Why the heavy-handed stomp_machine effort?
 *
 * - HT siblings must be idle and not execute other code while the other sibling
 *   is loading microcode in order to avoid any negative interactions caused by
 *   the loading.
 *
 * - In addition, microcode update on the cores must be serialized until this
 *   requirement can be relaxed in the future. Right now, this is conservative
 *   and good.
 */
#define SPINUNIT 100 /* 100 nsec */

static int check_online_cpus(void)
{
	unsigned int cpu;

	/*
	 * Make sure all CPUs are online.  It's fine for SMT to be disabled if
	 * all the primary threads are still online.
	 */
	for_each_present_cpu(cpu) {
		if (topology_is_primary_thread(cpu) && !cpu_online(cpu)) {
			pr_err("Not all CPUs online, aborting microcode update.\n");
			return -EBUSY;
		}
	}

	return 0;
}

static atomic_t late_cpus_in;
static atomic_t late_cpus_out;

static int __wait_for_cpus(atomic_t *t, long long timeout)
{
	int all_cpus = num_online_cpus();

	atomic_inc(t);

	while (atomic_read(t) < all_cpus) {
		if (timeout < SPINUNIT) {
			pr_err("Timeout while waiting for CPUs rendezvous, remaining: %d\n",
				all_cpus - atomic_read(t));
			return 1;
		}

		ndelay(SPINUNIT);
		timeout -= SPINUNIT;

		touch_nmi_watchdog();
	}
	return 0;
}

static void update_cpuinfo_x86(int cpu)
{
	struct cpuinfo_x86 *c = &cpu_data(cpu);
	bool bsp;

	bsp = c->cpu_index == boot_cpu_data.cpu_index;
	c->microcode = microcode_ops->get_current_rev();

	/* Update boot_cpu_data's revision too, if we're on the BSP: */
	if (bsp)
		boot_cpu_data.microcode = c->microcode;
}

static enum ucode_state apply_microcode(int cpu)
{
	enum ucode_state err;

	err = microcode_ops->apply_microcode(cpu);
	update_cpuinfo_x86(cpu);

	return err;
}

static enum ucode_load_scope load_scope;
static enum ucode_load_scope get_load_scope(void)
{
	if (!load_scope) {
		load_scope = microcode_ops->get_load_scope ?
				microcode_ops->get_load_scope() : CORE_SCOPE;
	}

	return load_scope;
}

static int get_target_cpu(int cpu)
{
	switch (load_scope) {
	case CORE_SCOPE:
		return cpumask_first(topology_sibling_cpumask(cpu));
	case PACKAGE_SCOPE:
		return cpumask_first(topology_core_cpumask(cpu));
	case PLATFORM_SCOPE:
		return cpumask_first(cpu_online_mask);
	default:
		return 0;
	}
}

static atomic_t ucode_updating;
static bool mce_in_progress;
void noinstr inform_ucode_mce_in_progress(void)
{
	if (arch_atomic_read(&ucode_updating))
		mce_in_progress = true;
}

/*
 * Returns:
 * < 0 - on error
 *   0 - success (no update done or microcode was updated)
 */
static int __reload_late(void *info)
{
	int first_cpu, cpu = smp_processor_id();
	enum ucode_state err;
	bool lead_thread;
	bool load_both;
	int ret = 0;
	struct cpuinfo_x86 *bsp_info = &boot_cpu_data;
	struct cpuinfo_x86 *this_cpu_info;

	/*
	 * Wait for all CPUs to arrive. A load will not be attempted unless all
	 * CPUs show up.
	 * */
	if (__wait_for_cpus(&late_cpus_in, NSEC_PER_SEC))
		return -1;

	/*
	 * On an SMT system, it suffices to load the microcode on one sibling of
	 * the core because the microcode engine is shared between the threads.
	 * Synchronization still needs to take place so that no concurrent
	 * loading attempts happen on multiple threads of an SMT core. See
	 * below.
	 */
	first_cpu = get_target_cpu(cpu);
	if (first_cpu == cpu) {
		lead_thread = true;
		err = apply_microcode(cpu);
	} else {
		lead_thread = false;
		goto wait_for_siblings;
	}

	if (err >= UCODE_NFOUND) {
		if (err == UCODE_ERROR) {
			pr_warn("Error reloading microcode on CPU %d\n", cpu);
			ret = -1;
		}
	}

wait_for_siblings:
	if (__wait_for_cpus(&late_cpus_out, NSEC_PER_SEC))
		panic("Timeout during microcode update!\n");

	load_both = microcode_ops->get_control_flags() & LATE_LOAD_BOTH;
	/*
	 * The lead thread has completed update on each core.
	 * For others, simply update the per-cpu cpuinfo
	 * with microcode revision.
	 */
	if (!lead_thread) {
		if (load_both)
			apply_microcode(cpu);
		else
			update_cpuinfo_x86(cpu);
	}

	/* When Uniform update is enabled, check if update applied on all CPUs */
	this_cpu_info = &cpu_data(cpu);
	if (load_scope > CORE_SCOPE &&  this_cpu_info->microcode != bsp_info->microcode) {
		pr_err("Microcode Revision for CPU %d = 0x%x doesn't match BSP rev 0x%x\n",
		       cpu, this_cpu_info->microcode, bsp_info->microcode);
		ret = -1;
	}

	return ret;
}

/*
 * Reload microcode late on all CPUs. Wait for a sec until they
 * all gather together.
 */
static int microcode_reload_late(void)
{
	int old = boot_cpu_data.microcode, ret;
	struct cpuinfo_x86 prev_info;

	atomic_set(&late_cpus_in,  0);
	atomic_set(&late_cpus_out, 0);

	/*
	 * Take a snapshot before the microcode update in order to compare and
	 * check whether any bits changed after an update.
	 */
	store_cpu_caps(&prev_info);

	/* Track if MCE occurred during update */
	mce_in_progress = false;
	atomic_set(&ucode_updating, 1);

	ret = stop_machine_cpuslocked(__reload_late, NULL, cpu_online_mask);

	if (mce_in_progress) {
		pr_warn("MCE occurred while microcode update was in progress\n");
		mce_in_progress = false;
	}
	atomic_set(&ucode_updating, 0);

	if (!ret) {
		pr_info("Reload succeeded, microcode revision: 0x%x -> 0x%x\n",
			old, boot_cpu_data.microcode);
		microcode_check(&prev_info);
	} else {
		pr_info("Reload failed, current microcode revision: 0x%x\n",
			boot_cpu_data.microcode);
	}

	return ret;
}

static bool is_lateload_safe(void)
{
	return (microcode_ops->get_control_flags() & LATE_LOAD_SAFE);
}

static ssize_t reload_store(struct device *dev,
			    struct device_attribute *attr,
			    const char *buf, size_t size)
{
	enum ucode_state tmp_ret = UCODE_OK;
	int bsp = boot_cpu_data.cpu_index;
	bool safe_late_load;
	bool load_success = false;
	unsigned long val;
	ssize_t ret;
	enum ucode_load_scope load_scope;

	ret = kstrtoul(buf, 0, &val);
	if (ret || val != 1)
		return -EINVAL;

	load_scope = get_load_scope();
	if (load_scope == NO_LATE_UPDATE) {
		pr_err_once("Platform doesn't support late loading!\n");
		pr_err_once("Please contact your BIOS vendor\n");
		return size;
	}

	cpus_read_lock();

	ret = check_online_cpus();
	if (ret)
		goto unlock;

	tmp_ret = microcode_ops->request_microcode_fw(bsp, &microcode_pdev->dev);
	if (tmp_ret != UCODE_NEW) {
		if (tmp_ret == UCODE_ERROR) {
			ret = -EBADF;
			goto unlock;
		}

		if (tmp_ret == UCODE_NFOUND) {
			ret = -ENOENT;
			goto unlock;
		}

		pr_warn("Force loading same microcode\n");
	}

	safe_late_load = is_lateload_safe();

	/*
	 * If safe loading indication isn't present, bail out.
	 */
	if (!safe_late_load || override_minrev) {
		pr_err("Attempting late microcode loading - it is dangerous and taints the kernel.\n");
		pr_err("You should switch to early loading.\n");

		if (!override_minrev) {
			ret = -EINVAL;
			goto unlock;
		}
	}

	mutex_lock(&microcode_mutex);

	ret = microcode_reload_late();

	if (microcode_ops->post_apply)
		microcode_ops->post_apply(!ret);

	mutex_unlock(&microcode_mutex);
	if (ret) {
		ret = -EIO;
		goto unlock;
	}

	load_success = true;
	ret = size;

unlock:
	cpus_read_unlock();

	/* Taint only when loading was successful */
	if (load_success) {
		if (!safe_late_load || override_minrev)
			add_taint(TAINT_CPU_OUT_OF_SPEC, LOCKDEP_STILL_OK);
	}

	return ret;
}

static ssize_t control_show(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "0x%x\n", microcode_ops->get_control_flags());
}

static DEVICE_ATTR_WO(reload);
static DEVICE_ATTR_RO(control);
#endif

static ssize_t version_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct ucode_cpu_info *uci = ucode_cpu_info + dev->id;

	return sprintf(buf, "0x%x\n", uci->cpu_sig.rev);
}

static ssize_t processor_flags_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct ucode_cpu_info *uci = ucode_cpu_info + dev->id;

	return sprintf(buf, "0x%x\n", uci->cpu_sig.pf);
}

static DEVICE_ATTR_RO(version);
static DEVICE_ATTR_RO(processor_flags);

static struct attribute *mc_default_attrs[] = {
	&dev_attr_version.attr,
	&dev_attr_processor_flags.attr,
	NULL
};

static const struct attribute_group mc_attr_group = {
	.attrs			= mc_default_attrs,
	.name			= "microcode",
};

static void microcode_fini_cpu(int cpu)
{
	if (microcode_ops->microcode_fini_cpu)
		microcode_ops->microcode_fini_cpu(cpu);
}

static enum ucode_state microcode_init_cpu(int cpu)
{
	struct ucode_cpu_info *uci = ucode_cpu_info + cpu;

	memset(uci, 0, sizeof(*uci));

	microcode_ops->collect_cpu_info(cpu, &uci->cpu_sig);

	return apply_microcode(cpu);
}

/**
 * microcode_bsp_resume - Update boot CPU microcode during resume.
 */
void microcode_bsp_resume(void)
{
	int cpu = smp_processor_id();
	struct ucode_cpu_info *uci = ucode_cpu_info + cpu;

	if (uci->mc)
		apply_microcode(cpu);
	else
		reload_early_microcode(cpu);
}

static struct syscore_ops mc_syscore_ops = {
	.resume	= microcode_bsp_resume,
};

static int mc_cpu_starting(unsigned int cpu)
{
	enum ucode_state err = apply_microcode(cpu);

	pr_debug("%s: CPU%d, err: %d\n", __func__, cpu, err);

	return err == UCODE_ERROR;
}

static int mc_cpu_online(unsigned int cpu)
{
	struct device *dev = get_cpu_device(cpu);

	if (sysfs_create_group(&dev->kobj, &mc_attr_group))
		pr_err("Failed to create group for CPU%d\n", cpu);
	return 0;
}

static int mc_cpu_down_prep(unsigned int cpu)
{
	struct device *dev;

	dev = get_cpu_device(cpu);

	microcode_fini_cpu(cpu);

	/* Suspend is in progress, only remove the interface */
	sysfs_remove_group(&dev->kobj, &mc_attr_group);
	pr_debug("%s: CPU%d\n", __func__, cpu);

	return 0;
}

static bool ucode_update_success;
static void setup_online_cpu(struct work_struct *work)
{
	int cpu = smp_processor_id();
	enum ucode_state err;

	err = microcode_init_cpu(cpu);
	if (err == UCODE_ERROR) {
		pr_err("Error applying microcode on CPU%d\n", cpu);
		return;
	}

	if (err == UCODE_UPDATED)
		ucode_update_success = true;

	mc_cpu_online(cpu);
}

static struct attribute *cpu_root_microcode_attrs[] = {
#ifdef CONFIG_MICROCODE_LATE_LOADING
	&dev_attr_reload.attr,
	&dev_attr_control.attr,
#endif
	NULL
};

static const struct attribute_group cpu_root_microcode_group = {
	.name  = "microcode",
	.attrs = cpu_root_microcode_attrs,
};

static int __init microcode_init(void)
{
	struct device *dev_root;
	struct cpuinfo_x86 *c = &boot_cpu_data;
	int error;
	int ret;

	if (dis_ucode_ldr)
		return -EINVAL;

	if (c->x86_vendor == X86_VENDOR_INTEL)
		microcode_ops = init_intel_microcode();
	else if (c->x86_vendor == X86_VENDOR_AMD)
		microcode_ops = init_amd_microcode();
	else
		pr_err("no support for this CPU vendor\n");

	if (!microcode_ops)
		return -ENODEV;

	microcode_pdev = platform_device_register_simple("microcode", -1, NULL, 0);
	if (IS_ERR(microcode_pdev))
		return PTR_ERR(microcode_pdev);

	dev_root = bus_get_dev_root(&cpu_subsys);
	if (dev_root) {
		error = sysfs_create_group(&dev_root->kobj, &cpu_root_microcode_group);
		put_device(dev_root);
		if (error) {
			pr_err("Error creating microcode group!\n");
			goto out_pdev;
		}
	}

	/* Do per-CPU setup */
	ucode_update_success = false;
	ret = schedule_on_each_cpu(setup_online_cpu);

	/* Update cached ucode to reflect the recently applied ucode */
	if (!ret && ucode_update_success && microcode_ops->post_apply)
		microcode_ops->post_apply(true);

	register_syscore_ops(&mc_syscore_ops);
	cpuhp_setup_state_nocalls(CPUHP_AP_MICROCODE_LOADER, "x86/microcode:starting",
				  mc_cpu_starting, NULL);
	cpuhp_setup_state_nocalls(CPUHP_AP_ONLINE_DYN, "x86/microcode:online",
				  mc_cpu_online, mc_cpu_down_prep);

	dentry_ucode = debugfs_create_dir("microcode", NULL);

	debugfs_create_bool("override_minrev", 0644, dentry_ucode, &override_minrev);
	debugfs_create_bool("load_same", 0644, dentry_ucode, &ucode_load_same);

	pr_info("Microcode Update Driver: v%s.", DRIVER_VERSION);

	return 0;

 out_pdev:
	platform_device_unregister(microcode_pdev);
	return error;

}
fs_initcall(save_microcode_in_initrd);
late_initcall(microcode_init);
