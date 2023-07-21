// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Intel CPU Microcode Update Driver for Linux
 *
 * Copyright (C) 2000-2006 Tigran Aivazian <aivazian.tigran@gmail.com>
 *		 2006 Shaohua Li <shaohua.li@intel.com>
 *
 * Intel CPU microcode early update for Linux
 *
 * Copyright (C) 2012 Fenghua Yu <fenghua.yu@intel.com>
 *		      H Peter Anvin" <hpa@zytor.com>
 */

/*
 * This needs to be before all headers so that pr_debug in printk.h doesn't turn
 * printk calls into no_printk().
 *
 *#define DEBUG
 */
#define pr_fmt(fmt) "microcode: " fmt

#include <linux/earlycpio.h>
#include <linux/firmware.h>
#include <linux/uaccess.h>
#include <linux/vmalloc.h>
#include <linux/initrd.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/cpu.h>
#include <linux/uio.h>
#include <linux/mm.h>
#include <linux/bitops.h>
#include <linux/debugfs.h>

#include <asm/microcode_intel.h>
#include <asm/intel-family.h>
#include <asm/msr-index.h>
#include <asm/processor.h>
#include <asm/tlbflush.h>
#include <asm/setup.h>
#include <asm/msr.h>

#include "doe.h"

static const char ucode_path[] = "kernel/x86/microcode/GenuineIntel.bin";

static bool ucode_staging = true;
extern struct dentry *dentry_ucode;

/*
 * Holds the microcode whose SVN is committed.
 */
static struct microcode_intel *committed_ucode;

/*
 * Always hold the currently applied microcode in the CPU. mc->hdr.rev
 * should match whats in the CPU revision MSR but SVN may not be committed yet.
 */
static struct microcode_intel *applied_ucode;

/*
 * Holds the microcode read from the file system and is yet to be applied to
 * the CPU. This allows post_apply() to free it in case the applying microcode
 * fails.
 */
static struct microcode_intel *unapplied_ucode;

struct ucode_info {
	struct microcode_intel *ucode;
	int size;
};

/* last level cache size per core */
static int llc_size_per_core;

enum scope {
	UNIFORM_CORE     = 0x02, // Core Scope
	UNIFORM_PACKAGE  = 0x80, // Package Scope
	UNIFORM_PLATFORM = 0xC0, // Platform Scope
};

union mcu_enumeration {
	u64	data;
	struct {
		u64	uniform_available:1;
		u64	cfg_required:1;
		u64	cfg_completed:1;
		u64	rollback_supported:1;
		u64	staging_supported:1;
		u64	reserved:3;
		u64	uniform_scope:8;
	};
};

union mcu_status {
	u64	data;
	struct {
		u64	partial:1;
		u64	auth_fail:1;
		u64	rsvd:1;
		u64	post_bios_mcu:1;
	};
};

/*
 * MSR's related to deferred commit architecture
 */
#define MSR_MCU_CONFIG		(0x7a0)
#define MSR_MCU_COMMIT		(0x7a1)
#define MSR_MCU_INFO		(0x7a2)

/* MSR_MCU_CONFIG */
union svn_config {
	u64	data;
	struct {
		u64	lock:1;
		u64	defer_svn:1;
		u64	rsvd:62;
	};
};

/* MSR_MCU_COMMIT */
union svn_commit {
	u64	data;
	struct {
		u64	commit_svn:1;
		u64	rsvd:63;
	};
};

/* MSR_MCU_SVN_INFO */
union svn_info {
	u64	data;
	struct {
		u64	committed_mcu_svn:16;
		u64	pending_mcu_svn:16;
		u64	rsvd:32;
	};
};

/*
 * MSR's related to rollback architecture
 */
#define MSR_MCU_ROLLBACK_MIN_ID	(0x7a4)

/* MSR_MCU_ROLLBACK_MIN_ID */
union rollback_min_id {
	u64	data;
	struct {
		u64	min_rev_id:32;
		u64	rsvd:32;
	};
};

/* Rollback metadata block */
struct rb_svn_info {
	u32	mcu_svn:16;
	u32	min_mcu_svn:16;
};

#define NUM_RB_INFO	16
struct ucode_meta {
	struct	metadata_header	rb_hdr;
	struct  rb_svn_info svn_info;
	u32	rollback_id[NUM_RB_INFO];
	u16	rollback_svn[NUM_RB_INFO];
};

enum commit_cfg {
	AUTO_COMMIT = 0,
	MANUAL_COMMIT
};

/**
 * struct mcu_staging -  Information of per package staging mailbox instances
 *
 * @mboxes    : Array of per package staging mailbox instances
 * @work      : Work to stage microcode
 * @scheduled : Whether the work to stage microcode is scheduled
 * @creating  : Whether the mailbox instance is being created
 * @result    : Result of microcode staging by @work
 * @mbox_num  : Number of staging mailbox instances
 */
struct mcu_staging {
	struct {
		struct uc_doe_mbox *mbox;
		struct work_struct work;
		bool scheduled;
		unsigned long creating;
		enum ucode_state result;
	} *mboxes;
	int mbox_num;
};

#define MSR_MCU_ENUM		(0x7b)
#define MSR_MCU_STATUS		(0x7c)
#define MSR_MCU_MBOX_ADDR	(0x7a5)

#define META_TYPE_ROLLBACK	(0x2)

static union mcu_enumeration mcu_cap;
static struct mcu_staging mcu_staging;

/* Vendor specific ucode control flags */
static enum late_load_flags intel_ucode_control;

union rollback_min_id rollback_rev;

static union svn_info bsp_svn_info;
static void save_bsp_svn_info(void)
{
	if (!mcu_cap.rollback_supported)
		return;

	rdmsrl(MSR_MCU_INFO, bsp_svn_info.data);
	pr_debug("committed_svn: 0x%x pending_svn: 0x%x\n",
		 bsp_svn_info.committed_mcu_svn, bsp_svn_info.pending_mcu_svn);
}

static bool pending_commit;
static void read_commit_status(struct work_struct *work)
{
	union svn_commit commit;

	rdmsrl(MSR_MCU_COMMIT, commit.data);
	if (commit.commit_svn && !pending_commit)
		pending_commit = 1;
}

static int check_pending(void)
{
	int ret = 0;

	pending_commit = 0;
	ret = schedule_on_each_cpu_locked(read_commit_status);

	if (!ret && pending_commit) {
		pr_err("Pending commit, Please commit before proceeding\n");
		ret = -EBUSY;
	}

	return ret;
}

static bool check_pending_commits(void)
{
	int ret;

	if (!mcu_cap.rollback_supported)
		return false;

	ret = check_pending();
	return (ret == 0 ? false : true);
}

static long read_msr_mcu_config(void *arg)
{
	union svn_config *cfg = arg;

	rdmsrl(MSR_MCU_CONFIG, cfg->data);
	return 0;
}

static long write_msr_mcu_config(void *arg)
{
	union svn_config *cfg = arg;

	wrmsrl(MSR_MCU_CONFIG, cfg->data);
	return 0;
}

static int switch_mcu_commit_config(bool commit_cfg)
{
	union svn_config cfg;
	int ret, cpu, first_cpu;

	if (!mcu_cap.rollback_supported)
		return 0;

	ret = check_pending();
	if (ret)
		return ret;

	for_each_online_cpu(cpu) {
		first_cpu = cpumask_first(topology_core_cpumask(cpu));
		if (cpu != first_cpu)
			continue;

		cfg.data = 0;
		work_on_cpu(cpu, read_msr_mcu_config, &cfg);

		if (cfg.defer_svn == commit_cfg)
			continue;

		/*
		 * Admin has locked commit mode configuration, it is not a
		 * preferred setting since booting a new kernel via kexec()
		 * will not know how to deal in case of manual commit.
		 */
		if (cfg.lock) {
			pr_err_once("mcu commit configuration locked, can't switch!\n");
			return -EBUSY;
		}

		cfg.defer_svn = commit_cfg;
		work_on_cpu(cpu, write_msr_mcu_config, &cfg);
	}

	return ret;
}

static inline void clear_ucode_store(struct microcode_intel **ucode)
{
	*ucode = NULL;
}

static void free_ucode_store(struct microcode_intel **ucode)
{
	kfree(*ucode);
	clear_ucode_store(ucode);
}

static enum ucode_load_scope get_load_scope(void)
{
	/*
	 * If no capability is found, default to CORE scope
	 */
	if (!mcu_cap.uniform_available)
		return CORE_SCOPE;

	/*
	 * If enumeration requires UNIFORM and the platform configuration
	 * is not complete, disable any further attempt to late loading.
	 */
	if (mcu_cap.cfg_required && !mcu_cap.cfg_completed)
		return NO_LATE_UPDATE;

	if (mcu_cap.uniform_scope == UNIFORM_PLATFORM)
		return PLATFORM_SCOPE;

	if (mcu_cap.uniform_scope == UNIFORM_PACKAGE)
		return PACKAGE_SCOPE;

	return CORE_SCOPE;
}

static enum ucode_state get_apply_status(void)
{
	enum ucode_state ret = UCODE_UPDATED;
	union mcu_status status;

	if (!mcu_cap.data)
		return ret;

	status.data = 0;
	rdmsrl(MSR_MCU_STATUS, status.data);

	/*
	 * AUTH_FAIL is evil, best to trigger reset
	 * PARTIAL update is ok, but OS policy is TBD.
	 */
	if (status.auth_fail)
		ret = UCODE_UPDATED_AUTH;
	else if (status.partial)
		ret = UCODE_UPDATED_PART;
	else if (!status.post_bios_mcu)
		ret = UCODE_ERROR;

	return ret;
}

/*
 * Returns 1 if update has been found, 0 otherwise.
 */
static int has_newer_microcode(void *mc, unsigned int csig, int cpf, int new_rev)
{
	struct microcode_header_intel *mc_hdr = mc;

	if (mc_hdr->rev <= new_rev)
		return 0;

	return intel_find_matching_signature(mc, csig, cpf);
}

static void save_microcode_patch(struct microcode_intel **ucode_ptr, void *data, unsigned int size)
{
	struct microcode_header_intel *p;

	if (!(ucode_ptr && data))
		return;

	kfree(*ucode_ptr);
	*ucode_ptr = NULL;

	p = kmemdup(data, size, GFP_KERNEL);
	if (!p)
		return;

	/*
	 * Save for early loading. On 32-bit, that needs to be a physical
	 * address as the APs are running from physical addresses, before
	 * paging has been enabled.
	 */
	if (IS_ENABLED(CONFIG_X86_32))
		*ucode_ptr = (struct microcode_intel *)__pa_nodebug(p);
	else
		*ucode_ptr = (struct microcode_intel *)p;
}

static int __is_lateload_safe(struct microcode_header_intel *mc_header)
{
	int cur_rev = boot_cpu_data.microcode;

	/*
	 * If minrev is bypassed via debugfs, then allow late-load.
	 */
	if (override_minrev) {
		pr_warn_once("Bypassing minrev enforcement via debugfs\n");
		return 0;
	}

	/*
	 * When late-loading, ensure the header declares a minimum revision
	 * required to perform a late-load.
	 */
	if (!mc_header->min_req_ver) {
		pr_warn("Late loading denied: No min version in microcode header\n");
		return -EINVAL;
	}

	if (cur_rev > mc_header->rev) {
		pr_warn("Current microcode rev 0x%x greater than 0x%x, aborting\n",
			cur_rev, mc_header->rev);
		return -EINVAL;
	}

	/*
	 * Enforce the minimum revision specified in the header is either
	 * greater or equal to the current revision.
	 */
	if (cur_rev < mc_header->min_req_ver) {
		pr_warn("Late loading denied: Current revision 0x%x too old to update\n", cur_rev);
		pr_warn("Must be at 0x%x or higher. Use early loading instead\n",
			mc_header->min_req_ver);
		return -EINVAL;
	}

	return 0;
}

/*
 * Get microcode matching with BSP's model. Only CPUs with the same model as
 * BSP can stay in the platform.
 */
static struct ucode_info
scan_microcode(void *data, size_t size, struct ucode_cpu_info *uci, bool save)
{
	struct microcode_header_intel *mc_header;
	struct ucode_info patch = {0};
	unsigned int mc_size;

	while (size) {
		if (size < sizeof(struct microcode_header_intel))
			break;

		mc_header = (struct microcode_header_intel *)data;

		mc_size = get_totalsize(mc_header);
		if (!mc_size ||
		    mc_size > size ||
		    intel_microcode_sanity_check(data, false, MC_HEADER_TYPE_MICROCODE) < 0)
			break;

		size -= mc_size;

		if (!intel_find_matching_signature(data, uci->cpu_sig.sig,
						   uci->cpu_sig.pf)) {
			data += mc_size;
			continue;
		}

		if (save) {
			save_microcode_patch(&unapplied_ucode, data, mc_size);
			goto next;
		}

		if (!patch.ucode) {
			/*
			 * Save patch even if it matches what's loaded.
			 * This is done for platforms supporting uniform update,
			 * so that APs can store the ucode patch found during
			 * early load.
			 */
			if (uci->cpu_sig.rev == mc_header->rev)
				goto save;

			if (!has_newer_microcode(data,
						 uci->cpu_sig.sig,
						 uci->cpu_sig.pf,
						 uci->cpu_sig.rev))
				goto next;

		} else {
			struct microcode_header_intel *phdr = &patch.ucode->hdr;

			if (!has_newer_microcode(data,
						 phdr->sig,
						 phdr->pf,
						 phdr->rev))
				goto next;
		}

save:
		patch.ucode = data;
		patch.size = mc_size;

next:
		data += mc_size;
	}

	return patch;
}

#ifdef DEBUG
static void dump_rollback_meta(struct ucode_meta *rb)
{
	int i;

	pr_debug("Type    : 0x%x\n", rb->rb_hdr.type);
	pr_debug("Block SZ: 0x%x\n", rb->rb_hdr.blk_size);
	pr_debug("MCU SVN : 0x%x\n", rb->svn_info.mcu_svn);
	pr_debug("Min SVN : 0x%x\n", rb->svn_info.min_mcu_svn);

	for (i = 0; i < NUM_RB_INFO; i++) {
		if (!rb->rollback_id[i])
			break;
		pr_debug("Rollback[%d]: ID: 0x%x SVN 0x%x\n", i, rb->rollback_id[i],
			 rb->rollback_svn[i]);
	}
}
#endif

static void show_saved_mc(void *mc)
{
#ifdef DEBUG
	struct microcode_intel *ucode = mc;
	unsigned int sig, pf, rev, total_size, data_size, date;
	struct extended_sigtable *ext_header;
	struct extended_signature *ext_sig;
	struct ucode_cpu_info uci;
	int j, ext_sigcount;
	struct ucode_meta *rb_meta;

	if (!ucode) {
		pr_debug("no microcode data saved.\n");
		return;
	}

	intel_cpu_collect_info(&uci);

	sig	= uci.cpu_sig.sig;
	pf	= uci.cpu_sig.pf;
	rev	= uci.cpu_sig.rev;
	pr_debug("CPU: sig=0x%x, pf=0x%x, rev=0x%x\n", sig, pf, rev);

	sig	= ucode->hdr.sig;
	pf	= ucode->hdr.pf;
	rev	= ucode->hdr.rev;
	date	= ucode->hdr.date;

	total_size	= get_totalsize(ucode);
	data_size	= get_datasize(ucode);

	pr_debug("mc_saved: sig=0x%x, pf=0x%x, rev=0x%x, total size=0x%x, date = %04x-%02x-%02x\n",
		 sig, pf, rev, total_size, date & 0xffff,
		 date >> 24, (date >> 16) & 0xff);

	rb_meta = (struct ucode_meta *)intel_microcode_find_meta_data(mc, META_TYPE_ROLLBACK);
	if (rb_meta)
		dump_rollback_meta(rb_meta);

	/* Look for ext. headers: */
	if (total_size <= data_size + MC_HEADER_SIZE)
		return;

	ext_header = (void *)ucode + data_size + MC_HEADER_SIZE;
	ext_sigcount = ext_header->count;
	ext_sig = (void *)ext_header + EXT_HEADER_SIZE;

	for (j = 0; j < ext_sigcount; j++) {
		sig = ext_sig->sig;
		pf = ext_sig->pf;

		pr_debug("\tExtended[%d]: sig=0x%x, pf=0x%x\n",
			 j, sig, pf);

		ext_sig++;
	}
#endif
}

static bool load_builtin_intel_microcode(struct cpio_data *cp)
{
	unsigned int eax = 1, ebx, ecx = 0, edx;
	struct firmware fw;
	char name[30];

	if (IS_ENABLED(CONFIG_X86_32))
		return false;

	native_cpuid(&eax, &ebx, &ecx, &edx);

	sprintf(name, "intel-ucode/%02x-%02x-%02x",
		      x86_family(eax), x86_model(eax), x86_stepping(eax));

	if (firmware_request_builtin(&fw, name)) {
		cp->size = fw.size;
		cp->data = (void *)fw.data;
		return true;
	}

	return false;
}

static void print_ucode_info(int old_rev, int new_rev, unsigned int date)
{
	pr_info_once("Early load succeeded, microcode revision: 0x%x -> 0x%x\n", old_rev, new_rev);
}

#ifdef CONFIG_X86_32

static int delay_ucode_info;
static int current_mc_date;
static int early_old_rev;

/*
 * Print early updated ucode info after printk works. This is delayed info dump.
 */
void show_ucode_info_early(void)
{
	struct ucode_cpu_info uci;

	if (delay_ucode_info) {
		intel_cpu_collect_info(&uci);
		print_ucode_info(early_old_rev, uci.cpu_sig.rev, current_mc_date);
		delay_ucode_info = 0;
	}
}

/*
 * At this point, we can not call printk() yet. Delay printing microcode info in
 * show_ucode_info_early() until printk() works.
 */
static void print_ucode(int old_rev, int new_rev, int date)
{
	int *delay_ucode_info_p;
	int *current_mc_date_p;
	int *early_old_rev_p;

	delay_ucode_info_p = (int *)__pa_nodebug(&delay_ucode_info);
	current_mc_date_p = (int *)__pa_nodebug(&current_mc_date);
	early_old_rev_p = (int *)__pa_nodebug(&early_old_rev);

	*delay_ucode_info_p = 1;
	*current_mc_date_p = date;
	*early_old_rev_p = old_rev;
}
#else

static inline void print_ucode(int old_rev, int new_rev, int date)
{
	print_ucode_info(old_rev, new_rev, date);
}
#endif

static int apply_microcode_early(struct ucode_cpu_info *uci, bool early)
{
	struct microcode_intel *mc;
	u32 rev, old_rev;

	mc = uci->mc;
	if (!mc)
		return 0;

	/*
	 * Save us the MSR write below - which is a particular expensive
	 * operation - when the other hyperthread has updated the microcode
	 * already.
	 */
	rev = intel_get_microcode_revision();
	if (rev >= mc->hdr.rev) {
		uci->cpu_sig.rev = rev;
		return UCODE_OK;
	}

	old_rev = rev;

	/* write microcode via MSR 0x79 */
	native_wrmsrl(MSR_IA32_UCODE_WRITE, (unsigned long)mc->bits);

	rev = intel_get_microcode_revision();
	if (rev != mc->hdr.rev)
		return -1;

	uci->cpu_sig.rev = rev;

	if (early)
		print_ucode(old_rev, uci->cpu_sig.rev, mc->hdr.date);
	else
		print_ucode_info(old_rev, uci->cpu_sig.rev, mc->hdr.date);

	return 0;
}

int __init save_microcode_in_initrd_intel(void)
{
	struct ucode_cpu_info uci;
	struct cpio_data cp;

	/*
	 * initrd is going away, clear patch ptr. We will scan the microcode one
	 * last time before jettisoning and save a patch, if found. Then we will
	 * update that pointer too, with a stable patch address to use when
	 * resuming the cores.
	 */
	unapplied_ucode = NULL;

	if (!load_builtin_intel_microcode(&cp))
		cp = find_microcode_in_initrd(ucode_path, false);

	if (!(cp.data && cp.size))
		return 0;

	intel_cpu_collect_info(&uci);

	scan_microcode(cp.data, cp.size, &uci, true);

	show_saved_mc(unapplied_ucode);

	return 0;
}

/*
 * @res_patch, output: a pointer to the patch we found.
 */
static struct ucode_info __load_ucode_intel(struct ucode_cpu_info *uci)
{
	static const char *path;
	struct cpio_data cp;
	bool use_pa;
	struct ucode_info patch = {0};

	if (IS_ENABLED(CONFIG_X86_32)) {
		path	  = (const char *)__pa_nodebug(ucode_path);
		use_pa	  = true;
	} else {
		path	  = ucode_path;
		use_pa	  = false;
	}

	/* try built-in microcode first */
	if (!load_builtin_intel_microcode(&cp))
		cp = find_microcode_in_initrd(path, use_pa);

	if (!(cp.data && cp.size))
		return patch;

	intel_cpu_collect_info(uci);

	return scan_microcode(cp.data, cp.size, uci, false);
}

void __init load_ucode_intel_bsp(void)
{
	struct ucode_info patch;
	struct ucode_cpu_info uci;

	patch = __load_ucode_intel(&uci);
	if (!patch.ucode)
		return;

	uci.mc = patch.ucode;

	apply_microcode_early(&uci, true);
}

static bool early_load_ap_failed;
static bool ucode_committed;
void load_ucode_intel_ap(void)
{
	struct microcode_intel **iup;
	struct ucode_cpu_info uci;
	struct ucode_info patch = {0};
	int ret;

	if (IS_ENABLED(CONFIG_X86_32))
		iup = (struct microcode_intel **)__pa_nodebug(&applied_ucode);
	else
		iup = &applied_ucode;

	/*
	 * Stop scanning and applying microcode after the first failure which
	 * clears the applied_ucode.
	 */
	if (early_load_ap_failed)
		return;

	if (!*iup) {
		patch = __load_ucode_intel(&uci);
		if (!patch.ucode)
			return;
		/*
		 * Copy the patch to kernel memory so that it can be freed
		 * later when we receive newer patch during late loading.
		 */
		save_microcode_patch(&unapplied_ucode, patch.ucode, patch.size);
		*iup = unapplied_ucode;
		clear_ucode_store(&unapplied_ucode);
	}

	uci.mc = *iup;

	ret = apply_microcode_early(&uci, true);

	/*
	 * Even if one cpu fails applying microcode, clear applied_ucode and
	 * cache when late loading succeeds.
	 */
	if (ret < 0 && !early_load_ap_failed) {
		early_load_ap_failed = true;
		free_ucode_store(&applied_ucode);
		free_ucode_store(&committed_ucode);
		return;
	}

	/* Promote from applied to committed ucode after successful apply */
	if (!ucode_committed) {
		committed_ucode = applied_ucode;
		ucode_committed = true;
	}
}

static struct microcode_intel *find_patch(void)
{
	return unapplied_ucode ? unapplied_ucode : applied_ucode;
}

void reload_ucode_intel(void)
{
	struct microcode_intel *p;
	struct ucode_cpu_info uci;

	intel_cpu_collect_info(&uci);

	p = find_patch();
	if (!p)
		return;

	uci.mc = p;

	apply_microcode_early(&uci, false);
}

static void intel_set_control_flags(enum late_load_flags flags)
{
	intel_ucode_control |= flags;
}

static enum late_load_flags intel_get_control_flags(void)
{
	return intel_ucode_control;
}

static void collect_staging_mailbox(int cpu)
{
	struct uc_doe_mbox *mbox;
	u64 mbox_addr;
	int i;

	if (!mcu_cap.staging_supported)
		return;

	i = topology_physical_package_id(cpu);

	/* Avoid multiple mailbox instances per package */
	if (test_and_set_bit(0, &mcu_staging.mboxes[i].creating))
		return;

	if (mcu_staging.mboxes[i].mbox)
		goto out;

	rdmsrl(MSR_MCU_MBOX_ADDR, mbox_addr);
	if (!mbox_addr)
		goto out;

	mbox = uc_doe_create_mbox(mbox_addr);
	if (!mbox)
		goto out;

	mcu_staging.mboxes[i].mbox = mbox;
out:
	clear_bit(0, &mcu_staging.mboxes[i].creating);
}

static int collect_cpu_info(int cpu_num, struct cpu_signature *csig)
{
	struct cpuinfo_x86 *c = &cpu_data(cpu_num);
	unsigned int val[2];

	/* Ensure the thread reads on the same CPU */
	WARN_ON_ONCE(cpu_num != raw_smp_processor_id());

	memset(csig, 0, sizeof(*csig));

	csig->sig = cpuid_eax(0x00000001);

	if ((c->x86_model >= 5) || (c->x86 > 6)) {
		/* get processor flags from MSR 0x17 */
		rdmsr(MSR_IA32_PLATFORM_ID, val[0], val[1]);
		csig->pf = 1 << ((val[1] >> 18) & 7);
	}

	c->microcode = intel_get_microcode_revision();
	csig->rev = c->microcode;

	collect_staging_mailbox(cpu_num);

	return 0;
}

static enum ucode_state apply_microcode_intel(int cpu, enum reload_type type)
{
	struct ucode_cpu_info *uci = ucode_cpu_info + cpu;
	struct cpuinfo_x86 *c = &cpu_data(cpu);
	bool bsp = c->cpu_index == boot_cpu_data.cpu_index;
	struct microcode_intel *mc;
	enum ucode_state ret;
	static int prev_rev;
	u32 rev;

	/* We should bind the task to the CPU */
	if (WARN_ON(raw_smp_processor_id() != cpu))
		return UCODE_ERROR;

	/* Look for a newer patch in our cache: */
	mc = find_patch();
	if (!mc) {
		mc = uci->mc;
		if (!mc)
			return UCODE_NFOUND;
	}

	/*
	 * Save us the MSR write below - which is a particular expensive
	 * operation - when the other hyperthread has updated the microcode
	 * already.
	 */
	rev = intel_get_microcode_revision();
	if (rev >= mc->hdr.rev && type != RELOAD_ROLLBACK && !ucode_load_same) {
		ret = UCODE_OK;
		goto out;
	}

	/* write microcode via MSR 0x79 */
	wrmsrl(MSR_IA32_UCODE_WRITE, (unsigned long)mc->bits);

	rev = intel_get_microcode_revision();

	if (rev != mc->hdr.rev) {
		pr_err("CPU%d update to revision 0x%x failed\n",
		       cpu, mc->hdr.rev);
		return UCODE_ERROR;
	}

	if (bsp && rev != prev_rev) {
		prev_rev = rev;
		save_bsp_svn_info();
	}

	ret = UCODE_UPDATED;

out:
	uci->cpu_sig.rev = rev;

	ret = get_apply_status();
	return ret;
}

static enum ucode_state generic_load_microcode(int cpu, struct iov_iter *iter)
{
	struct ucode_cpu_info *uci = ucode_cpu_info + cpu;
	unsigned int curr_mc_size = 0, new_mc_size = 0;
	enum ucode_state ret = UCODE_OK;
	int new_rev = uci->cpu_sig.rev;
	u8 *new_mc = NULL, *mc = NULL;
	unsigned int csig, cpf;

	while (iov_iter_count(iter)) {
		struct microcode_header_intel mc_header;
		unsigned int mc_size, data_size;
		u8 *data;

		if (!copy_from_iter_full(&mc_header, sizeof(mc_header), iter)) {
			pr_err("error! Truncated or inaccessible header in microcode data file\n");
			break;
		}

		mc_size = get_totalsize(&mc_header);
		if (mc_size < sizeof(mc_header)) {
			pr_err("error! Bad data in microcode data file (totalsize too small)\n");
			break;
		}
		data_size = mc_size - sizeof(mc_header);
		if (data_size > iov_iter_count(iter)) {
			pr_err("error! Bad data in microcode data file (truncated file?)\n");
			break;
		}

		/* For performance reasons, reuse mc area when possible */
		if (!mc || mc_size > curr_mc_size) {
			vfree(mc);
			mc = vmalloc(mc_size);
			if (!mc)
				break;
			curr_mc_size = mc_size;
		}

		memcpy(mc, &mc_header, sizeof(mc_header));
		data = mc + sizeof(mc_header);
		if (!copy_from_iter_full(data, data_size, iter) ||
		    intel_microcode_sanity_check(mc, true, MC_HEADER_TYPE_MICROCODE) < 0 ||
		    __is_lateload_safe(&mc_header)) {
			ret = UCODE_ERROR;
			break;
		}

		csig = uci->cpu_sig.sig;
		cpf = uci->cpu_sig.pf;
		if (ucode_load_same || has_newer_microcode(mc, csig, cpf, new_rev)) {
			vfree(new_mc);
			new_rev = mc_header.rev;
			new_mc  = mc;
			new_mc_size = mc_size;
			mc = NULL;	/* trigger new vmalloc */
			ret = UCODE_NEW;
		}
	}

	vfree(mc);

	if (iov_iter_count(iter) || ret == UCODE_ERROR) {
		vfree(new_mc);
		return UCODE_ERROR;
	}

	if (!new_mc) {
		pr_debug("Newer microcode not found, check your microcode version\n");
		return UCODE_NFOUND;
	}

	vfree(uci->mc);
	uci->mc = (struct microcode_intel *)new_mc;

	save_microcode_patch(&unapplied_ucode, new_mc, new_mc_size);

	pr_debug("CPU%d found a matching microcode update with version 0x%x (current=0x%x)\n",
		 cpu, new_rev, uci->cpu_sig.rev);

	return ret;
}

static void do_staging(struct work_struct *work)
{
	int i, cpu = smp_processor_id();
	struct ucode_cpu_info *uci;
	struct microcode_intel *mc;
	struct uc_doe_mbox *mbox;

	uci = ucode_cpu_info + cpu;
	i = topology_physical_package_id(cpu);

	mc = find_patch();
	if (!mc) {
		mc = uci->mc;
		if (!mc) {
			mcu_staging.mboxes[i].result = UCODE_NFOUND;
			return;
		}
	}

	if (uci->cpu_sig.rev >= mc->hdr.rev && !override_minrev) {
		mcu_staging.mboxes[i].result  = UCODE_OK;
		return;
	}

	mbox = mcu_staging.mboxes[i].mbox;

	/* Need to include the external header */
	if (uc_doe_stage_ucode(mbox, mc, mc->hdr.totalsize)) {
		mcu_staging.mboxes[i].result = UCODE_ERROR;
		return;
	}

	mcu_staging.mboxes[i].result = UCODE_OK;

	pr_debug("Microcode is staged for package %d\n", i);
}

static enum ucode_state perform_staging(void)
{
	enum ucode_state ret = UCODE_OK;
	struct work_struct *work;
	int cpu, i;

	if (!mcu_cap.staging_supported || !ucode_staging)
		return UCODE_OK;

	for (i = 0; i < mcu_staging.mbox_num; i++) {
		mcu_staging.mboxes[i].scheduled = false;
		mcu_staging.mboxes[i].result = UCODE_OK;
	}

	for_each_online_cpu(cpu) {
		i = topology_physical_package_id(cpu);

		if (!mcu_staging.mboxes[i].mbox)
			continue;

		if (mcu_staging.mboxes[i].scheduled)
			continue;

		mcu_staging.mboxes[i].scheduled = true;

		work = &mcu_staging.mboxes[i].work;
		INIT_WORK(work, do_staging);
		schedule_work_on(cpu, work);
	}

	for (i = 0; i < mcu_staging.mbox_num; i++) {
		if (!mcu_staging.mboxes[i].scheduled)
			continue;

		work = &mcu_staging.mboxes[i].work;
		flush_work(work);

		if (ret < mcu_staging.mboxes[i].result)
			ret = mcu_staging.mboxes[i].result;

		mcu_staging.mboxes[i].scheduled = false;
	}

	return ret;
}

static  bool commit_status;
static void do_commit(struct work_struct *work)
{
	union svn_commit commit;

	if (!mcu_cap.rollback_supported)
		return;

	commit.data = 0;
	rdmsrl(MSR_MCU_COMMIT, commit.data);

	/* Nothing to commit */
	if (!commit.commit_svn)
		return;

	commit.data = 0;
	commit.commit_svn = 1;
	wrmsrl(MSR_MCU_COMMIT, commit.data);

	commit.data = 0;
	rdmsrl(MSR_MCU_COMMIT, commit.data);

	if (commit.commit_svn && !commit_status)
		commit_status = 1;
}

static int perform_commit(void)
{
	int ret;

	commit_status = 0;
	ret = schedule_on_each_cpu_locked(do_commit);

	if (!ret && !commit_status) {
		free_ucode_store(&committed_ucode);
		committed_ucode = applied_ucode;
		return ret;
	}

	pr_err("Commit failed: Pending commit\n");
	return -EBUSY;
}

static int pre_apply_intel(enum reload_type type)
{
	int ret;

	switch (type) {
	case RELOAD_COMMIT:
		ret = switch_mcu_commit_config(AUTO_COMMIT);
		if (ret)
			return ret;
		break;
	case RELOAD_NO_COMMIT:
		if (!mcu_cap.rollback_supported)
			return -EINVAL;

		if (!committed_ucode) {
			pr_debug("Defer Commit: no prior microcode, can't continue...\n");
			return -ENOENT;
		}

		ret = switch_mcu_commit_config(MANUAL_COMMIT);
		if (ret)
			return ret;
		break;
	case RELOAD_ROLLBACK:
		if (!mcu_cap.rollback_supported)
			return -EINVAL;

		if (!committed_ucode) {
			pr_debug("Rollback: no prior microcode, can't continue...\n");
			return -ENOENT;
		}

		unapplied_ucode = committed_ucode;
		break;
	default:
		return -EINVAL;
	}

	ret = perform_staging();
	if (ret == UCODE_ERROR || ret == UCODE_NFOUND) {
		pr_err("Error staging microcode\n");
		return -EINVAL;
	}

	return 0;
}

static void post_apply_intel(enum reload_type type, bool apply_state)
{
	switch (type) {
	case RELOAD_COMMIT:
		/*
		 * If apply was successful, then move from unapplied to applied_ucode
		 */
		if (apply_state) {
			free_ucode_store(&applied_ucode);
			applied_ucode = unapplied_ucode;
			committed_ucode = applied_ucode;
			clear_ucode_store(&unapplied_ucode);
		} else {
			free_ucode_store(&unapplied_ucode);
		}
		break;
	case RELOAD_NO_COMMIT:
		if (apply_state) {
			/*
			 * Don't free applied_ucode yet as committed_ucode is
			 * may also be pointing to the same microcode. We can
			 * free it once the microcode is committed.
			 */
			applied_ucode = unapplied_ucode;
			clear_ucode_store(&unapplied_ucode);
		} else {
			free_ucode_store(&unapplied_ucode);
		}
		break;
	case RELOAD_ROLLBACK:
		if (apply_state) {
			free_ucode_store(&applied_ucode);
			applied_ucode = unapplied_ucode;
			clear_ucode_store(&unapplied_ucode);
		}
		break;

	default:
		return;
	}
}

static void release_staging_mailbox(int cpu)
{
	struct uc_doe_mbox *mbox;
	int i, cpus;

	if (!mcu_cap.staging_supported)
		return;

	i = topology_physical_package_id(cpu);
	mbox = mcu_staging.mboxes[i].mbox;
	if (!mbox)
		return;

	/* The number of online CPUs of current package */
	cpus = cpumask_weight_and(topology_core_cpumask(cpu), cpu_online_mask);

	/*
	 * If current CPU is the last online CPU of current package,
	 * release the staging mailbox instance.
	 */
	if (cpus == 1) {
		uc_doe_destroy_mbox(mbox);
		mcu_staging.mboxes[i].mbox = NULL;
	}
}

static void microcode_fini_cpu_intel(int cpu)
{
	release_staging_mailbox(cpu);
}

static bool is_blacklisted(unsigned int cpu)
{
	struct cpuinfo_x86 *c = &cpu_data(cpu);

	/*
	 * Late loading on model 79 with microcode revision less than 0x0b000021
	 * and LLC size per core bigger than 2.5MB may result in a system hang.
	 * This behavior is documented in item BDF90, #334165 (Intel Xeon
	 * Processor E7-8800/4800 v4 Product Family).
	 */
	if (c->x86 == 6 &&
	    c->x86_model == INTEL_FAM6_BROADWELL_X &&
	    c->x86_stepping == 0x01 &&
	    llc_size_per_core > 2621440 &&
	    c->microcode < 0x0b000021) {
		pr_err_once("Erratum BDF90: late loading with revision < 0x0b000021 (0x%x) disabled.\n", c->microcode);
		pr_err_once("Please consider either early loading through initrd/built-in or a potential BIOS update.\n");
		return true;
	}

	return false;
}

static bool check_svn_update(struct ucode_meta *rb_meta)
{
	if (rb_meta->svn_info.mcu_svn < bsp_svn_info.committed_mcu_svn) {
		pr_err("MCU SVN 0x%x less than CPU SVN 0x%x, can't update\n",
		       rb_meta->svn_info.mcu_svn, bsp_svn_info.committed_mcu_svn);
		return false;
	}

	return true;
}

static bool is_ucode_listed(struct ucode_meta *rb_meta)
{
	//int i;
	int cpu = raw_smp_processor_id();
	struct ucode_cpu_info *uci;
	int rev, svn;

	uci = ucode_cpu_info + cpu;
	rev = uci->cpu_sig.rev;
	svn = bsp_svn_info.committed_mcu_svn;

#if 0
	for (i = 0; i < NUM_RB_INFO; i++) {
		if (!rb_meta->rollback_id[i])
			return false;
		if (rb_meta->rollback_id[i] == rev && rb_meta->rollback_svn[i] == svn)
			return true;
	}
#endif

	/*
	 * FIXME: Current ucode doesn't populate rollback_id and rollback_svn and
	 * so always return true. (must be false)
	 */
	return true;
}

static bool can_do_nocommit(struct ucode_meta *rb_meta)
{
	if (!check_svn_update(rb_meta))
		return false;

	if (!is_ucode_listed(rb_meta))
		return false;

	/*
	 * From the spec POV, it is actually OK to manually commit(defer commit) microcode whose
	 * min svn is greater than the current cpu svn. But this will make rollback impossible
	 * as now cpu svn will be set to microcode's min svn and we cannot load mcu with svn lower
	 * than cpu svn.
	 */
	if (rb_meta->svn_info.min_mcu_svn > bsp_svn_info.committed_mcu_svn) {
		pr_err("MCU MIN SVN 0x%x greater than CPU SVN 0x%x, can't update\n",
		       rb_meta->svn_info.min_mcu_svn, bsp_svn_info.committed_mcu_svn);
		return false;
	}

	return true;
}

static bool check_ucode_constraints(enum reload_type type)
{
	struct ucode_meta *rb_meta;
	struct microcode_header_intel *mc_header = (struct microcode_header_intel *)unapplied_ucode;

	if (mc_header->rev < rollback_rev.min_rev_id) {
		pr_err("Microcode Revision 0x%x less than 0x%x (Post BIOS rev)\n",
		       mc_header->rev, rollback_rev.min_rev_id);
		return false;
	}

	if (check_pending())
		return false;

	/*
	 * If platform supports architectural rollback, microcode is
	 * expected to have rollback meta data.
	 */
	rb_meta = (struct ucode_meta *)intel_microcode_find_meta_data((void *)unapplied_ucode,
								      META_TYPE_ROLLBACK);
	if (!rb_meta)
		return false;

	if (type == RELOAD_NO_COMMIT && !can_do_nocommit(rb_meta))
		return false;

	if (type == RELOAD_COMMIT && !check_svn_update(rb_meta))
		return false;

	return true;
}

static enum ucode_state request_microcode_fw(int cpu, struct device *device, enum reload_type type)
{
	struct cpuinfo_x86 *c = &cpu_data(cpu);
	const struct firmware *firmware;
	struct iov_iter iter;
	enum ucode_state ret;
	struct kvec kvec;
	char name[30];

	if (is_blacklisted(cpu))
		return UCODE_NFOUND;

	sprintf(name, "intel-ucode/%02x-%02x-%02x",
		c->x86, c->x86_model, c->x86_stepping);

	/*
	 * This allows fetching a production ucode that might not be committed by OS,
	 * but loaded from BIOS directly from the built-in firmware path or initrd.
	 * This may be needed in case we need to rollback.
	 */
	if (type == RELOAD_NO_COMMIT) {
		if (!committed_ucode) {
			pr_err("No original ucode found!\n");
			pr_err("Ensure original ucode is either part of initrd or built-in fw path\n");
			return UCODE_ERROR;
		}
		sprintf(name, "intel-ucode/stage/%02x-%02x-%02x",
			c->x86, c->x86_model, c->x86_stepping);
	}

	if (request_firmware_direct(&firmware, name, device)) {
		pr_debug("data file %s load failed\n", name);
		return UCODE_NFOUND;
	}

	kvec.iov_base = (void *)firmware->data;
	kvec.iov_len = firmware->size;
	iov_iter_kvec(&iter, ITER_SOURCE, &kvec, 1, firmware->size);
	ret = generic_load_microcode(cpu, &iter);

	if (ret == UCODE_NEW && mcu_cap.rollback_supported && !check_ucode_constraints(type))
		ret = UCODE_ERROR;

	release_firmware(firmware);
	if (ret == UCODE_ERROR)
		free_ucode_store(&unapplied_ucode);

	return ret;
}

static bool is_rollback_supported(void)
{
	return mcu_cap.rollback_supported;
}

static struct microcode_ops microcode_intel_ops = {
	.get_control_flags                = intel_get_control_flags,
	.get_load_scope                   = get_load_scope,
	.check_pending_commits            = check_pending_commits,
	.perform_commit                   = perform_commit,
	.is_rollback_supported            = is_rollback_supported,
	.request_microcode_fw             = request_microcode_fw,
	.collect_cpu_info                 = collect_cpu_info,
	.apply_microcode                  = apply_microcode_intel,
	.get_current_rev                  = intel_get_microcode_revision,
	.pre_apply                        = pre_apply_intel,
	.post_apply                       = post_apply_intel,
	.microcode_fini_cpu               = microcode_fini_cpu_intel,
};

static int __init calc_llc_size_per_core(struct cpuinfo_x86 *c)
{
	u64 llc_size = c->x86_cache_size * 1024ULL;

	do_div(llc_size, c->x86_max_cores);

	return (int)llc_size;
}

static void setup_mcu_enumeration(void)
{
	u64 arch_cap;
	union mcu_status status;

	arch_cap = x86_read_arch_cap_msr();
	if (!(arch_cap & ARCH_CAP_MCU_ENUM))
		return;

	rdmsrl(MSR_MCU_ENUM, mcu_cap.data);

	if (mcu_cap.uniform_available) {
		pr_info_once("Microcode Uniform Update Capability detected\n");

		status.data = 0;
		rdmsrl(MSR_MCU_STATUS, status.data);
		if (!status.post_bios_mcu)
			pr_warn("WARNING: Post bios update not successful! Contact BIOS Vendor.\n");
	}

	if (mcu_cap.staging_supported) {
		pr_info_once("Microcode Staging Capability detected\n");

		mcu_staging.mbox_num = topology_max_packages();
		mcu_staging.mboxes   = kcalloc(mcu_staging.mbox_num,
					       sizeof(*mcu_staging.mboxes),
					       GFP_KERNEL);

		if (!dentry_ucode)
			dentry_ucode = debugfs_create_dir("microcode", NULL);

		debugfs_create_bool("ucode_staging", 0644, dentry_ucode,
				    &ucode_staging);
	}

	if (mcu_cap.rollback_supported) {
		pr_info_once("Microcode Rollback Capability detected\n");

		/* Store the minimum mcu revision for rollback */
		rollback_rev.data = 0;
		rdmsrl(MSR_MCU_ROLLBACK_MIN_ID, rollback_rev.data);

		save_bsp_svn_info();
	}
}

struct microcode_ops * __init init_intel_microcode(void)
{
	struct cpuinfo_x86 *c = &boot_cpu_data;

	if (c->x86_vendor != X86_VENDOR_INTEL || c->x86 < 6 ||
	    cpu_has(c, X86_FEATURE_IA64)) {
		pr_err("Intel CPU family 0x%x not supported\n", c->x86);
		return NULL;
	}

	llc_size_per_core = calc_llc_size_per_core(c);
	intel_set_control_flags(LATE_LOAD_SAFE);

	setup_mcu_enumeration();

	return &microcode_intel_ops;
}
