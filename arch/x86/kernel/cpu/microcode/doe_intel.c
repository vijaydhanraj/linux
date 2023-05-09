// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Intel Microcode Data Object Exchange Mailbox Driver for Linux
 *
 * Copyright (C) 2023 Intel Corporation
 *	Qiuxu Zhuo <qiuxu.zhuo@intel.com>
 *	Ashok Raj <ashok.raj@intel.com>
 */

/* TODO: Remove DEBUG when this file goes to upstream */
#define DEBUG
#define pr_fmt(fmt) "microcode doe: " fmt

#include <linux/io.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/bitfield.h>
#include <linux/pci_ids.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <vdso/time64.h>
#include "doe.h"

/* DOE registers' offsets */
#define DOE_REG_CTRL		0x00
#define DOE_REG_STATUS		0x04
#define DOE_REG_WRITE		0x08
#define DOE_REG_READ		0x0c
#define DOE_REG_NUM		4
#define DOE_REG_SIZE		sizeof(u32)

/**
 * union doe_ctrl - DOE control register
 *
 * @abort      : When set, DOE aborts all data object processing.
 * @rsvd       : Reserved bits.
 * @enabled    : When set, the DOE mailbox is enabled.
 * @data_ready : When set, a dword available in read/write register to be read out.
 * @go         : When set, DOE starts processing the DOE object.
 */
union doe_ctrl {
	u32 data;
	struct {
		u32 abort:1;
		u32 rsvd:28;
		u32 enabled:1;
		u32 data_ready:1;
		u32 go:1;
	};
};

/**
 * union doe_status - DOE status register
 *
 * @busy         : When set, DOE is unable to receive new data via write register.
 * @rsvd1        : Reserved bits
 * @error        : When set, an error on handling the last DOE object.
 * @rsvd2        : Reserved bits
 * @object_ready : When set, an object is available as response.
 * @rsvd3        : Reserved bits
 */
union doe_status {
	u32 data;
	struct {
		u32 busy:1;
		u32 rsvd1:1;
		u32 error:1;
		u32 rsvd2:27;
		u32 object_ready:1;
		u32 resv3:1;
	};
};

#define DOE_DATA_OBJ_MAX_LEN	0x0003ffff
#define DOE_DATA_OBJ_TYPE	0x0b
#define DOE_DATA_OBJ_HDR_LEN	8
#define DOE_DATA_OBJ_CMD_LEN	8

/**
 * union doe_data_obj_hdr1 - DOE data object header1
 *
 * @vid  : Vendor ID
 * @type : DOE data object type
 * @rsvd : Reserved bits
 */
union doe_data_obj_hdr1 {
	u32 data;
	struct {
		u32 vid:16;
		u32 type:8;
		u32 rsvd:8;
	};
};

/**
 * union doe_data_obj_hdr2 - DOE Data object header2
 *
 * @len  : Length of the whole DOE object (include DOE header) in dword
 * @rsvd : Reserved bits
 */
union doe_data_obj_hdr2 {
	u32 data;
	struct {
		u32 len:18;
		u32 rsvd:14;
	};
};

/**
 * struct uc_doe_mbox - DOE mailbox instance
 *
 * @base     : DOE MMIO registers base address
 * @page_len : Ucode page length in byte per DOE transaction
 */
struct uc_doe_mbox {
	void __iomem *base;
	size_t page_len;
};

/**
 * struct uc_doe_transaction - Represent a pair of DOE request/response
 *
 * @cmd     : DOE command, sub-command and parameters.
 * @req     : DOE request payload
 * @req_len : Length of the DOE request payload in byte
 * @rsp     : DOE response payload
 * @rsp_len : Length of the DOE response payload in byte
 */
struct uc_doe_transaction {
	u32 cmd[DOE_DATA_OBJ_CMD_LEN / sizeof(u32)];
	u32 *req;
	size_t req_len;
	u32 *rsp;
	size_t rsp_len;
};

enum uc_doe_cmd {
	DOE_CMD_DISCOVER		= 0x00,
	DOE_CMD_GET_STAGED_PATCH_ID	= 0x01,
	DOE_CMD_STAGE_AND_VERIFY	= 0x03,
	DOE_CMD_RESET_STAGING		= 0x03 | (0x01 << 8),
};

/**
 * struct uc_doe_rsp_discovery - DOE response for command DOE_CMD_DISCOVER
 *
 * @revision : DOE mailbox revision
 * @mbox_len : DOE mailbox length in byte
 * @page_len : Ucode page length in byte
 */
struct uc_doe_rsp_discovery {
	u16 revision;
	u16 reserved;
	u32 mbox_len;
	u32 page_len;
};

/**
 * struct uc_doe_rsp - DOE response for commands:
 *                     DOE_CMD_GET_STAGED_PATCH_ID
 *                     DOE_CMD_STAGE_AND_VERIFY
 *                     DOE_CMD_RESET_STAGING
 *
 * @patch_id         : The revision of the staged ucode. This field is valid for
 *                     the response of DOE_CMD_GET_STAGED_PATCH_ID.
 * @next_page_offset : Offset of next ucode page to be staged. This filed is valid for
 *                     the response of DOE_CMD_STAGE_AND_VERIFY or DOE_CMD_RESET_STAGING.
 * @completed        : The whole ucode staging has completed.
 * @in_process       : The whole ucode staging is in progress.
 * @failed           : The whole ucode staging is failed.
 */
struct uc_doe_rsp {
	union {
		u32 patch_id;
		u32 next_page_offset;
	} result;
	union {
		u32 data;
		struct {
			u32 completed:1;
			u32 in_progress:1;
			u32 failed:1;
			u32 rsvd:29;
		};
	} flags;
	union {
		u32 data1;
		struct {
			u32 mcu_svn:16;
			u32 rsvd1:16;
		};
	} svn_info;
};

/*
 * Polling interval in millisecond
 */
#define DOE_POLL_INTERVAL		2
#define DOE_POLL_NUM			512

static DEFINE_MUTEX(doe_mutex);

static bool doe_enabled(struct uc_doe_mbox *mbox)
{
	union doe_ctrl ctrl;

	ctrl.data = readl(mbox->base + DOE_REG_CTRL);

	return !!ctrl.enabled;
}

static inline void doe_abort(struct uc_doe_mbox *mbox)
{
	union doe_ctrl ctrl;

	ctrl.data  = 0;
	ctrl.abort = 1;
	writel(ctrl.data, mbox->base + DOE_REG_CTRL);
}

static inline void doe_go(struct uc_doe_mbox *mbox)
{
	union doe_ctrl ctrl;

	/*
	 * Notify DOE that the DOE object is available
	 */
	ctrl.data = 0;
	ctrl.go   = 1;
	writel(ctrl.data, mbox->base + DOE_REG_CTRL);
}

static int doe_reset(struct uc_doe_mbox *mbox)
{
	union doe_status status;
	int i;

	pr_debug("Reset DOE mailbox\n");

	doe_abort(mbox);

	i = DOE_POLL_NUM;
	do {
		msleep(DOE_POLL_INTERVAL);

		status.data = readl(mbox->base + DOE_REG_STATUS);
		if (!status.busy && !status.error)
			return 0;
	} while (--i);

	return -ETIMEDOUT;
}

static int doe_wait_for_dword(struct uc_doe_mbox *mbox, int set)
{
	union doe_ctrl ctrl;
	int i = DOE_POLL_NUM;

	do {
		ctrl.data = readl(mbox->base + DOE_REG_CTRL);
		if (ctrl.data_ready == set)
			return 0;

		msleep(DOE_POLL_INTERVAL);
	} while (--i);

	pr_err("Timeout to wait for dword %s\n", set ? "ready" : "consumed");

	return -ETIMEDOUT;
}

static inline int doe_wait_for_dword_consumed(struct uc_doe_mbox *mbox)
{
	return doe_wait_for_dword(mbox, 0);
}

static inline int doe_wait_for_dword_ready(struct uc_doe_mbox *mbox)
{
	return doe_wait_for_dword(mbox, 1);
}

static int doe_write_dword(struct uc_doe_mbox *mbox, u32 val)
{
	union doe_ctrl ctrl;
	int rc;

	/*
	 * Wait for DOE to consume the last dword before
	 * writing a new dword.
	 */
	rc = doe_wait_for_dword_consumed(mbox);
	if (rc)
		return rc;

	writel(val, mbox->base + DOE_REG_WRITE);

	/*
	 * Notify DOE that the dword is available.
	 */
	ctrl.data	= 0;
	ctrl.data_ready = 1;
	writel(ctrl.data, mbox->base + DOE_REG_CTRL);

	return 0;
}

static int doe_read_dword(struct uc_doe_mbox *mbox, u32 *val)
{
	union doe_ctrl ctrl;
	int rc;

	/*
	 * Wait for DOE to get the dword ready before
	 * reading it.
	 */
	rc = doe_wait_for_dword_ready(mbox);
	if (rc)
		return rc;

	*val = readl(mbox->base + DOE_REG_READ);

	/*
	 * Notify DOE that the dword has been read out.
	 */
	ctrl.data	= 0;
	ctrl.data_ready = 0;
	writel(ctrl.data, mbox->base + DOE_REG_CTRL);

	return 0;
}

static inline int doe_write_obj_hdr(struct uc_doe_mbox *mbox, size_t len)
{
	union doe_data_obj_hdr1 hdr1;
	union doe_data_obj_hdr2 hdr2;
	int rc;

	hdr1.data = 0;
	hdr1.vid  = PCI_VENDOR_ID_INTEL;
	hdr1.type = DOE_DATA_OBJ_TYPE;
	rc = doe_write_dword(mbox, hdr1.data);
	if (rc)
		return rc;

	hdr2.data = 0;
	hdr2.len  = len;
	rc = doe_write_dword(mbox, hdr2.data);
	return rc;
}

static inline int doe_write_obj_cmd(struct uc_doe_mbox *mbox, struct uc_doe_transaction *t)
{
	int rc, i, n;

	n = sizeof(t->cmd) / sizeof(u32);
	for (i = 0; i < n; i++) {
		rc = doe_write_dword(mbox, t->cmd[i]);
		if (rc)
			return rc;
	}

	return 0;
}

static inline int doe_write_obj_data(struct uc_doe_mbox *mbox, struct uc_doe_transaction *t)
{
	int rc, i, n;

	n = t->req_len / sizeof(u32);
	for (i = 0; i < n; i++) {
		rc = doe_write_dword(mbox, t->req[i]);
		if (rc)
			return rc;
	}

	return 0;
}

static int doe_read_obj_hdr(struct uc_doe_mbox *mbox, size_t *len)
{
	union doe_data_obj_hdr1 hdr1;
	union doe_data_obj_hdr2 hdr2;
	int rc;

	rc = doe_read_dword(mbox, &hdr1.data);
	if (rc)
		return rc;

	if (hdr1.vid != PCI_VENDOR_ID_INTEL ||
	    hdr1.type != DOE_DATA_OBJ_TYPE) {
		pr_err("Invalid DOE header1 0x%x\n", hdr1.data);
		return -EINVAL;
	}

	rc = doe_read_dword(mbox, &hdr2.data);
	if (rc)
		return rc;

	if (hdr2.len < DOE_DATA_OBJ_HDR_LEN / sizeof(u32)) {
		pr_err("Invalid DOE header2 0x%x (length too small)\n", hdr2.data);
		return -EINVAL;
	}

	*len = hdr2.len;

	return 0;
}

static int doe_read_obj_data(struct uc_doe_mbox *mbox,
			     struct uc_doe_transaction *t,
			     size_t rd_len)
{
	size_t payload_len;
	int rc, i;
	u32 val;

	/*
	 * The DOE header has been read out
	 */
	rd_len -= DOE_DATA_OBJ_HDR_LEN / sizeof(u32);

	/*
	 * Read the response payload
	 */
	payload_len = min(rd_len, t->rsp_len / sizeof(u32));
	for (i = 0; i < payload_len; i++) {
		rc = doe_read_dword(mbox, &t->rsp[i]);
		if (rc)
			return rc;
	}

	/*
	 * Drain the excess response payload
	 */
	if (i < rd_len)
		pr_warn("Drain %zu dwords of excess response for cmd %u", rd_len - i, t->cmd[0]);

	for (; i < rd_len; i++) {
		rc = doe_read_dword(mbox, &val);
		if (rc)
			return rc;
	}

	return 0;
}

/*
 * 1: Response object is ready, 0: No response object is expected, < 0 on error
 */
static int doe_wait_for_response_ready(struct uc_doe_mbox *mbox, struct uc_doe_transaction *t)
{
	union doe_status status;
	int i = DOE_POLL_NUM;

	do {
		status.data = readl(mbox->base + DOE_REG_STATUS);

		if (status.error) {
			pr_err("DOE error when waiting for response ready\n");
			return -EIO;
		}

		/*
		 * If no response is expected, check whether DOE is busy
		 */
		if (!t->rsp_len && !status.busy) {
			if (status.object_ready) {
				pr_warn("No response expected but response is ready\n");
				break;
			}

			return 0;
		}

		/*
		 * If a response is expected, check whether DOE object is ready
		 */
		if (t->rsp_len && !status.busy && status.object_ready)
			break;

		msleep(DOE_POLL_INTERVAL);
	} while (--i);

	if (!i) {
		pr_err("Timeout to wait for response ready\n");
		return -ETIMEDOUT;
	}

	return 1;
}

static int doe_send_req(struct uc_doe_mbox *mbox, struct uc_doe_transaction *t)
{
	union doe_status status;
	size_t len;
	u32 cmd;
	int rc;

	cmd = t->cmd[0];

	len = (DOE_DATA_OBJ_HDR_LEN + sizeof(t->cmd) + t->req_len) / sizeof(u32);
	if (len > DOE_DATA_OBJ_MAX_LEN) {
		pr_err("Invalid DOE length (0x%zx exceeds max 0x%x) for cmd 0x%x\n",
		       len, DOE_DATA_OBJ_MAX_LEN, cmd);
		return -EINVAL;
	}

	status.data = readl(mbox->base + DOE_REG_STATUS);
	if (status.error) {
		pr_err("DOE error before sending request for cmd 0x%x\n", cmd);
		return -EIO;
	}
	if (status.busy) {
		pr_err("DOE busy before sending request for cmd 0x%x\n", cmd);
		return -EBUSY;
	}

	rc = doe_write_obj_hdr(mbox, len);
	if (rc) {
		pr_err("Failed to write DOE object header for cmd 0x%x\n", cmd);
		return rc;
	}

	rc = doe_write_obj_cmd(mbox, t);
	if (rc) {
		pr_err("Failed to write DOE cmd 0x%x\n", cmd);
		return rc;
	}

	rc = doe_write_obj_data(mbox, t);
	if (rc) {
		pr_err("Failed to write DOE object data for cmd 0x%x\n", cmd);
		return rc;
	}

	/*
	 * Before making a go, make sure DOE has consumed the last dword.
	 */
	rc = doe_wait_for_dword_consumed(mbox);
	if (rc)
		return rc;

	doe_go(mbox);

	return 0;
}

static int doe_recv_rsp(struct uc_doe_mbox *mbox, struct uc_doe_transaction *t)
{
	union doe_status status;
	size_t rd_len;
	u32 cmd;
	int rc;

	cmd = t->cmd[0];

	rc = doe_read_obj_hdr(mbox, &rd_len);
	if (rc) {
		pr_err("Failed to read DOE object header for cmd 0x%x\n", cmd);
		return rc;
	}

	rc = doe_read_obj_data(mbox, t, rd_len);
	if (rc) {
		pr_err("Failed to read DOE object data for cmd 0x%x\n", cmd);
		return rc;
	}

	status.data = readl(mbox->base + DOE_REG_STATUS);
	if (status.error) {
		pr_err("DOE error after receiving response of cmd 0x%x\n", cmd);
		return -EIO;
	}

	return 0;
}

static int doe_handle_transaction(struct uc_doe_mbox *mbox, struct uc_doe_transaction *t)
{
	int rc;

	mutex_lock(&doe_mutex);

	rc = doe_send_req(mbox, t);
	if (rc)
		goto out;

	rc = doe_wait_for_response_ready(mbox, t);
	if (rc <= 0)
		goto out;

	rc = doe_recv_rsp(mbox, t);
out:
	mutex_unlock(&doe_mutex);

	return rc;
}

#define DECLARE_DOE_TRANS(t, command, request, request_len,	\
			     response, response_len)		\
struct uc_doe_transaction t = {					\
	.cmd		= { command },				\
	.req		= request,				\
	.req_len	= request_len,				\
	.rsp		= (u32 *)response,			\
	.rsp_len	= response_len				\
}

static int uc_doe_cmd_discover(struct uc_doe_mbox *mbox)
{
#define PAYLOAD_MIN_LEN	4
	struct uc_doe_rsp_discovery rsp;
	int rc;
	DECLARE_DOE_TRANS(t, DOE_CMD_DISCOVER, NULL, 0, &rsp, sizeof(rsp));

	rc = doe_handle_transaction(mbox, &t);
	if (rc)
		return rc;

	if (rsp.mbox_len & (sizeof(u32) - 1)) {
		pr_err("Invalid DOE mailbox length 0x%x (non-dword aligned)\n", rsp.mbox_len);
		return -EINVAL;
	}

	if (rsp.page_len & (sizeof(u32) - 1)) {
		pr_err("Invalid ucode page length 0x%x (non-dword aligned)\n", rsp.page_len);
		return -EINVAL;
	}

	if (rsp.page_len < PAYLOAD_MIN_LEN) {
		pr_err("Invalid ucode page length 0x%x (too small)\n", rsp.page_len);
		return -EINVAL;
	}

	if (rsp.mbox_len < DOE_DATA_OBJ_HDR_LEN + DOE_DATA_OBJ_CMD_LEN + rsp.page_len) {
		pr_err("Invalid DOE mailbox length 0x%x (too small)\n", rsp.mbox_len);
		return -EINVAL;
	}

	pr_info_once("DOE mailbox rev 0x%x, mailbox len 0x%x, ucode page len 0x%x\n",
		     rsp.revision, rsp.mbox_len, rsp.page_len);

	mbox->page_len = rsp.page_len;

	return 0;
}

static int uc_doe_cmd_get_patch_id(struct uc_doe_mbox *mbox, struct uc_doe_rsp *rsp)
{
	int rc;
	DECLARE_DOE_TRANS(t, DOE_CMD_GET_STAGED_PATCH_ID, NULL, 0, rsp, sizeof(*rsp));

	rc = doe_handle_transaction(mbox, &t);
	if (rc) {
		pr_err("Failed to get staged patch ID: %d\n", rc);
		return rc;
	}

	return 0;
}

static int uc_doe_cmd_stage_and_verify(struct uc_doe_mbox *mbox, u32 *req,
				       size_t req_len, struct uc_doe_rsp *rsp)
{
	int rc;
	DECLARE_DOE_TRANS(t, DOE_CMD_STAGE_AND_VERIFY, req, req_len, rsp, sizeof(*rsp));

	rc = doe_handle_transaction(mbox, &t);
	if (rc) {
		pr_err("Failed to stage and verify ucode: %d\n", rc);
		return rc;
	}

	return 0;
}

static int uc_doe_cmd_reset_staging(struct uc_doe_mbox *mbox, struct uc_doe_rsp *rsp)
{
	int rc;
	DECLARE_DOE_TRANS(t, DOE_CMD_RESET_STAGING, NULL, 0, rsp, sizeof(*rsp));

	rc = doe_handle_transaction(mbox, &t);
	if (rc) {
		pr_err("Failed to reset ucode staging: %d\n", rc);
		return rc;
	}

	return 0;
}

static int uc_doe_check_staged_ucode(struct uc_doe_mbox *mbox)
{
	struct uc_doe_rsp rsp;
	int rc;

	rc = uc_doe_cmd_get_patch_id(mbox, &rsp);
	if (rc) {
		pr_err("Failed to get staged ucode status\n");
		return rc;
	}

	if (!rsp.flags.completed) {
		pr_err("Invalid staged ucode status\n");
		return -EIO;
	}

	pr_debug("Staged ucode revision 0x%x svn 0x%x\n", rsp.result.patch_id,
		 rsp.svn_info.mcu_svn);

	return 0;
}

/**
 * uc_doe_destroy_mbox() - Destroy a DOE mailbox instance.
 *
 * @mbox: DOE mailbox instance to be destroyed.
 */
void uc_doe_destroy_mbox(struct uc_doe_mbox *mbox)
{
	if (mbox && mbox->base)
		iounmap(mbox->base);

	kfree(mbox);
}

/**
 * uc_doe_create_mbox() - Create a DOE mailbox instance.
 *
 * @doe_addr : MMIO-backed DOE mailbox address
 *
 * RETURNS   : Created DOE mailbox instance, NULL on failure.
 */
struct uc_doe_mbox *uc_doe_create_mbox(phys_addr_t doe_addr)
{
	struct uc_doe_mbox *mbox;
	int rc;

	if (!doe_addr)
		return NULL;

	mbox = kzalloc(sizeof(*mbox), GFP_KERNEL);
	if (!mbox)
		return NULL;

	mbox->base = ioremap(doe_addr, DOE_REG_NUM * DOE_REG_SIZE);
	if (!mbox->base) {
		pr_err("Failed to ioremap for DOE mailbox %pap\n", &doe_addr);
		goto failed;
	}

	if (!doe_enabled(mbox)) {
		pr_debug("DOE mailbox is disabled\n");
		goto failed;
	}

	rc = doe_reset(mbox);
	if (rc) {
		pr_err("Timeout to reset DOE mailbox\n");
		goto failed;
	}

	rc = uc_doe_cmd_discover(mbox);
	if (rc) {
		pr_err("Failed to discover DOE mailbox\n");
		goto failed;
	}

	return mbox;
failed:
	uc_doe_destroy_mbox(mbox);
	return NULL;
}

/**
 * uc_doe_stage_ucode() - Stage the ucode into CPU internal buffer.
 *
 * @mbox   : DOE mailbox instance
 * @uc     : Ucode to be staged
 * @uc_len : Length of the ucode to be staged
 *
 * RETURNS : 0 if the ucode is staged successfully, -ERRNO on error.
 */
int uc_doe_stage_ucode(struct uc_doe_mbox *mbox, void *uc, size_t uc_len)
{
	struct uc_doe_rsp rsp;
	u32 offset, *req;
	size_t req_len;
	int rc;

	if (uc_len & (sizeof(u32) - 1)) {
		pr_err("Invalid ucode length 0x%zx (non-dword aligned)\n", uc_len);
		return -EINVAL;
	}

	rc = doe_reset(mbox);
	if (rc) {
		pr_err("Timeout to reset DOE mailbox\n");
		return rc;
	}

	rc = uc_doe_cmd_reset_staging(mbox, &rsp);
	if (rc) {
		pr_err("Failed to reset ucode staging\n");
		return rc;
	}

	offset = rsp.result.next_page_offset;
	if (offset & (sizeof(u32) - 1)) {
		pr_err("Invalid ucode initial offset 0x%x (non-dword aligned)\n", offset);
		return -EINVAL;
	}

	if (offset >= uc_len) {
		pr_err("Invalid ucode initial offset 0x%x (exceeds ucode size)\n", offset);
		return -EINVAL;
	}

	pr_debug("Ucode initial offset 0x%x\n", offset);

	/* Stage ucode */
	req_len = min(mbox->page_len, uc_len - offset);
	while (req_len > 0) {
		req = uc + offset;

		pr_debug("Ucode staging at offset 0x%x, length 0x%zx\n", offset, req_len);

		rc = uc_doe_cmd_stage_and_verify(mbox, req, req_len, &rsp);
		if (rc) {
			pr_err("Failed to stage ucode during staging\n");
			return rc;
		}

		if (rsp.flags.failed) {
			pr_err("Failed to stage ucode (reported by CPU)\n");
			return -EIO;
		}

		if (rsp.flags.completed) {
			rc = uc_doe_check_staged_ucode(mbox);
			if (rc)
				return rc;

			pr_debug("Ucode staging completed successfully\n");
			return 0;
		}

		if (!rsp.flags.in_progress) {
			pr_err("Unknown ucode staging status (expected in progress)\n");
			return -EIO;
		}

		offset = rsp.result.next_page_offset;

		if (offset >= uc_len) {
			pr_err("Invalid ucode next offset 0x%x (exceeds ucode size)\n", offset);
			return -EINVAL;
		}

		req_len = min(mbox->page_len, uc_len - offset);
	}

	/*
	 * Should not reach here
	 */
	return -EIO;
}
