/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef __X86_MCU_DOE_H__
#define __X86_MCU_DOE_H__

#include <linux/types.h>

struct uc_doe_mbox;

#ifdef CONFIG_MICROCODE_LATE_LOADING
struct uc_doe_mbox *uc_doe_create_mbox(phys_addr_t doe_addr);
void uc_doe_destroy_mbox(struct uc_doe_mbox *mbox);
int uc_doe_stage_ucode(struct uc_doe_mbox *mbox, void *uc, size_t uc_len);
#else
static inline struct uc_doe_mbox *uc_doe_create_mbox(phys_addr_t doe_addr) { return NULL; }
static inline void uc_doe_destroy_mbox(struct uc_doe_mbox *mbox) { }
static inline int uc_doe_stage_ucode(struct uc_doe_mbox *mbox, void *uc, size_t uc_len)
{
	return -ENODEV;
}
#endif

#endif /* __X86_MCU_DOE_H__ */
