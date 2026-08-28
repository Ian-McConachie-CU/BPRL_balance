/*
 * ffconf.h — FatFS configuration for BPRL flight logging.
 * Keep it minimal: sequential binary writes, no LFN, no timestamps.
 * To add a new option: copy the relevant define from ext/fatfs/source/_ffconf.h.
 */

#pragma once

#define FFCONF_DEF          86631   /* Revision ID — must match ff.c */

/* ── Function switches ──────────────────────────────────────────────────── */
#define FF_FS_READONLY      0       /* Read/write */
#define FF_FS_MINIMIZE      0       /* Full function set */
#define FF_USE_FIND         0
#define FF_USE_MKFS         0       /* No f_mkfs — card pre-formatted FAT32 */
#define FF_USE_FASTSEEK     0
#define FF_USE_EXPAND       0
#define FF_USE_CHMOD        0
#define FF_USE_LABEL        0
#define FF_USE_FORWARD      0
#define FF_USE_STRFUNC      0
#define FF_PRINT_LLI        0
#define FF_PRINT_FLOAT      0
#define FF_STRF_ENCODE      0

/* ── Locale ─────────────────────────────────────────────────────────────── */
#define FF_CODE_PAGE        437     /* US English — smallest code page table */
#define FF_USE_LFN          0       /* 8.3 names only (LOG0001.BIN is fine) */
#define FF_MAX_LFN          12
#define FF_LFN_UNICODE      0
#define FF_LFN_BUF          12
#define FF_SFN_BUF          12
#define FF_FS_RPATH         0

/* ── Drive / volume ─────────────────────────────────────────────────────── */
#define FF_VOLUMES          1
#define FF_STR_VOLUME_ID    0
#define FF_VOLUME_STRS      "SD"
#define FF_MULTI_PARTITION  0
#define FF_MIN_SS           512
#define FF_MAX_SS           512
#define FF_LBA64            0
#define FF_MIN_GPT          0x10000000
#define FF_USE_TRIM         0

/* ── System ─────────────────────────────────────────────────────────────── */
#define FF_FS_TINY          0       /* Private sector buffer per FIL object */
#define FF_FS_EXFAT         0
#define FF_FS_NORTC         1       /* No RTC — use fixed timestamp below */
#define FF_NORTC_MON        1
#define FF_NORTC_MDAY       1
#define FF_NORTC_YEAR       2024
#define FF_FS_NOFSINFO      0
#define FF_FS_LOCK          0
#define FF_FS_REENTRANT     1       /* USBCmdThread + LogThread both use FatFS */
#define FF_FS_TIMEOUT       1000
#define FF_SYNC_t           void *          /* opaque handle; cast to semaphore_t * in fatfs_syscall.c */
