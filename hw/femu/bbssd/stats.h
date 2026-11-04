#ifndef __FEMU_BB_STATS_H
#define __FEMU_BB_STATS_H

#include <stdbool.h>
#include <stdint.h>

struct FemuCtrl;

/*
 * Per-workload FTL statistics for blackbox mode.
 *
 * Only the FTL thread touches this, so there is no locking. A workload starts
 * at the first host read/write and ends once no I/O has arrived for idle_ns
 * (default 60s, FEMU_STATS_IDLE_SEC). While it runs, a one-line summary of
 * each interval is printed (default every 5s, FEMU_STATS_INTERVAL_SEC, 0 = off).
 * At the end, the full report is printed and the counters are reset.
 */

/*
 * Log-linear latency histogram: each power of two is split into
 * 2^BB_LAT_SUB_BITS buckets, so percentiles are within ~12.5%.
 */
#define BB_LAT_SUB_BITS 3
#define BB_LAT_BUCKETS  512

struct bb_lat_hist {
    uint64_t b[BB_LAT_BUCKETS];
};

/* Host command accounting for one direction (read or write). */
struct bb_io_acc {
    uint64_t cmds, bytes;
    uint64_t lat_sum, lat_max;   /* emulated device latency, ns */
    uint64_t queue_sum;          /* part spent waiting for a busy LUN, ns */
    struct bb_lat_hist hist;
};

/* Whole-workload counters, zeroed at the start of each workload. */
struct bb_stats_ctr {
    bool     active;         /* a workload is in progress */
    uint64_t start_ns;       /* first I/O of the workload */
    uint64_t last_io_ns;     /* most recent I/O */
    uint32_t idle_polls;     /* throttles clock reads while idle */

    struct bb_io_acc rd, wr;
    uint64_t flash_rd_user;  /* flash page reads serving host reads */
    uint64_t flash_rd_gc;    /* flash page reads done by GC */

    /* poller completion counters when the workload started */
    int64_t  base_tt_ios, base_late_ios;
};

/* Counters for the current periodic-log interval. */
struct bb_stats_iv {
    uint64_t start_ns;
    struct bb_io_acc rd, wr;
    uint64_t base_flash_rd;  /* flash_rd_user + flash_rd_gc at interval start */
    int64_t  base_tt_ios, base_late_ios;
};

struct bb_stats {
    /* fixed at init */
    struct FemuCtrl *n;
    uint64_t idle_ns;        /* idle time that ends a workload */
    uint64_t interval_ns;    /* periodic log interval, 0 = off */
    uint32_t page_bytes;     /* flash page size */
    int      nchs, luns_per_ch;
    uint64_t *lun_rd;        /* flash page reads per [ch * luns_per_ch + lun] */

    struct bb_stats_ctr c;
    struct bb_stats_iv  iv;
};

void bb_stats_init(struct bb_stats *st, struct FemuCtrl *n,
                   uint32_t page_bytes, int nchs, int luns_per_ch);

/*
 * Call once per completed host read/write command. service_ns is the
 * contention-free NAND time of one page (pg_rd_lat or pg_wr_lat); anything
 * above it counts as queueing.
 */
void bb_stats_io_done(struct bb_stats *st, bool is_write, uint64_t bytes,
                      uint64_t lat_ns, uint64_t service_ns);

/* Call when the FTL thread finds no work; prints logs/report when due. */
void bb_stats_idle_tick(struct bb_stats *st);

/* Call for every flash page read issued to NAND. */
static inline void bb_stats_flash_read(struct bb_stats *st, int ch, int lun,
                                       bool is_gc)
{
    if (is_gc) {
        st->c.flash_rd_gc++;
    } else {
        st->c.flash_rd_user++;
    }
    st->lun_rd[ch * st->luns_per_ch + lun]++;
}

#endif
