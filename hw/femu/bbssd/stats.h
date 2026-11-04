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
 * (default 60s, override with FEMU_STATS_IDLE_SEC). At that point the report
 * is printed and the counters are reset for the next workload.
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

/* Everything that is zeroed at the start of each workload. */
struct bb_stats_ctr {
    bool     active;         /* a workload is in progress */
    uint64_t start_ns;       /* first I/O of the workload */
    uint64_t last_io_ns;     /* most recent I/O */
    uint32_t idle_polls;     /* throttles clock reads while idle */

    uint64_t host_rd_cmds, host_wr_cmds;
    uint64_t host_rd_bytes, host_wr_bytes;

    uint64_t flash_rd_user;  /* flash page reads serving host reads */
    uint64_t flash_rd_gc;    /* flash page reads done by GC */

    /* emulated device latency per command, in ns */
    uint64_t rd_lat_sum, wr_lat_sum;
    uint64_t rd_lat_max, wr_lat_max;
    /* part of the latency spent waiting for a busy LUN, in ns */
    uint64_t rd_queue_sum, wr_queue_sum;
    struct bb_lat_hist rd_hist, wr_hist;

    /* poller completion counters when the workload started */
    int64_t  base_tt_ios, base_late_ios;
};

struct bb_stats {
    /* fixed at init */
    struct FemuCtrl *n;
    uint64_t idle_ns;        /* idle time that ends a workload */
    uint32_t page_bytes;     /* flash page size */
    int      nchs, luns_per_ch;
    uint64_t *lun_rd;        /* flash page reads per [ch * luns_per_ch + lun] */

    struct bb_stats_ctr c;
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

/* Call when the FTL thread finds no work; prints the report once idle. */
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
