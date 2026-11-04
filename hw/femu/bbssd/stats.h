#ifndef __FEMU_BB_STATS_H
#define __FEMU_BB_STATS_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Per-workload FTL statistics for blackbox mode.
 *
 * Only the FTL thread touches this, so there is no locking. A workload starts
 * at the first host read/write and ends once no I/O has arrived for idle_ns
 * (default 60s, override with FEMU_STATS_IDLE_SEC). At that point the report
 * is printed and the counters are reset for the next workload.
 */
struct bb_stats {
    uint64_t idle_ns;        /* idle time that ends a workload */
    uint32_t page_bytes;     /* flash page size */

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
};

void bb_stats_init(struct bb_stats *st, uint32_t page_bytes);

/* Call once per completed host read/write command. */
void bb_stats_io_done(struct bb_stats *st, bool is_write, uint64_t bytes,
                      uint64_t lat_ns);

/* Call when the FTL thread finds no work; prints the report once idle. */
void bb_stats_idle_tick(struct bb_stats *st, const char *devname);

/* Call for every flash page read issued to NAND. */
static inline void bb_stats_flash_read(struct bb_stats *st, bool is_gc)
{
    if (is_gc) {
        st->flash_rd_gc++;
    } else {
        st->flash_rd_user++;
    }
}

#endif
