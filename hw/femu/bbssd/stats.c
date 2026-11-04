#include "../nvme.h"
#include "stats.h"

#define BB_STATS_DEFAULT_IDLE_SEC 60
/* read the clock only every N idle passes of the FTL loop */
#define BB_STATS_IDLE_POLL_EVERY  4096

static void bb_stats_reset(struct bb_stats *st)
{
    uint64_t idle_ns = st->idle_ns;
    uint32_t page_bytes = st->page_bytes;

    memset(st, 0, sizeof(*st));
    st->idle_ns = idle_ns;
    st->page_bytes = page_bytes;
}

void bb_stats_init(struct bb_stats *st, uint32_t page_bytes)
{
    const char *env = getenv("FEMU_STATS_IDLE_SEC");
    long sec = BB_STATS_DEFAULT_IDLE_SEC;

    if (env && *env) {
        char *end;
        long v = strtol(env, &end, 10);
        if (*end == '\0' && v > 0) {
            sec = v;
        } else {
            femu_err("stats: ignoring FEMU_STATS_IDLE_SEC=%s\n", env);
        }
    }

    st->idle_ns = (uint64_t)sec * 1000000000ULL;
    st->page_bytes = page_bytes;
    bb_stats_reset(st);
    femu_log("stats: report after %lds of idleness\n", sec);
}

void bb_stats_io_done(struct bb_stats *st, bool is_write, uint64_t bytes,
                      uint64_t lat_ns)
{
    uint64_t now = qemu_clock_get_ns(QEMU_CLOCK_REALTIME);

    if (!st->active) {
        st->active = true;
        st->start_ns = now;
    }
    st->last_io_ns = now;
    st->idle_polls = 0;

    if (is_write) {
        st->host_wr_cmds++;
        st->host_wr_bytes += bytes;
        st->wr_lat_sum += lat_ns;
        if (lat_ns > st->wr_lat_max) {
            st->wr_lat_max = lat_ns;
        }
    } else {
        st->host_rd_cmds++;
        st->host_rd_bytes += bytes;
        st->rd_lat_sum += lat_ns;
        if (lat_ns > st->rd_lat_max) {
            st->rd_lat_max = lat_ns;
        }
    }
}

static double avg_us(uint64_t sum_ns, uint64_t n)
{
    return n ? (double)sum_ns / n / 1000.0 : 0.0;
}

static void bb_stats_report(struct bb_stats *st, const char *devname)
{
    uint64_t flash_rd = st->flash_rd_user + st->flash_rd_gc;
    uint64_t cmds = st->host_rd_cmds + st->host_wr_cmds;
    double secs = (double)(st->last_io_ns - st->start_ns) / 1e9;
    double host_rd_pages = st->page_bytes ?
        (double)st->host_rd_bytes / st->page_bytes : 0.0;
    double ra_total = host_rd_pages > 0 ? flash_rd / host_rd_pages : 0.0;
    double ra_user = host_rd_pages > 0 ? st->flash_rd_user / host_rd_pages : 0.0;

    printf("\n===== FEMU stats [%s] workload done (%.1fs active) =====\n",
           devname, secs);
    printf("Flash page reads   : %" PRIu64 " (host %" PRIu64 ", GC %" PRIu64 ")\n",
           flash_rd, st->flash_rd_user, st->flash_rd_gc);
    printf("Host reads         : %" PRIu64 " cmds, %.2f MiB (%.1f pages of %u B)\n",
           st->host_rd_cmds, st->host_rd_bytes / 1048576.0, host_rd_pages,
           st->page_bytes);
    printf("Read amplification : %.3f (host-only %.3f)\n", ra_total, ra_user);
    printf("Avg latency        : read %.2f us, write %.2f us, all %.2f us\n",
           avg_us(st->rd_lat_sum, st->host_rd_cmds),
           avg_us(st->wr_lat_sum, st->host_wr_cmds),
           avg_us(st->rd_lat_sum + st->wr_lat_sum, cmds));
    printf("Max latency        : read %.2f us, write %.2f us\n",
           st->rd_lat_max / 1000.0, st->wr_lat_max / 1000.0);
    printf("Host writes        : %" PRIu64 " cmds, %.2f MiB\n",
           st->host_wr_cmds, st->host_wr_bytes / 1048576.0);
    printf("===========================================================\n");
    fflush(stdout);
}

void bb_stats_idle_tick(struct bb_stats *st, const char *devname)
{
    if (!st->active || ++st->idle_polls < BB_STATS_IDLE_POLL_EVERY) {
        return;
    }
    st->idle_polls = 0;

    if (qemu_clock_get_ns(QEMU_CLOCK_REALTIME) - st->last_io_ns >= st->idle_ns) {
        bb_stats_report(st, devname);
        bb_stats_reset(st);
    }
}
