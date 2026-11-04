#include "../nvme.h"
#include "stats.h"

#define BB_STATS_DEFAULT_IDLE_SEC 60
/* read the clock only every N idle passes of the FTL loop */
#define BB_STATS_IDLE_POLL_EVERY  4096

#define SUB_BITS  BB_LAT_SUB_BITS
#define SUB_COUNT (1 << SUB_BITS)

/* ---- latency histogram ---- */

static int lat_bucket(uint64_t v)
{
    if (v < SUB_COUNT) {
        return (int)v;
    }
    int msb = 63 - __builtin_clzll(v);
    int sub = (int)(v >> (msb - SUB_BITS)) & (SUB_COUNT - 1);
    return ((msb - SUB_BITS + 1) << SUB_BITS) + sub;
}

/* highest value that falls into bucket idx */
static uint64_t lat_bucket_high(int idx)
{
    if (idx < SUB_COUNT) {
        return idx;
    }
    int shift = (idx >> SUB_BITS) - 1;
    uint64_t low = (uint64_t)(SUB_COUNT + (idx & (SUB_COUNT - 1))) << shift;
    return low + ((1ULL << shift) - 1);
}

static void hist_add(struct bb_lat_hist *h, uint64_t v)
{
    h->b[lat_bucket(v)]++;
}

/* p in (0, 1]; result capped at the observed max */
static uint64_t hist_pct(const struct bb_lat_hist *h, uint64_t count,
                         uint64_t max, double p)
{
    uint64_t target = (uint64_t)(p * count + 0.999999);
    uint64_t seen = 0;

    if (count == 0) {
        return 0;
    }
    if (target == 0) {
        target = 1;
    }
    for (int i = 0; i < BB_LAT_BUCKETS; i++) {
        seen += h->b[i];
        if (seen >= target) {
            uint64_t hi = lat_bucket_high(i);
            return hi < max ? hi : max;
        }
    }
    return max;
}

/* ---- poller completion counters (written by poller threads) ---- */

static void poller_totals(FemuCtrl *n, int64_t *tt, int64_t *late)
{
    *tt = 0;
    *late = 0;
    if (!n || !n->poller_ctr) {
        return;
    }
    for (uint32_t p = 1; p <= n->nr_pollers; p++) {
        *tt += qatomic_read(&n->poller_ctr[p].nr_tt_ios);
        *late += qatomic_read(&n->poller_ctr[p].nr_tt_late_ios);
    }
}

/* ---- lifecycle ---- */

static void bb_stats_reset(struct bb_stats *st)
{
    memset(&st->c, 0, sizeof(st->c));
    memset(st->lun_rd, 0, sizeof(uint64_t) * st->nchs * st->luns_per_ch);
}

void bb_stats_init(struct bb_stats *st, FemuCtrl *n, uint32_t page_bytes,
                   int nchs, int luns_per_ch)
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

    st->n = n;
    st->idle_ns = (uint64_t)sec * 1000000000ULL;
    st->page_bytes = page_bytes;
    st->nchs = nchs;
    st->luns_per_ch = luns_per_ch;
    st->lun_rd = g_malloc0(sizeof(uint64_t) * nchs * luns_per_ch);
    bb_stats_reset(st);
    femu_log("stats: report after %lds of idleness\n", sec);
}

void bb_stats_io_done(struct bb_stats *st, bool is_write, uint64_t bytes,
                      uint64_t lat_ns, uint64_t service_ns)
{
    struct bb_stats_ctr *c = &st->c;
    uint64_t now = qemu_clock_get_ns(QEMU_CLOCK_REALTIME);
    uint64_t queue_ns = lat_ns > service_ns ? lat_ns - service_ns : 0;

    if (!c->active) {
        c->active = true;
        c->start_ns = now;
        poller_totals(st->n, &c->base_tt_ios, &c->base_late_ios);
    }
    c->last_io_ns = now;
    c->idle_polls = 0;

    if (is_write) {
        c->host_wr_cmds++;
        c->host_wr_bytes += bytes;
        c->wr_lat_sum += lat_ns;
        c->wr_queue_sum += queue_ns;
        if (lat_ns > c->wr_lat_max) {
            c->wr_lat_max = lat_ns;
        }
        hist_add(&c->wr_hist, lat_ns);
    } else {
        c->host_rd_cmds++;
        c->host_rd_bytes += bytes;
        c->rd_lat_sum += lat_ns;
        c->rd_queue_sum += queue_ns;
        if (lat_ns > c->rd_lat_max) {
            c->rd_lat_max = lat_ns;
        }
        hist_add(&c->rd_hist, lat_ns);
    }
}

/* ---- report ---- */

static double avg_us(uint64_t sum_ns, uint64_t n)
{
    return n ? (double)sum_ns / n / 1000.0 : 0.0;
}

static double pct(uint64_t part, uint64_t whole)
{
    return whole ? 100.0 * part / whole : 0.0;
}

static void report_latency(const char *name, const struct bb_lat_hist *h,
                           uint64_t cmds, uint64_t sum, uint64_t queue_sum,
                           uint64_t max)
{
    if (cmds == 0) {
        printf("  %-5s: no commands\n", name);
        return;
    }
    printf("  %-5s: avg %.2f  p50 %.2f  p99 %.2f  p99.9 %.2f  max %.2f us\n",
           name, avg_us(sum, cmds),
           hist_pct(h, cmds, max, 0.50) / 1000.0,
           hist_pct(h, cmds, max, 0.99) / 1000.0,
           hist_pct(h, cmds, max, 0.999) / 1000.0,
           max / 1000.0);
    printf("         queueing avg %.2f us (%.1f%% of latency), "
           "NAND avg %.2f us\n",
           avg_us(queue_sum, cmds), pct(queue_sum, sum),
           avg_us(sum - queue_sum, cmds));
}

static void report_lun_balance(struct bb_stats *st)
{
    int nluns = st->nchs * st->luns_per_ch;
    uint64_t total = 0, min = UINT64_MAX, max = 0;
    int min_i = 0, max_i = 0;

    for (int i = 0; i < nluns; i++) {
        uint64_t v = st->lun_rd[i];
        total += v;
        if (v < min) {
            min = v;
            min_i = i;
        }
        if (v > max) {
            max = v;
            max_i = i;
        }
    }
    if (total == 0) {
        printf("Read balance       : no flash reads\n");
        return;
    }

    double mean = (double)total / nluns;
    printf("Read balance (LUN) : min %" PRIu64 " (ch%d/lun%d), "
           "max %" PRIu64 " (ch%d/lun%d), mean %.1f, max/mean %.2f\n",
           min, min_i / st->luns_per_ch, min_i % st->luns_per_ch,
           max, max_i / st->luns_per_ch, max_i % st->luns_per_ch,
           mean, max / mean);

    printf("Reads per channel  :");
    for (int ch = 0; ch < st->nchs; ch++) {
        uint64_t sum = 0;
        for (int l = 0; l < st->luns_per_ch; l++) {
            sum += st->lun_rd[ch * st->luns_per_ch + l];
        }
        printf(" %" PRIu64, sum);
    }
    printf("\n");
}

static void bb_stats_report(struct bb_stats *st)
{
    struct bb_stats_ctr *c = &st->c;
    const char *devname = st->n ? st->n->devname : "?";
    uint64_t flash_rd = c->flash_rd_user + c->flash_rd_gc;
    double secs = (double)(c->last_io_ns - c->start_ns) / 1e9;
    double host_rd_pages = st->page_bytes ?
        (double)c->host_rd_bytes / st->page_bytes : 0.0;
    double ra_total = host_rd_pages > 0 ? flash_rd / host_rd_pages : 0.0;
    double ra_user = host_rd_pages > 0 ? c->flash_rd_user / host_rd_pages : 0.0;
    int64_t tt, late;

    printf("\n===== FEMU stats [%s] workload done (%.1fs active) =====\n",
           devname, secs);

    printf("Flash page reads   : %" PRIu64 " (host %" PRIu64 ", GC %" PRIu64 ")\n",
           flash_rd, c->flash_rd_user, c->flash_rd_gc);
    printf("Host reads         : %" PRIu64 " cmds, %.2f MiB (%.1f pages of %u B)\n",
           c->host_rd_cmds, c->host_rd_bytes / 1048576.0, host_rd_pages,
           st->page_bytes);
    printf("Host writes        : %" PRIu64 " cmds, %.2f MiB\n",
           c->host_wr_cmds, c->host_wr_bytes / 1048576.0);
    printf("Read amplification : %.3f (host-only %.3f)\n", ra_total, ra_user);

    if (secs > 0) {
        printf("Throughput         : read %.0f IOPS %.2f MiB/s, "
               "write %.0f IOPS %.2f MiB/s\n",
               c->host_rd_cmds / secs, c->host_rd_bytes / 1048576.0 / secs,
               c->host_wr_cmds / secs, c->host_wr_bytes / 1048576.0 / secs);
    } else {
        printf("Throughput         : n/a (workload too short)\n");
    }

    printf("Latency (emulated device time):\n");
    report_latency("read", &c->rd_hist, c->host_rd_cmds, c->rd_lat_sum,
                   c->rd_queue_sum, c->rd_lat_max);
    report_latency("write", &c->wr_hist, c->host_wr_cmds, c->wr_lat_sum,
                   c->wr_queue_sum, c->wr_lat_max);

    report_lun_balance(st);

    poller_totals(st->n, &tt, &late);
    if (tt < c->base_tt_ios || late < c->base_late_ios) {
        printf("Late completions   : n/a (counters reset during workload)\n");
    } else {
        int64_t d_tt = tt - c->base_tt_ios;
        int64_t d_late = late - c->base_late_ios;
        printf("Late completions   : %" PRId64 " of %" PRId64 " (%.3f%%) "
               "posted >20us after their deadline\n",
               d_late, d_tt, pct(d_late, d_tt));
    }

    printf("===========================================================\n");
    fflush(stdout);
}

void bb_stats_idle_tick(struct bb_stats *st)
{
    struct bb_stats_ctr *c = &st->c;

    if (!c->active || ++c->idle_polls < BB_STATS_IDLE_POLL_EVERY) {
        return;
    }
    c->idle_polls = 0;

    if (qemu_clock_get_ns(QEMU_CLOCK_REALTIME) - c->last_io_ns >= st->idle_ns) {
        bb_stats_report(st);
        bb_stats_reset(st);
    }
}
