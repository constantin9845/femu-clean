#include "../nvme.h"
#include "stats.h"

#define BB_STATS_DEFAULT_IDLE_SEC     60
#define BB_STATS_DEFAULT_INTERVAL_SEC 5
/* read the clock only every N idle passes of the FTL loop */
#define BB_STATS_IDLE_POLL_EVERY      4096

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

/* p in (0, 1]; result capped at the observed max */
static uint64_t acc_pct(const struct bb_io_acc *a, double p)
{
    uint64_t target = (uint64_t)(p * a->cmds + 0.999999);
    uint64_t seen = 0;

    if (a->cmds == 0) {
        return 0;
    }
    if (target == 0) {
        target = 1;
    }
    for (int i = 0; i < BB_LAT_BUCKETS; i++) {
        seen += a->hist.b[i];
        if (seen >= target) {
            uint64_t hi = lat_bucket_high(i);
            return hi < a->lat_max ? hi : a->lat_max;
        }
    }
    return a->lat_max;
}

static void acc_add(struct bb_io_acc *a, uint64_t bytes, uint64_t lat_ns,
                    uint64_t queue_ns)
{
    a->cmds++;
    a->bytes += bytes;
    a->lat_sum += lat_ns;
    a->queue_sum += queue_ns;
    if (lat_ns > a->lat_max) {
        a->lat_max = lat_ns;
    }
    a->hist.b[lat_bucket(lat_ns)]++;
}

/* ---- helpers ---- */

static double avg_us(uint64_t sum_ns, uint64_t n)
{
    return n ? (double)sum_ns / n / 1000.0 : 0.0;
}

static double pct(uint64_t part, uint64_t whole)
{
    return whole ? 100.0 * part / whole : 0.0;
}

static uint64_t flash_rd_total(const struct bb_stats *st)
{
    return st->c.flash_rd_user + st->c.flash_rd_gc;
}

static double read_amp(const struct bb_stats *st, uint64_t flash_rd,
                       uint64_t host_rd_bytes)
{
    return host_rd_bytes ?
        (double)flash_rd * st->page_bytes / host_rd_bytes : 0.0;
}

static long env_sec(const char *name, long def, bool allow_zero)
{
    const char *env = getenv(name);
    char *end;
    long v;

    if (!env || !*env) {
        return def;
    }
    v = strtol(env, &end, 10);
    if (*end != '\0' || v < 0 || (v == 0 && !allow_zero)) {
        femu_err("stats: ignoring %s=%s\n", name, env);
        return def;
    }
    return v;
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

static void interval_start(struct bb_stats *st, uint64_t now)
{
    memset(&st->iv, 0, sizeof(st->iv));
    st->iv.start_ns = now;
    st->iv.base_flash_rd = flash_rd_total(st);
    poller_totals(st->n, &st->iv.base_tt_ios, &st->iv.base_late_ios);
}

static void bb_stats_reset(struct bb_stats *st)
{
    memset(&st->c, 0, sizeof(st->c));
    memset(&st->iv, 0, sizeof(st->iv));
    memset(st->lun_rd, 0, sizeof(uint64_t) * st->nchs * st->luns_per_ch);
}

void bb_stats_init(struct bb_stats *st, FemuCtrl *n, uint32_t page_bytes,
                   int nchs, int luns_per_ch)
{
    long idle = env_sec("FEMU_STATS_IDLE_SEC", BB_STATS_DEFAULT_IDLE_SEC,
                        false);
    long iv = env_sec("FEMU_STATS_INTERVAL_SEC", BB_STATS_DEFAULT_INTERVAL_SEC,
                      true);

    st->n = n;
    st->idle_ns = (uint64_t)idle * 1000000000ULL;
    st->interval_ns = (uint64_t)iv * 1000000000ULL;
    st->page_bytes = page_bytes;
    st->nchs = nchs;
    st->luns_per_ch = luns_per_ch;
    st->lun_rd = g_malloc0(sizeof(uint64_t) * nchs * luns_per_ch);
    bb_stats_reset(st);
    femu_log("stats: log every %lds, report after %lds of idleness\n",
             iv, idle);
}

/* ---- periodic log ---- */

static void log_dir(const char *name, const struct bb_io_acc *a, double secs)
{
    printf(" | %s %.0f IOPS %.2f MiB/s", name,
           a->cmds / secs, a->bytes / 1048576.0 / secs);
    if (a->cmds) {
        printf(" avg %.1f p99 %.1f us q %.0f%%",
               avg_us(a->lat_sum, a->cmds), acc_pct(a, 0.99) / 1000.0,
               pct(a->queue_sum, a->lat_sum));
    }
}

/* Print the interval if it is over, then start a new one. */
static void interval_maybe_log(struct bb_stats *st, uint64_t now)
{
    struct bb_stats_iv *iv = &st->iv;
    int64_t tt, late;

    if (st->interval_ns == 0 || now - iv->start_ns < st->interval_ns) {
        return;
    }

    /* stay quiet for intervals with no host I/O (e.g. while going idle) */
    if (iv->rd.cmds || iv->wr.cmds) {
        double secs = (double)(now - iv->start_ns) / 1e9;
        uint64_t flash_rd = flash_rd_total(st) - iv->base_flash_rd;

        poller_totals(st->n, &tt, &late);

        printf("[FEMU stats %s] t=%.0fs",
               st->n ? st->n->devname : "?",
               (double)(now - st->c.start_ns) / 1e9);
        log_dir("rd", &iv->rd, secs);
        log_dir("wr", &iv->wr, secs);
        printf(" | flash rd %" PRIu64 " RA %.2f", flash_rd,
               read_amp(st, flash_rd, iv->rd.bytes));
        if (tt >= iv->base_tt_ios && late >= iv->base_late_ios) {
            printf(" | late %.2f%%",
                   pct(late - iv->base_late_ios, tt - iv->base_tt_ios));
        }
        printf("\n");
        fflush(stdout);
    }

    interval_start(st, now);
}

/* ---- host I/O ---- */

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
        interval_start(st, now);
    } else {
        interval_maybe_log(st, now);
    }
    c->last_io_ns = now;
    c->idle_polls = 0;

    if (is_write) {
        acc_add(&c->wr, bytes, lat_ns, queue_ns);
        acc_add(&st->iv.wr, bytes, lat_ns, queue_ns);
    } else {
        acc_add(&c->rd, bytes, lat_ns, queue_ns);
        acc_add(&st->iv.rd, bytes, lat_ns, queue_ns);
    }
}

/* ---- final report ---- */

static void report_latency(const char *name, const struct bb_io_acc *a)
{
    if (a->cmds == 0) {
        printf("  %-5s: no commands\n", name);
        return;
    }
    printf("  %-5s: avg %.2f  p50 %.2f  p99 %.2f  p99.9 %.2f  max %.2f us\n",
           name, avg_us(a->lat_sum, a->cmds),
           acc_pct(a, 0.50) / 1000.0,
           acc_pct(a, 0.99) / 1000.0,
           acc_pct(a, 0.999) / 1000.0,
           a->lat_max / 1000.0);
    printf("         queueing avg %.2f us (%.1f%% of latency), "
           "NAND avg %.2f us\n",
           avg_us(a->queue_sum, a->cmds), pct(a->queue_sum, a->lat_sum),
           avg_us(a->lat_sum - a->queue_sum, a->cmds));
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
    uint64_t flash_rd = flash_rd_total(st);
    double secs = (double)(c->last_io_ns - c->start_ns) / 1e9;
    double host_rd_pages = st->page_bytes ?
        (double)c->rd.bytes / st->page_bytes : 0.0;
    int64_t tt, late;

    printf("\n===== FEMU stats [%s] workload done (%.1fs active) =====\n",
           st->n ? st->n->devname : "?", secs);

    printf("Flash page reads   : %" PRIu64 " (host %" PRIu64 ", GC %" PRIu64 ")\n",
           flash_rd, c->flash_rd_user, c->flash_rd_gc);
    printf("Host reads         : %" PRIu64 " cmds, %.2f MiB (%.1f pages of %u B)\n",
           c->rd.cmds, c->rd.bytes / 1048576.0, host_rd_pages, st->page_bytes);
    printf("Host writes        : %" PRIu64 " cmds, %.2f MiB\n",
           c->wr.cmds, c->wr.bytes / 1048576.0);
    printf("Read amplification : %.3f (host-only %.3f)\n",
           read_amp(st, flash_rd, c->rd.bytes),
           read_amp(st, c->flash_rd_user, c->rd.bytes));

    if (secs > 0) {
        printf("Throughput         : read %.0f IOPS %.2f MiB/s, "
               "write %.0f IOPS %.2f MiB/s\n",
               c->rd.cmds / secs, c->rd.bytes / 1048576.0 / secs,
               c->wr.cmds / secs, c->wr.bytes / 1048576.0 / secs);
    } else {
        printf("Throughput         : n/a (workload too short)\n");
    }

    printf("Latency (emulated device time):\n");
    report_latency("read", &c->rd);
    report_latency("write", &c->wr);

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
    uint64_t now;

    if (!c->active || ++c->idle_polls < BB_STATS_IDLE_POLL_EVERY) {
        return;
    }
    c->idle_polls = 0;

    now = qemu_clock_get_ns(QEMU_CLOCK_REALTIME);
    interval_maybe_log(st, now);

    if (now - c->last_io_ns >= st->idle_ns) {
        bb_stats_report(st);
        bb_stats_reset(st);
    }
}
