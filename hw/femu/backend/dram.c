#include <sys/syscall.h>
#include "../nvme.h"

/* Coperd: FEMU Memory Backend (mbe) for emulated SSD */

/*
 * Optionally bind the emulated-SSD backend buffer to a NUMA node before it is
 * first touched. FEMU_MBE_INTERLEAVE=<n> binds every page to node n
 * (MPOL_BIND); =on interleaves across nodes 0-1 (MPOL_INTERLEAVE). This is the
 * device-side half of the strict socket-isolation setup: pin the guest's vCPUs
 * and RAM to one socket and the FEMU poller threads + this backend to the
 * other, so the only thing crossing the inter-socket link is the per-I/O
 * emulation copy and the two DRAM-bandwidth domains do not contend. A direct
 * mbind(2) syscall avoids a libnuma build dependency. No-op (default) when the
 * env var is unset, so existing single-socket setups are unchanged.
 */
static void mbe_numa_bind(void *addr, int64_t len)
{
    const char *il = getenv("FEMU_MBE_INTERLEAVE");
    if (!il || !il[0]) {
        return;
    }
    if (strcmp(il, "on") && strcmp(il, "0") && strcmp(il, "1")) {
        femu_err("backend: ignoring FEMU_MBE_INTERLEAVE=%s (want on|0|1)\n", il);
        return;
    }
    int mode = strcmp(il, "on") ? 2 /* MPOL_BIND */ : 3 /* MPOL_INTERLEAVE */;
    unsigned long nodemask = strcmp(il, "on") ? (1ul << atoi(il)) : 0x3ul;

    /*
     * mbind(2) requires a page-aligned start address; g_malloc0 returns only
     * malloc alignment. Round the start down and the length up to whole pages
     * so the policy covers every page backing the buffer. (Touching the shared
     * boundary pages is harmless: the policy only steers where not-yet-faulted
     * pages land, and the prealloc below faults our buffer immediately.)
     */
    long pgsz = sysconf(_SC_PAGESIZE);
    uintptr_t start = (uintptr_t)addr & ~((uintptr_t)pgsz - 1);
    uintptr_t end = ((uintptr_t)addr + len + pgsz - 1) & ~((uintptr_t)pgsz - 1);
    /*
     * MPOL_MF_MOVE (0x2): g_malloc0 has already zeroed (hence faulted) the
     * pages onto the default node, so a plain mbind would not relocate them.
     * MF_MOVE migrates the already-resident pages to the target node now.
     */
    if (syscall(SYS_mbind, (void *)start, end - start, mode, &nodemask,
                sizeof(nodemask) * 8, 0x2 /* MPOL_MF_MOVE */)) {
        femu_err("backend: mbind(FEMU_MBE_INTERLEAVE=%s) failed: %s\n",
                 il, strerror(errno));
    } else {
        femu_log("backend: %ld MB bound via FEMU_MBE_INTERLEAVE=%s\n",
                 (long)(len >> 20), il);
    }
}

int init_dram_backend(SsdDramBackend **mbe, int64_t nbytes)
{
    SsdDramBackend *b = *mbe = g_malloc0(sizeof(SsdDramBackend));

    b->size = nbytes;
    b->logical_space = g_malloc0(nbytes);

    /* bind to the requested NUMA node before mlock faults the pages in */
    mbe_numa_bind(b->logical_space, nbytes);

    if (mlock(b->logical_space, nbytes) == -1) {
        femu_err("Failed to pin the memory backend to the host DRAM\n");
        g_free(b->logical_space);
        abort();
    }

    return 0;
}

void free_dram_backend(SsdDramBackend *b)
{
    if (b->logical_space) {
        munlock(b->logical_space, b->size);
        g_free(b->logical_space);
    }
}

int backend_rw(SsdDramBackend *b, QEMUSGList *qsg, uint64_t *lbal, bool is_write)
{
    int sg_cur_index = 0;
    dma_addr_t sg_cur_byte = 0;
    dma_addr_t cur_addr, cur_len;
    uint64_t mb_oft = lbal[0];
    void *mb = b->logical_space;

    DMADirection dir = DMA_DIRECTION_FROM_DEVICE;

    if (is_write) {
        dir = DMA_DIRECTION_TO_DEVICE;
    }

    while (sg_cur_index < qsg->nsg) {
        cur_addr = qsg->sg[sg_cur_index].base + sg_cur_byte;
        cur_len = qsg->sg[sg_cur_index].len - sg_cur_byte;
        if (dma_memory_rw(qsg->as, cur_addr, mb + mb_oft, cur_len, dir, MEMTXATTRS_UNSPECIFIED)) {
            femu_err("dma_memory_rw error: off=%" PRIu64 " len=%" PRIu64 " %s\n",
                     mb_oft, (uint64_t)cur_len, is_write ? "write" : "read");
            qemu_sglist_destroy(qsg);
            return -1;
        }

        /*
        if(dir == DMA_DIRECTION_TO_DEVICE){
            printf("[BACKEND WRITE LOG] Offset: %" PRIu64 " | Length: %" PRIu64 " bytes | SG Entry: %d\n",
                       mb_oft, (uint64_t)cur_len, sg_cur_index);
        }
        else{
            printf("[BACKEND READ LOG] Offset: %" PRIu64 " | Length: %" PRIu64 " bytes | SG Entry: %d\n",
                       mb_oft, (uint64_t)cur_len, sg_cur_index);
        }
        */
        

        sg_cur_byte += cur_len;
        if (sg_cur_byte == qsg->sg[sg_cur_index].len) {
            sg_cur_byte = 0;
            ++sg_cur_index;
        }

        if (b->femu_mode == FEMU_OCSSD_MODE) {
            mb_oft = lbal[sg_cur_index];
        } else if (b->femu_mode == FEMU_BBSSD_MODE ||
                   b->femu_mode == FEMU_NOSSD_MODE ||
                   b->femu_mode == FEMU_ZNSSD_MODE ||
                   b->femu_mode == FEMU_CSD_MODE ){
            mb_oft += cur_len;
        } else {
            assert(0);
        }
    }

    qemu_sglist_destroy(qsg);

    return 0;
}

/*
 * Vector copy for BBSSD HOST_FTL vector I/O: the guest buffer described by
 * @qsg holds @skip bytes of address list followed by @nr sectors of data.
 * Sector k of the data moves to/from backend offset vec[k] << @shift.
 * Runs of consecutive addresses are copied in one go. Scatter-gather entries
 * may be of any length; a sector may span two of them.
 *
 * Like backend_rw, it destroys @qsg before returning.
 */
int backend_rw_vec(SsdDramBackend *b, QEMUSGList *qsg, uint64_t skip,
                   const uint64_t *vec, uint32_t nr, int shift, bool is_write)
{
    DMADirection dir = is_write ? DMA_DIRECTION_TO_DEVICE
                                : DMA_DIRECTION_FROM_DEVICE;
    uint8_t *mb = b->logical_space;
    int idx = 0;
    dma_addr_t off = skip;
    uint32_t k = 0;
    int ret = 0;

    /* skip the address list at the front of the buffer */
    while (idx < qsg->nsg && off >= qsg->sg[idx].len) {
        off -= qsg->sg[idx].len;
        idx++;
    }

    while (k < nr) {
        uint32_t run = 1;
        while (k + run < nr && vec[k + run] == vec[k] + run) {
            run++;
        }

        uint64_t mb_oft = vec[k] << shift;
        uint64_t left = (uint64_t)run << shift;
        if (mb_oft + left > (uint64_t)b->size) {
            femu_err("backend_rw_vec: off=%" PRIu64 " len=%" PRIu64
                     " past backend end\n", mb_oft, left);
            ret = -1;
            goto out;
        }

        while (left) {
            if (idx >= qsg->nsg) {
                femu_err("backend_rw_vec: buffer shorter than %u sectors\n", nr);
                ret = -1;
                goto out;
            }
            dma_addr_t chunk = MIN(left, qsg->sg[idx].len - off);
            if (dma_memory_rw(qsg->as, qsg->sg[idx].base + off, mb + mb_oft,
                              chunk, dir, MEMTXATTRS_UNSPECIFIED)) {
                femu_err("backend_rw_vec: dma error off=%" PRIu64 " len=%"
                         PRIu64 " %s\n", mb_oft, (uint64_t)chunk,
                         is_write ? "write" : "read");
                ret = -1;
                goto out;
            }
            mb_oft += chunk;
            left -= chunk;
            off += chunk;
            if (off == qsg->sg[idx].len) {
                off = 0;
                idx++;
            }
        }
        k += run;
    }

out:
    qemu_sglist_destroy(qsg);
    return ret;
}
