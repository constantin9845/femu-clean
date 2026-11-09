#include "../nvme.h"
#include "./ftl.h"

static void bb_init_ctrl_str(FemuCtrl *n)
{
    static int fsid_vbb = 0;
    const char *vbbssd_mn = "FEMU BlackBox-SSD Controller";
    const char *vbbssd_sn = "vSSD";

    nvme_set_ctrl_name(n, vbbssd_mn, vbbssd_sn, &fsid_vbb);
}

/* bb <=> black-box */
static void bb_init(FemuCtrl *n, Error **errp)
{
    struct ssd *ssd = n->ssd = g_malloc0(sizeof(struct ssd));

    bb_init_ctrl_str(n);

    ssd->dataplane_started_ptr = &n->dataplane_started;
    ssd->ssdname = (char *)n->devname;
    femu_debug("Starting FEMU in Blackbox-SSD mode ...\n");
    ssd_init(n);
}

static void bb_flip(FemuCtrl *n, NvmeCmd *cmd)
{
    struct ssd *ssd = n->ssd;
    int64_t cdw10 = le64_to_cpu(cmd->cdw10);

    switch (cdw10) {
    case FEMU_ENABLE_GC_DELAY:
        ssd->sp.enable_gc_delay = true;
        femu_log("%s,FEMU GC Delay Emulation [Enabled]!\n", n->devname);
        break;
    case FEMU_DISABLE_GC_DELAY:
        ssd->sp.enable_gc_delay = false;
        femu_log("%s,FEMU GC Delay Emulation [Disabled]!\n", n->devname);
        break;
    case FEMU_ENABLE_DELAY_EMU:
        ssd->sp.pg_rd_lat = NAND_READ_LATENCY;
        ssd->sp.pg_wr_lat = NAND_PROG_LATENCY;
        ssd->sp.blk_er_lat = NAND_ERASE_LATENCY;
        ssd->sp.ch_xfer_lat = 0;
        femu_log("%s,FEMU Delay Emulation [Enabled]!\n", n->devname);
        break;
    case FEMU_DISABLE_DELAY_EMU:
        ssd->sp.pg_rd_lat = 0;
        ssd->sp.pg_wr_lat = 0;
        ssd->sp.blk_er_lat = 0;
        ssd->sp.ch_xfer_lat = 0;
        femu_log("%s,FEMU Delay Emulation [Disabled]!\n", n->devname);
        break;
    case FEMU_RESET_ACCT: {
        /* counters are sharded per poller (see FemuPollerCtr); sum then reset */
        int64_t tt = 0, late = 0;
        if (n->poller_ctr) {
            for (uint32_t p = 1; p <= n->nr_pollers; p++) {
                tt += n->poller_ctr[p].nr_tt_ios;
                late += n->poller_ctr[p].nr_tt_late_ios;
                n->poller_ctr[p].nr_tt_ios = 0;
                n->poller_ctr[p].nr_tt_late_ios = 0;
            }
        }
        femu_log("%s,Reset tt_late_ios/tt_ios,%ld/%ld\n", n->devname, late, tt);
        break;
    }
    case FEMU_ENABLE_LOG:
        n->print_log = true;
        femu_log("%s,Log print [Enabled]!\n", n->devname);
        break;
    case FEMU_DISABLE_LOG:
        n->print_log = false;
        femu_log("%s,Log print [Disabled]!\n", n->devname);
        break;
    default:
        printf("FEMU:%s,Not implemented flip cmd (%lu)\n", n->devname, cdw10);
    }
}

static uint16_t bb_nvme_rw(FemuCtrl *n, NvmeNamespace *ns, NvmeCmd *cmd,
                           NvmeRequest *req)
{
    return nvme_rw(n, ns, cmd, req);
}

#ifdef HOST_FTL
/*
 * Vector I/O (docs/BBSSD-VECTOR-IO.md). The command carries nr physical sector
 * addresses (dense HOST_FTL encoding, see dev_lba_to_ppa):
 *   cdw12[15:0] : nr - 1
 *   cdw13[0]    : 0 = list is at the start of the data buffer, data follows at
 *                 the next BB_VEC_LIST_ALIGN boundary;
 *                 1 = cdw10-11 is a guest-physical pointer to the list, or the
 *                 address itself when nr == 1 (OCSSD 2.0 convention)
 * Data moves here; NAND timing is charged later in the FTL thread from
 * req->vec.
 */
static uint16_t bb_vec_err(FemuCtrl *n, NvmeNamespace *ns, NvmeCmd *cmd,
                           NvmeRequest *req, uint16_t status, uint32_t field,
                           uint64_t info)
{
    nvme_set_error_page(n, req->sq->sqid, cmd->cid, status, field, info,
                        ns->id);
    return status | NVME_DNR;
}

/* copy the first @len bytes described by @qsg into @buf */
static int bb_qsg_read(QEMUSGList *qsg, void *buf, uint64_t len)
{
    uint8_t *p = buf;

    for (int i = 0; i < qsg->nsg && len; i++) {
        dma_addr_t chunk = MIN(len, qsg->sg[i].len);
        if (dma_memory_read(qsg->as, qsg->sg[i].base, p, chunk,
                            MEMTXATTRS_UNSPECIFIED)) {
            return -1;
        }
        p += chunk;
        len -= chunk;
    }

    return len ? -1 : 0;
}

static uint16_t bb_vec_rw(FemuCtrl *n, NvmeNamespace *ns, NvmeCmd *cmd,
                          NvmeRequest *req)
{
    struct ssdparams *spp = &n->ssd->sp;
    uint8_t opc = cmd->opcode;
    uint32_t nr = (le32_to_cpu(cmd->cdw12) & 0xffff) + 1;
    bool list_in_buf = !(le32_to_cpu(cmd->cdw13) & 0x1);
    uint64_t lbal = le32_to_cpu(cmd->cdw10) |
                    ((uint64_t)le32_to_cpu(cmd->cdw11) << 32);
    const uint8_t lba_index = NVME_ID_NS_FLBAS_INDEX(ns->id_ns.flbas);
    const uint8_t data_shift = ns->id_ns.lbaf[lba_index].lbads;
    uint64_t list_bytes = (uint64_t)nr * sizeof(uint64_t);
    uint64_t list_len = list_in_buf ? ROUND_UP(list_bytes, BB_VEC_LIST_ALIGN) : 0;
    uint64_t data_len = (opc == NVME_CMD_OC_ERASE) ? 0 : (uint64_t)nr << data_shift;
    uint64_t total = list_len + data_len;
    uint64_t max_addr = MIN(le64_to_cpu(ns->id_ns.nsze),
                            (uint64_t)spp->tt_pgs * spp->secs_per_pg);
    bool mapped = false;
    uint16_t err;

    req->is_write = (opc == NVME_CMD_OC_WRITE);

    if (nr > BB_MAX_VEC) {
        return bb_vec_err(n, ns, cmd, req, NVME_INVALID_FIELD,
                          offsetof(NvmeCmd, cdw12), nr);
    }
    if ((1 << data_shift) != spp->secsz) {
        femu_err("vector I/O: LBA size %d != FTL sector size %d\n",
                 1 << data_shift, spp->secsz);
        return bb_vec_err(n, ns, cmd, req, NVME_INVALID_FIELD,
                          offsetof(NvmeCmd, cdw12), data_shift);
    }
    if (n->id_ctrl.mdts && total > n->page_size * (1 << n->id_ctrl.mdts)) {
        return bb_vec_err(n, ns, cmd, req, NVME_INVALID_FIELD,
                          offsetof(NvmeCmd, cdw12), nr);
    }

    if (!req->vec) {
        req->vec = g_malloc(BB_MAX_VEC * sizeof(uint64_t));
    }

    /* map the guest buffer (list and/or data) */
    if (total) {
        uint64_t prp1 = le64_to_cpu(cmd->dptr.prp1);
        uint64_t prp2 = le64_to_cpu(cmd->dptr.prp2);

        if (cmd->psdt) {
            if (!n->sgl || cmd->psdt > NVME_PSDT_SGL_MPTR_SGL ||
                nvme_map_sgl(&req->qsg, &req->iov, cmd->dptr.sgl, total, n)) {
                return bb_vec_err(n, ns, cmd, req, NVME_INVALID_FIELD,
                                  offsetof(NvmeCmd, dptr), 0);
            }
        } else if (nvme_map_prp(&req->qsg, &req->iov, prp1, prp2, total, n)) {
            return bb_vec_err(n, ns, cmd, req, NVME_INVALID_FIELD,
                              offsetof(NvmeCmd, dptr), 0);
        }
        if (req->qsg.nsg == 0) {
            /* buffer in the controller memory buffer: not supported here */
            qemu_iovec_destroy(&req->iov);
            return bb_vec_err(n, ns, cmd, req, NVME_INVALID_FIELD,
                              offsetof(NvmeCmd, dptr), 0);
        }
        mapped = true;
    }

    /* fetch the address list */
    if (list_in_buf) {
        if (bb_qsg_read(&req->qsg, req->vec, list_bytes)) {
            err = bb_vec_err(n, ns, cmd, req, NVME_INVALID_FIELD,
                             offsetof(NvmeCmd, dptr), 0);
            goto fail;
        }
    } else if (nr == 1) {
        req->vec[0] = lbal;
    } else {
        nvme_addr_read(n, lbal, req->vec, list_bytes);
    }

    for (uint32_t k = 0; k < nr; k++) {
        if (list_in_buf || nr > 1) {
            req->vec[k] = le64_to_cpu(req->vec[k]);
        }
        if (req->vec[k] >= max_addr) {
            err = bb_vec_err(n, ns, cmd, req, NVME_LBA_RANGE,
                             offsetof(NvmeCmd, cdw10), req->vec[k]);
            goto fail;
        }
    }

    /* writes program whole NAND pages: aligned runs of secs_per_pg sectors */
    if (req->is_write) {
        uint32_t spg = spp->secs_per_pg;
        bool ok = (nr % spg) == 0;

        for (uint32_t k = 0; ok && k < nr; k += spg) {
            ok = (req->vec[k] % spg) == 0;
            for (uint32_t j = 1; ok && j < spg; j++) {
                ok = req->vec[k + j] == req->vec[k] + j;
            }
        }
        if (!ok) {
            err = bb_vec_err(n, ns, cmd, req, NVME_INVALID_FIELD,
                             offsetof(NvmeCmd, cdw10), nr);
            goto fail;
        }
    }

    /* move the data; backend_rw_vec consumes the mapping */
    if (data_len) {
        mapped = false;
        if (backend_rw_vec(n->mbe, &req->qsg, list_len, req->vec, nr,
                           data_shift, req->is_write)) {
            return NVME_INTERNAL_DEV_ERROR;
        }
    } else if (mapped) {
        qemu_sglist_destroy(&req->qsg);
        mapped = false;
    }

    if (req->is_write) {
        for (uint32_t k = 0; k < nr; k++) {
            nvme_mark_written(ns, req->vec[k], 1);
        }
    }

    req->slba = req->vec[0];
    req->nlb = nr;
    req->nr_vec = nr;
    req->status = NVME_SUCCESS;

    return NVME_SUCCESS;

fail:
    if (mapped) {
        qemu_sglist_destroy(&req->qsg);
    }
    return err;
}
#endif /* HOST_FTL */

static uint16_t bb_io_cmd(FemuCtrl *n, NvmeNamespace *ns, NvmeCmd *cmd,
                          NvmeRequest *req)
{
    switch (cmd->opcode) {
    case NVME_CMD_READ:
    case NVME_CMD_WRITE:
        return bb_nvme_rw(n, ns, cmd, req);
#ifdef HOST_FTL
    case NVME_CMD_OC_ERASE:
    case NVME_CMD_OC_WRITE:
    case NVME_CMD_OC_READ:
        return bb_vec_rw(n, ns, cmd, req);
#endif
    default:
        return NVME_INVALID_OPCODE | NVME_DNR;
    }
}

static uint16_t bb_admin_cmd(FemuCtrl *n, NvmeCmd *cmd)
{
    switch (cmd->opcode) {
    case NVME_ADM_CMD_FEMU_FLIP:
        bb_flip(n, cmd);
        return NVME_SUCCESS;
    default:
        return NVME_INVALID_OPCODE | NVME_DNR;
    }
}

int nvme_register_bbssd(FemuCtrl *n)
{
    n->ext_ops = (FemuExtCtrlOps) {
        .state            = NULL,
        .init             = bb_init,
        .exit             = NULL,
        .rw_check_req     = NULL,
        .admin_cmd        = bb_admin_cmd,
        .io_cmd           = bb_io_cmd,
        .get_log          = NULL,
    };

    return 0;
}

