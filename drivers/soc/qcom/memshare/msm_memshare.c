/* Copyright (c) 2013-2018, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#define DEBUG
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/dma-mapping.h>
#include <linux/mutex.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/notifier.h>
#include <linux/atomic.h>
#include <soc/qcom/subsystem_restart.h>
#include <soc/qcom/subsystem_notif.h>
#include <soc/qcom/msm_qmi_interface.h>
#include <soc/qcom/scm.h>
#include "msm_memshare.h"
#include "heap_mem_ext_v01.h"

#include <soc/qcom/secure_buffer.h>
#include <soc/qcom/ramdump.h>

/* Macros */
#define MEMSHARE_DEV_NAME "memshare"
#define MEMSHARE_CHILD_DEV_NAME "memshare_child"
static DEFINE_DMA_ATTRS(attrs);

/*
 * Debug tracing.  Use pr_info() rather than pr_debug() so the trace is
 * visible on a production/vendor kernel without dynamic-debug setup.
 * The sequence number makes interleaved lines from the QMI workqueue and
 * the subsystem notifier correlatable.  Remove/downgrade before upstreaming.
 */
static atomic_t memshare_trace_seq = ATOMIC_INIT(0);
#define msh_trace(fmt, ...) \
	pr_info("memshare[%4d]: " fmt, atomic_inc_return(&memshare_trace_seq), ##__VA_ARGS__)

static struct qmi_handle *mem_share_svc_handle;
static void mem_share_svc_recv_msg(struct work_struct *work);
static DECLARE_DELAYED_WORK(work_recv_msg, mem_share_svc_recv_msg);
static struct workqueue_struct *mem_share_svc_workqueue;
static uint64_t bootup_request;
static bool ramdump_event;
static void *memshare_ramdump_dev[MAX_CLIENTS];
static struct device *memshare_dev[MAX_CLIENTS];
static const char *msh_notif_name(unsigned long code)
{
	switch (code) {
	case SUBSYS_BEFORE_SHUTDOWN:
		return "BEFORE_SHUTDOWN";
	case SUBSYS_RAMDUMP_NOTIFICATION:
		return "RAMDUMP_NOTIFICATION";
	case SUBSYS_BEFORE_POWERUP:
		return "BEFORE_POWERUP";
	case SUBSYS_AFTER_POWERUP:
		return "AFTER_POWERUP";
	default:
		return "UNKNOWN";
	}
}

/* Memshare Driver Structure */
struct memshare_driver {
	struct device *dev;
	struct mutex mem_share;
	struct mutex mem_free;
	struct work_struct memshare_init_work;
};

struct memshare_child {
	struct device *dev;
};

static struct memshare_driver *memsh_drv;
static struct memshare_child *memsh_child;
static struct mem_blocks memblock[MAX_CLIENTS];
static uint32_t num_clients;
static struct msg_desc mem_share_svc_alloc_req_desc = {
	.max_msg_len = MEM_ALLOC_REQ_MAX_MSG_LEN_V01,
	.msg_id = MEM_ALLOC_REQ_MSG_V01,
	.ei_array = mem_alloc_req_msg_data_v01_ei,
};

static struct msg_desc mem_share_svc_alloc_resp_desc = {
	.max_msg_len = MEM_ALLOC_REQ_MAX_MSG_LEN_V01,
	.msg_id = MEM_ALLOC_RESP_MSG_V01,
	.ei_array = mem_alloc_resp_msg_data_v01_ei,
};

static struct msg_desc mem_share_svc_free_req_desc = {
	.max_msg_len = MEM_FREE_REQ_MAX_MSG_LEN_V01,
	.msg_id = MEM_FREE_REQ_MSG_V01,
	.ei_array = mem_free_req_msg_data_v01_ei,
};

static struct msg_desc mem_share_svc_free_resp_desc = {
	.max_msg_len = MEM_FREE_REQ_MAX_MSG_LEN_V01,
	.msg_id = MEM_FREE_RESP_MSG_V01,
	.ei_array = mem_free_resp_msg_data_v01_ei,
};

static struct msg_desc mem_share_svc_alloc_generic_req_desc = {
	.max_msg_len = MEM_ALLOC_REQ_MAX_MSG_LEN_V01,
	.msg_id = MEM_ALLOC_GENERIC_REQ_MSG_V01,
	.ei_array = mem_alloc_generic_req_msg_data_v01_ei,
};

static struct msg_desc mem_share_svc_alloc_generic_resp_desc = {
	.max_msg_len = MEM_ALLOC_REQ_MAX_MSG_LEN_V01,
	.msg_id = MEM_ALLOC_GENERIC_RESP_MSG_V01,
	.ei_array = mem_alloc_generic_resp_msg_data_v01_ei,
};

static struct msg_desc mem_share_svc_free_generic_req_desc = {
	.max_msg_len = MEM_FREE_REQ_MAX_MSG_LEN_V01,
	.msg_id = MEM_FREE_GENERIC_REQ_MSG_V01,
	.ei_array = mem_free_generic_req_msg_data_v01_ei,
};

static struct msg_desc mem_share_svc_free_generic_resp_desc = {
	.max_msg_len = MEM_FREE_REQ_MAX_MSG_LEN_V01,
	.msg_id = MEM_FREE_GENERIC_RESP_MSG_V01,
	.ei_array = mem_free_generic_resp_msg_data_v01_ei,
};

static struct msg_desc mem_share_svc_size_query_req_desc = {
	.max_msg_len = MEM_FREE_REQ_MAX_MSG_LEN_V01,
	.msg_id = MEM_QUERY_SIZE_REQ_MSG_V01,
	.ei_array = mem_query_size_req_msg_data_v01_ei,
};

static struct msg_desc mem_share_svc_size_query_resp_desc = {
	.max_msg_len = MEM_FREE_REQ_MAX_MSG_LEN_V01,
	.msg_id = MEM_QUERY_SIZE_RESP_MSG_V01,
	.ei_array = mem_query_size_resp_msg_data_v01_ei,
};

/*
 *  This API creates ramdump dev handlers
 *  for each of the memshare clients.
 *  These dev handlers will be used for
 *  extracting the ramdump for loaned memory
 *  segments.
 */

static int mem_share_configure_ramdump(void)
{
	char client_name[18] = "memshare_";
	char *clnt = NULL;

	msh_trace("ramdump_config: num_clients=%u (max %d)\n",
		  num_clients, MAX_CLIENTS);

	switch (num_clients) {
	case 0:
		clnt = "GPS";
		break;
	case 1:
		clnt = "FTM";
		break;
	case 2:
		clnt = "DIAG";
		break;
	default:
		pr_info("memshare: no memshare clients registered\n");
		return -EINVAL;
	}

	snprintf(client_name, 18, "memshare_%s", clnt);
	msh_trace("ramdump_config: idx=%u name=%s dev=%s\n", num_clients,
		  client_name,
		  memshare_dev[num_clients] ?
			dev_name(memshare_dev[num_clients]) : "<null>");
	if (memshare_dev[num_clients]) {
		memshare_ramdump_dev[num_clients] =
			create_ramdump_device(client_name,
				memshare_dev[num_clients]);
	} else {
		pr_err("memshare:%s: invalid memshare device\n", __func__);
		return -ENODEV;
	}
	msh_trace("ramdump_config: create_ramdump_device(%s) => %lx\n",
		  client_name,
		  (unsigned long)memshare_ramdump_dev[num_clients]);
	if (IS_ERR_OR_NULL(memshare_ramdump_dev[num_clients])) {
		pr_err("memshare: %s: Unable to create memshare ramdump device.\n",
				__func__);
		memshare_ramdump_dev[num_clients] = NULL;
		return -ENOMEM;
	}

	return 0;
}

static int check_client(int client_id, int proc, int request)
{
	int i = 0, rc;
	int found = DHMS_MEM_CLIENT_INVALID;

	msh_trace("check_client: client_id=%d proc=%d request=%d\n",
		  client_id, proc, request);

	for (i = 0; i < MAX_CLIENTS; i++) {
		if (memblock[i].client_id == client_id &&
				memblock[i].peripheral == proc) {
			found = i;
			break;
		}
	}
	if ((found == DHMS_MEM_CLIENT_INVALID) && !request) {
		pr_debug("memshare: No registered client, adding a new client\n");
		/* Add a new client */
		for (i = 0; i < MAX_CLIENTS; i++) {
			if (memblock[i].client_id == DHMS_MEM_CLIENT_INVALID) {
				msh_trace("check_client: not found, adding at idx=%d\n", i);
				memblock[i].client_id = client_id;
				memblock[i].alloted = 0;
				memblock[i].guarantee = 0;
				memblock[i].peripheral = proc;
				found = i;

				if (!memblock[i].file_created) {
					rc = mem_share_configure_ramdump();
					if (rc)
						pr_err("In %s, Cannot create ramdump for client: %d\n",
							__func__, client_id);
					else
						memblock[i].file_created = 1;
				}

				break;
			}
		}
	}

	msh_trace("check_client: -> idx=%d\n", found);
	return found;
}

void free_client(int id)
{
	msh_trace("free_client: idx=%d client=%u proc=%u phy=%pa size=%u alloted=%u guarantee=%u seq=%u\n",
		  id, memblock[id].client_id, memblock[id].peripheral,
		  &memblock[id].phy_addr, memblock[id].size,
		  memblock[id].alloted, memblock[id].guarantee,
		  memblock[id].sequence_id);
	memblock[id].phy_addr = 0;
	memblock[id].virtual_addr = 0;
	memblock[id].alloted = 0;
	memblock[id].guarantee = 0;
	memblock[id].sequence_id = -1;
	memblock[id].memory_type = MEMORY_CMA;

}

void fill_alloc_response(struct mem_alloc_generic_resp_msg_v01 *resp,
						int id, int *flag)
{
	resp->sequence_id_valid = 1;
	resp->sequence_id = memblock[id].sequence_id;
	resp->dhms_mem_alloc_addr_info_valid = 1;
	resp->dhms_mem_alloc_addr_info_len = 1;
	resp->dhms_mem_alloc_addr_info[0].phy_addr = memblock[id].phy_addr;
	resp->dhms_mem_alloc_addr_info[0].num_bytes = memblock[id].size;
	if (!*flag) {
		resp->resp.result = QMI_RESULT_SUCCESS_V01;
		resp->resp.error = QMI_ERR_NONE_V01;
	} else {
		resp->resp.result = QMI_RESULT_FAILURE_V01;
		resp->resp.error = QMI_ERR_NO_MEMORY_V01;
	}

}

void initialize_client(void)
{
	int i;

	msh_trace("initialize_client: %d client slots, DMA_ATTR_NO_KERNEL_MAPPING\n",
		  MAX_CLIENTS);

	for (i = 0; i < MAX_CLIENTS; i++) {
		memblock[i].alloted = 0;
		memblock[i].size = 0;
		memblock[i].guarantee = 0;
		memblock[i].phy_addr = 0;
		memblock[i].virtual_addr = 0;
		memblock[i].client_id = DHMS_MEM_CLIENT_INVALID;
		memblock[i].peripheral = -1;
		memblock[i].sequence_id = -1;
		memblock[i].memory_type = MEMORY_CMA;
		memblock[i].free_memory = 0;
		memblock[i].hyp_mapping = 0;
		memblock[i].file_created = 0;
	}
	dma_set_attr(DMA_ATTR_NO_KERNEL_MAPPING, &attrs);
}

/*
 *  This API initializes the ramdump segments
 *  with the physical address and size of
 *  the memshared clients. Extraction of ramdump
 *  is skipped if memshare client is not alloted
 *  This calls the ramdump api in extracting the
 *  ramdump in elf format.
 */

static int mem_share_do_ramdump(void)
{
	int i = 0, ret;
	char *client_name = NULL;

	msh_trace("ramdump: start, num_clients=%u\n", num_clients);
	for (i = 0; i < num_clients; i++) {

		struct ramdump_segment *ramdump_segments_tmp = NULL;

		switch (i) {
		case 0:
			client_name = "GPS";
			break;
		case 1:
			client_name = "FTM";
			break;
		case 2:
			client_name = "DIAG";
			break;
		default:
			pr_info("memshare: no memshare clients registered\n");
			break;
		}

		msh_trace("ramdump: idx=%d name=%s alloted=%u phy=%pa size=%u\n",
			  i, client_name, memblock[i].alloted,
			  &memblock[i].phy_addr, memblock[i].size);
		if (!memblock[i].alloted) {
			pr_err("memshare:%s memblock is not alloted\n",
			client_name);
			continue;
		}

		ramdump_segments_tmp = kcalloc(1,
			sizeof(struct ramdump_segment),
			GFP_KERNEL);
		if (!ramdump_segments_tmp)
			return -ENOMEM;

		ramdump_segments_tmp[0].size = memblock[i].size;
		ramdump_segments_tmp[0].address = memblock[i].phy_addr;

		pr_debug("memshare: %s:%s client:phy_address = %llx, size = %d\n",
		__func__, client_name,
		(unsigned long long) memblock[i].phy_addr, memblock[i].size);

		msh_trace("ramdump: do_elf_ramdump(idx=%d, phy=%pa, size=%u)\n",
			  i, &memblock[i].phy_addr, memblock[i].size);
		ret = do_elf_ramdump(memshare_ramdump_dev[i],
					ramdump_segments_tmp, 1);
		if (ret < 0) {
			pr_err("memshare: Unable to dump: %d\n", ret);
			kfree(ramdump_segments_tmp);
			return ret;
		}
		kfree(ramdump_segments_tmp);
	}
	return 0;
}

static int modem_notifier_cb(struct notifier_block *this, unsigned long code,
					void *_cmd)
{
	int i;
	int ret;
	u32 source_vmlist[2] = {VMID_HLOS, VMID_MSS_MSA};
	int dest_vmids[1] = {VMID_HLOS};
	int dest_perms[1] = {PERM_READ|PERM_WRITE|PERM_EXEC};
	struct notif_data *notifdata = NULL;

	msh_trace("notifier: code=%lu (%s) bootup_request=%llu ramdump_event=%d\n",
		  code, msh_notif_name(code),
		  (unsigned long long)bootup_request, ramdump_event);

	mutex_lock(&memsh_drv->mem_share);

	switch (code) {

	case SUBSYS_BEFORE_SHUTDOWN:
		bootup_request++;
		msh_trace("notifier: BEFORE_SHUTDOWN, bootup_request=%llu\n",
			  (unsigned long long)bootup_request);
		break;

	case SUBSYS_RAMDUMP_NOTIFICATION:
		ramdump_event = 1;
		msh_trace("notifier: RAMDUMP_NOTIFICATION, ramdump_event=1\n");
		break;

	case SUBSYS_BEFORE_POWERUP:
		msh_trace("notifier: BEFORE_POWERUP, ramdump_event=%d\n",
			  ramdump_event);
		if (_cmd) {
			notifdata = (struct notif_data *) _cmd;
		} else {
			ramdump_event = 0;
			break;
		}

		msh_trace("notifier: BEFORE_POWERUP enable_ramdump=%d ramdump_event=%d\n",
			  notifdata->enable_ramdump, ramdump_event);
		if (notifdata->enable_ramdump && ramdump_event) {
			pr_info("memshare: %s, Ramdump collection is enabled\n",
					__func__);
			ret = mem_share_do_ramdump();
			if (ret)
				pr_err("Ramdump collection failed\n");
			ramdump_event = 0;
		}
		break;

	case SUBSYS_AFTER_POWERUP:
		msh_trace("notifier: AFTER_POWERUP, bootup_request=%llu\n",
			  (unsigned long long)bootup_request);
		pr_debug("memshare: Modem has booted up\n");
		for (i = 0; i < MAX_CLIENTS; i++) {
			msh_trace("notifier: idx=%d client=%u proc=%u alloted=%u guarantee=%u client_request=%u free_memory=%d hyp=%d\n",
				  i, memblock[i].client_id, memblock[i].peripheral,
				  memblock[i].alloted, memblock[i].guarantee,
				  memblock[i].client_request, memblock[i].free_memory,
				  memblock[i].hyp_mapping);
			if (memblock[i].free_memory > 0 &&
					bootup_request >= 2) {
				memblock[i].free_memory -= 1;
				pr_debug("memshare: free_memory count: %d for clinet id: %d\n",
					memblock[i].free_memory,
					memblock[i].client_id);
			}

			if (memblock[i].free_memory == 0) {
				msh_trace("notifier: idx=%d free_memory=0, checking MPSS/guarantee/client_request/alloted\n",
					  i);
				if (memblock[i].peripheral ==
					DHMS_MEM_PROC_MPSS_V01 &&
					!memblock[i].guarantee &&
					!memblock[i].client_request &&
					memblock[i].alloted) {
					pr_debug("memshare: Freeing memory for client id: %d\n",
						memblock[i].client_id);
					ret = hyp_assign_phys(
							memblock[i].phy_addr,
							memblock[i].size,
							source_vmlist,
							2, dest_vmids,
							dest_perms, 1);
					if (ret &&
						memblock[i].hyp_mapping == 1) {
						/*
						 * This is an error case as hyp
						 * mapping was successful
						 * earlier but during unmap
						 * it lead to failure.
						 */
						pr_err("memshare: %s, failed to unmap the region\n",
							__func__);
						memblock[i].hyp_mapping = 1;
					} else {
						memblock[i].hyp_mapping = 0;
					}
					msh_trace("notifier: freeing idx=%d client=%u phy=%pa size=%u hyp_ret=%d hyp_mapping=%d\n",
						  i, memblock[i].client_id,
						  &memblock[i].phy_addr, memblock[i].size,
						  ret, memblock[i].hyp_mapping);
					dma_free_attrs(memsh_drv->dev,
						memblock[i].size,
						memblock[i].virtual_addr,
						memblock[i].phy_addr,
						&attrs);
					free_client(i);
				}
			}
		}
		bootup_request++;
		break;

	default:
		pr_debug("Memshare: code: %lu\n", code);
		break;
	}

	mutex_unlock(&memsh_drv->mem_share);
	msh_trace("notifier: done, bootup_request=%llu\n",
		  (unsigned long long)bootup_request);
	return NOTIFY_DONE;
}

static struct notifier_block nb = {
	.notifier_call = modem_notifier_cb,
};

static void shared_hyp_mapping(int client_id)
{
	int ret;
	u32 source_vmlist[1] = {VMID_HLOS};
	int dest_vmids[2] = {VMID_HLOS, VMID_MSS_MSA};
	int dest_perms[2] = {PERM_READ|PERM_WRITE,
				PERM_READ|PERM_WRITE};

	if (client_id == DHMS_MEM_CLIENT_INVALID) {
		pr_err("memshare: %s, Invalid Client\n", __func__);
		return;
	}

	msh_trace("hyp_assign: idx=%d client=%u proc=%u phy=%pa size=%u\n",
		  client_id, memblock[client_id].client_id,
		  memblock[client_id].peripheral,
		  &memblock[client_id].phy_addr, memblock[client_id].size);
	ret = hyp_assign_phys(memblock[client_id].phy_addr,
			memblock[client_id].size,
			source_vmlist, 1, dest_vmids,
			dest_perms, 2);

	if (ret != 0) {
		pr_err("memshare: hyp_assign_phys failed size=%u err=%d\n",
				memblock[client_id].size, ret);
		return;
	}
	msh_trace("hyp_assign: OK idx=%d client=%u\n", client_id,
		  memblock[client_id].client_id);
	memblock[client_id].hyp_mapping = 1;
}

static int handle_alloc_req(void *req_h, void *req, void *conn_h)
{
	struct mem_alloc_req_msg_v01 *alloc_req;
	struct mem_alloc_resp_msg_v01 alloc_resp;
	int rc = 0;

	alloc_req = (struct mem_alloc_req_msg_v01 *)req;
	msh_trace("legacy alloc_req: num_bytes=%u block_alignment_valid=%d block_alignment=%d gps_size=%u alloted=%u\n",
		  alloc_req->num_bytes, alloc_req->block_alignment_valid,
		  alloc_req->block_alignment, memblock[GPS].size,
		  memblock[GPS].alloted);
	pr_debug("%s: Received Alloc Request\n", __func__);
	pr_debug("%s: req->num_bytes = %d\n", __func__, alloc_req->num_bytes);
	mutex_lock(&memsh_drv->mem_share);
	msh_trace("legacy alloc_req: %s existing GPS block\n",
		  memblock[GPS].size ? "reusing" : "allocating");
	if (!memblock[GPS].size) {
		memset(&alloc_resp, 0, sizeof(struct mem_alloc_resp_msg_v01));
		alloc_resp.resp = QMI_RESULT_FAILURE_V01;
		rc = memshare_alloc(memsh_drv->dev, alloc_req->num_bytes,
					&memblock[GPS]);
	}
	alloc_resp.num_bytes_valid = 1;
	alloc_resp.num_bytes =  alloc_req->num_bytes;
	alloc_resp.handle_valid = 1;
	alloc_resp.handle = memblock[GPS].phy_addr;
	if (rc) {
		alloc_resp.resp = QMI_RESULT_FAILURE_V01;
		memblock[GPS].size = 0;
	} else {
		alloc_resp.resp = QMI_RESULT_SUCCESS_V01;
	}

	msh_trace("legacy alloc_resp: num_bytes=%u handle=0x%llx resp=%d alloted=%u size=%u\n",
		  alloc_resp.num_bytes,
		  (unsigned long long)alloc_resp.handle,
		  alloc_resp.resp, memblock[GPS].alloted,
		  memblock[GPS].size);

	mutex_unlock(&memsh_drv->mem_share);

	pr_debug("alloc_resp.num_bytes :%d, alloc_resp.handle :%lx, alloc_resp.mem_req_result :%lx\n",
			  alloc_resp.num_bytes,
			  (unsigned long int)alloc_resp.handle,
			  (unsigned long int)alloc_resp.resp);
	rc = qmi_send_resp_from_cb(mem_share_svc_handle, conn_h, req_h,
			&mem_share_svc_alloc_resp_desc, &alloc_resp,
			sizeof(alloc_resp));
	if (rc < 0)
		pr_err("In %s, Error sending the alloc request: %d\n",
					__func__, rc);

	return rc;
}

static int handle_alloc_generic_req(void *req_h, void *req, void *conn_h)
{
	struct mem_alloc_generic_req_msg_v01 *alloc_req;
	struct mem_alloc_generic_resp_msg_v01 *alloc_resp;
	int rc, resp = 0;
	int client_id;
	uint32_t size = 0;

	alloc_req = (struct mem_alloc_generic_req_msg_v01 *)req;
	msh_trace("alloc_generic_req: num_bytes=%u client_id=%d proc_id=%d seq=%u alloc_contig_valid=%d alloc_contig=%d align_valid=%d align=%d\n",
		  alloc_req->num_bytes, alloc_req->client_id, alloc_req->proc_id,
		  alloc_req->sequence_id, alloc_req->alloc_contiguous_valid,
		  alloc_req->alloc_contiguous, alloc_req->block_alignment_valid,
		  alloc_req->block_alignment);
	pr_debug("memshare: alloc request client id: %d proc _id: %d\n",
			alloc_req->client_id, alloc_req->proc_id);
	mutex_lock(&memsh_drv->mem_share);
	alloc_resp = kzalloc(sizeof(struct mem_alloc_generic_resp_msg_v01),
					GFP_KERNEL);
	if (!alloc_resp) {
		mutex_unlock(&memsh_drv->mem_share);
		return -ENOMEM;
	}
	alloc_resp->resp.result = QMI_RESULT_FAILURE_V01;
	alloc_resp->resp.error = QMI_ERR_NO_MEMORY_V01;
	client_id = check_client(alloc_req->client_id, alloc_req->proc_id,
								CHECK);

	if (client_id >= MAX_CLIENTS) {
		pr_err("memshare: %s client not found, requested client: %d, proc_id: %d\n",
				__func__, alloc_req->client_id,
				alloc_req->proc_id);
		kfree(alloc_resp);
		alloc_resp = NULL;
		mutex_unlock(&memsh_drv->mem_share);
		return -EINVAL;
	}

	msh_trace("alloc_generic_req: idx=%d %s\n", client_id,
		  memblock[client_id].alloted ? "already alloted" : "allocating");
	if (!memblock[client_id].alloted) {
		if (alloc_req->client_id == 1 && alloc_req->num_bytes > 0)
			size = alloc_req->num_bytes + MEMSHARE_GUARD_BYTES;
		else
			size = alloc_req->num_bytes;
		rc = memshare_alloc(memsh_drv->dev, size,
					&memblock[client_id]);
		if (rc) {
			pr_err("In %s,Unable to allocate memory for requested client\n",
							__func__);
			resp = 1;
		}
		if (!resp) {
			memblock[client_id].free_memory += 1;
			memblock[client_id].alloted = 1;
			memblock[client_id].size = alloc_req->num_bytes;
			memblock[client_id].peripheral = alloc_req->proc_id;
		}
	}
	pr_debug("memshare: In %s, free memory count for client id: %d = %d",
		__func__, memblock[client_id].client_id,
		memblock[client_id].free_memory);

	msh_trace("alloc_generic_req: idx=%d client=%u proc=%u alloted=%u size=%u free_memory=%d phy=%pa\n",
		  client_id, memblock[client_id].client_id,
		  memblock[client_id].peripheral, memblock[client_id].alloted,
		  memblock[client_id].size, memblock[client_id].free_memory,
		  &memblock[client_id].phy_addr);
	memblock[client_id].sequence_id = alloc_req->sequence_id;

	fill_alloc_response(alloc_resp, client_id, &resp);
	msh_trace("alloc_generic_resp: idx=%d seq=%u result=%d error=%d phy=0x%llx num_bytes=%u\n",
		  client_id, alloc_resp->sequence_id, alloc_resp->resp.result,
		  alloc_resp->resp.error,
		  (unsigned long long)alloc_resp->dhms_mem_alloc_addr_info[0].phy_addr,
		  alloc_resp->dhms_mem_alloc_addr_info[0].num_bytes);
	/*
	 * Perform the Hypervisor mapping in order to avoid XPU viloation
	 * to the allocated region for Modem Clients
	 */
	msh_trace("alloc_generic_req: idx=%d hyp_mapping=%d alloted=%u\n",
		  client_id, memblock[client_id].hyp_mapping,
		  memblock[client_id].alloted);
	if (!memblock[client_id].hyp_mapping &&
		memblock[client_id].alloted)
		shared_hyp_mapping(client_id);
	mutex_unlock(&memsh_drv->mem_share);
	pr_debug("memshare: alloc_resp.num_bytes :%d, alloc_resp.handle :%lx, alloc_resp.mem_req_result :%lx\n",
			  alloc_resp->dhms_mem_alloc_addr_info[0].num_bytes,
			  (unsigned long int)
			  alloc_resp->dhms_mem_alloc_addr_info[0].phy_addr,
			  (unsigned long int)alloc_resp->resp.result);
	rc = qmi_send_resp_from_cb(mem_share_svc_handle, conn_h, req_h,
			&mem_share_svc_alloc_generic_resp_desc, alloc_resp,
			sizeof(alloc_resp));

	if (rc < 0)
		pr_err("In %s, Error sending the alloc request: %d\n",
							__func__, rc);

	kfree(alloc_resp);
	alloc_resp = NULL;
	return rc;
}

static int handle_free_req(void *req_h, void *req, void *conn_h)
{
	struct mem_free_req_msg_v01 *free_req;
	struct mem_free_resp_msg_v01 free_resp;
	int rc;

	mutex_lock(&memsh_drv->mem_free);
	if (!memblock[GPS].guarantee) {
		free_req = (struct mem_free_req_msg_v01 *)req;
		msh_trace("legacy free_req: handle=0x%x gps_size=%u alloted=%u guarantee=%u\n",
			  free_req->handle, memblock[GPS].size,
			  memblock[GPS].alloted, memblock[GPS].guarantee);
		pr_debug("%s: Received Free Request\n", __func__);
		memset(&free_resp, 0, sizeof(struct mem_free_resp_msg_v01));
		pr_debug("In %s: pblk->virtual_addr :%lx, pblk->phy_addr: %lx\n,size: %d",
				__func__,
			(unsigned long int)memblock[GPS].virtual_addr,
			(unsigned long int)free_req->handle,
			memblock[GPS].size);
		msh_trace("legacy free: dma_free_coherent(size=%u virt=%lx handle=0x%x)\n",
			  memblock[GPS].size,
			  (unsigned long)memblock[GPS].virtual_addr,
			  free_req->handle);
		dma_free_coherent(memsh_drv->dev, memblock[GPS].size,
			memblock[GPS].virtual_addr,
				free_req->handle);
	}
	free_resp.resp = QMI_RESULT_SUCCESS_V01;
	msh_trace("legacy free_resp: resp=%d\n", free_resp.resp);
	mutex_unlock(&memsh_drv->mem_free);
	rc = qmi_send_resp_from_cb(mem_share_svc_handle, conn_h, req_h,
			&mem_share_svc_free_resp_desc, &free_resp,
			sizeof(free_resp));
	if (rc < 0)
		pr_err("In %s, Error sending the free request: %d\n",
					__func__, rc);

	return rc;
}

static int handle_free_generic_req(void *req_h, void *req, void *conn_h)
{
	struct mem_free_generic_req_msg_v01 *free_req;
	struct mem_free_generic_resp_msg_v01 free_resp;
	int rc;
	int flag = 0;
	uint32_t client_id;

	free_req = (struct mem_free_generic_req_msg_v01 *)req;
	msh_trace("free_generic_req: addr_len=%u client_valid=%d client=%d proc_valid=%d proc=%d\n",
		  free_req->dhms_mem_alloc_addr_info_len,
		  free_req->client_id_valid, free_req->client_id,
		  free_req->proc_id_valid, free_req->proc_id);
	pr_debug("memshare: %s: Received Free Request\n", __func__);
	mutex_lock(&memsh_drv->mem_free);
	memset(&free_resp, 0, sizeof(struct mem_free_generic_resp_msg_v01));
	free_resp.resp.error = QMI_ERR_INTERNAL_V01;
	free_resp.resp.result = QMI_RESULT_FAILURE_V01;
	pr_debug("memshare: Client id: %d proc id: %d\n", free_req->client_id,
				free_req->proc_id);
	client_id = check_client(free_req->client_id, free_req->proc_id, FREE);
	if (client_id == DHMS_MEM_CLIENT_INVALID) {
		pr_err("In %s, Invalid client request to free memory\n",
					__func__);
		flag = 1;
	} else if (!memblock[client_id].guarantee &&
				memblock[client_id].client_request &&
					memblock[client_id].alloted) {
		pr_debug("memshare: %s: client_id:%d, size: %d",
				__func__, client_id,
				memblock[client_id].size);
		msh_trace("free_generic_req: freeing idx=%d client=%u phy=%pa size=%u\n",
			  client_id, memblock[client_id].client_id,
			  &memblock[client_id].phy_addr, memblock[client_id].size);
		dma_free_attrs(memsh_drv->dev, memblock[client_id].size,
			memblock[client_id].virtual_addr,
			memblock[client_id].phy_addr,
			&attrs);
		free_client(client_id);
	} else {
		pr_err("In %s, Request came for a guaranteed client(client_id: %d) cannot free up the memory\n",
						__func__, client_id);
	}

	msh_trace("free_generic_req: idx=%d flag=%d\n", client_id, flag);
	if (flag) {
		free_resp.resp.result = QMI_RESULT_FAILURE_V01;
		free_resp.resp.error = QMI_ERR_INVALID_ID_V01;
	} else {
		free_resp.resp.result = QMI_RESULT_SUCCESS_V01;
		free_resp.resp.error = QMI_ERR_NONE_V01;
	}

	mutex_unlock(&memsh_drv->mem_free);
	rc = qmi_send_resp_from_cb(mem_share_svc_handle, conn_h, req_h,
		&mem_share_svc_free_generic_resp_desc, &free_resp,
		sizeof(free_resp));

	if (rc < 0)
		pr_err("In %s, Error sending the free request: %d\n",
					__func__, rc);

	return rc;
}

static int handle_query_size_req(void *req_h, void *req, void *conn_h)
{
	int rc, client_id;
	struct mem_query_size_req_msg_v01 *query_req;
	struct mem_query_size_rsp_msg_v01 *query_resp;

	query_req = (struct mem_query_size_req_msg_v01 *)req;
	msh_trace("query_size_req: client=%d proc_valid=%d proc=%d\n",
		  query_req->client_id, query_req->proc_id_valid,
		  query_req->proc_id);
	mutex_lock(&memsh_drv->mem_share);
	query_resp = kzalloc(sizeof(struct mem_query_size_rsp_msg_v01),
					GFP_KERNEL);
	if (!query_resp) {
		mutex_unlock(&memsh_drv->mem_share);
		return -ENOMEM;
	}
	pr_debug("memshare: query request client id: %d proc _id: %d\n",
		query_req->client_id, query_req->proc_id);
	client_id = check_client(query_req->client_id, query_req->proc_id,
								CHECK);

	if (client_id >= MAX_CLIENTS) {
		pr_err("memshare: %s client not found, requested client: %d, proc_id: %d\n",
				__func__, query_req->client_id,
				query_req->proc_id);
		kfree(query_resp);
		query_resp = NULL;
		return -EINVAL;
	}

	msh_trace("query_size_req: idx=%d size=%u alloted=%u\n",
		  client_id, memblock[client_id].size,
		  memblock[client_id].alloted);
	if (memblock[client_id].size) {
		query_resp->size_valid = 1;
		query_resp->size = memblock[client_id].size;
	} else {
		query_resp->size_valid = 1;
		query_resp->size = 0;
	}
	query_resp->resp.result = QMI_RESULT_SUCCESS_V01;
	query_resp->resp.error = QMI_ERR_NONE_V01;
	mutex_unlock(&memsh_drv->mem_share);

	pr_debug("memshare: query_resp.size :%d, alloc_resp.mem_req_result :%lx\n",
			  query_resp->size,
			  (unsigned long int)query_resp->resp.result);
	rc = qmi_send_resp_from_cb(mem_share_svc_handle, conn_h, req_h,
			&mem_share_svc_size_query_resp_desc, query_resp,
			sizeof(query_resp));

	if (rc < 0)
		pr_err("In %s, Error sending the query request: %d\n",
							__func__, rc);

	kfree(query_resp);
	query_resp = NULL;
	return rc;
}

static int mem_share_svc_connect_cb(struct qmi_handle *handle,
			       void *conn_h)
{
	if (mem_share_svc_handle != handle || !conn_h)
		return -EINVAL;

	return 0;
}

static int mem_share_svc_disconnect_cb(struct qmi_handle *handle,
				  void *conn_h)
{
	if (mem_share_svc_handle != handle || !conn_h)
		return -EINVAL;

	return 0;
}

static int mem_share_svc_req_desc_cb(unsigned int msg_id,
				struct msg_desc **req_desc)
{
	int rc;

	pr_debug("memshare: In %s\n", __func__);
	switch (msg_id) {
	case MEM_ALLOC_REQ_MSG_V01:
		*req_desc = &mem_share_svc_alloc_req_desc;
		rc = sizeof(struct mem_alloc_req_msg_v01);
		break;

	case MEM_FREE_REQ_MSG_V01:
		*req_desc = &mem_share_svc_free_req_desc;
		rc = sizeof(struct mem_free_req_msg_v01);
		break;

	case MEM_ALLOC_GENERIC_REQ_MSG_V01:
		*req_desc = &mem_share_svc_alloc_generic_req_desc;
		rc = sizeof(struct mem_alloc_generic_req_msg_v01);
		break;

	case MEM_FREE_GENERIC_REQ_MSG_V01:
		*req_desc = &mem_share_svc_free_generic_req_desc;
		rc = sizeof(struct mem_free_generic_req_msg_v01);
		break;

	case MEM_QUERY_SIZE_REQ_MSG_V01:
		*req_desc = &mem_share_svc_size_query_req_desc;
		rc = sizeof(struct mem_query_size_req_msg_v01);
		break;

	default:
		rc = -ENOTSUPP;
		break;
	}
	return rc;
}

static int mem_share_svc_req_cb(struct qmi_handle *handle, void *conn_h,
			void *req_h, unsigned int msg_id, void *req)
{
	int rc;

	pr_debug("memshare: In %s\n", __func__);
	if (mem_share_svc_handle != handle || !conn_h)
		return -EINVAL;

	msh_trace("qmi_req: msg_id=0x%x handle=%lx conn=%lx req=%lx\n",
		  msg_id, (unsigned long)handle, (unsigned long)conn_h,
		  (unsigned long)req_h);
	switch (msg_id) {
	case MEM_ALLOC_REQ_MSG_V01:
		rc = handle_alloc_req(req_h, req, conn_h);
		break;

	case MEM_FREE_REQ_MSG_V01:
		rc = handle_free_req(req_h, req, conn_h);
		break;

	case MEM_ALLOC_GENERIC_REQ_MSG_V01:
		rc = handle_alloc_generic_req(req_h, req, conn_h);
		break;

	case MEM_FREE_GENERIC_REQ_MSG_V01:
		rc = handle_free_generic_req(req_h, req, conn_h);
		break;

	case MEM_QUERY_SIZE_REQ_MSG_V01:
		rc = handle_query_size_req(req_h, req, conn_h);
		break;

	default:
		rc = -ENOTSUPP;
		break;
	}
	return rc;
}

static void mem_share_svc_recv_msg(struct work_struct *work)
{
	int rc;

	pr_debug("memshare: In %s\n", __func__);
	do {
		pr_debug("%s: Notified about a Receive Event", __func__);
	} while ((rc = qmi_recv_msg(mem_share_svc_handle)) == 0);

	msh_trace("recv_msg: done, rc=%d\n", rc);

	if (rc != -ENOMSG)
		pr_err("%s: Error receiving message\n", __func__);
}

static void qmi_mem_share_svc_ntfy(struct qmi_handle *handle,
		enum qmi_event_type event, void *priv)
{
	pr_debug("memshare: In %s\n", __func__);
	switch (event) {
	case QMI_RECV_MSG:
		msh_trace("qmi_ntfy: RECV_MSG, queueing receive work\n");
		queue_delayed_work(mem_share_svc_workqueue,
				   &work_recv_msg, 0);
		break;
	default:
		break;
	}
}

static struct qmi_svc_ops_options mem_share_svc_ops_options = {
	.version = 1,
	.service_id = MEM_SHARE_SERVICE_SVC_ID,
	.service_vers = MEM_SHARE_SERVICE_VERS,
	.service_ins = MEM_SHARE_SERVICE_INS_ID,
	.connect_cb = mem_share_svc_connect_cb,
	.disconnect_cb = mem_share_svc_disconnect_cb,
	.req_desc_cb = mem_share_svc_req_desc_cb,
	.req_cb = mem_share_svc_req_cb,
};

int memshare_alloc(struct device *dev,
					unsigned int block_size,
					struct mem_blocks *pblk)
{

	int ret;

	pr_debug("%s: memshare_alloc called", __func__);
	if (!pblk) {
		pr_err("%s: Failed to alloc\n", __func__);
		return -ENOMEM;
	}

	msh_trace("dma_alloc: client=%u proc=%u size=%u dev=%s\n",
		  pblk->client_id, pblk->peripheral, block_size,
		  dev_name(dev));
	pblk->virtual_addr = dma_alloc_attrs(dev, block_size,
						&pblk->phy_addr, GFP_KERNEL,
						&attrs);
	if (pblk->virtual_addr == NULL) {
		pr_err("allocation failed, %d\n", block_size);
		ret = -ENOMEM;
		return ret;
	}

	msh_trace("dma_alloc: client=%u size=%u => phy=%pa cookie=%lx\n",
		  pblk->client_id, block_size, &pblk->phy_addr,
		  (unsigned long)pblk->virtual_addr);
	pr_debug("pblk->phy_addr :%lx, pblk->virtual_addr %lx\n",
		  (unsigned long int)pblk->phy_addr,
		  (unsigned long int)pblk->virtual_addr);
	return 0;
}

static void memshare_init_worker(struct work_struct *work)
{
	int rc;

	msh_trace("init_worker: start\n");

	mem_share_svc_workqueue =
		create_singlethread_workqueue("mem_share_svc");
	msh_trace("init_worker: workqueue=%lx\n",
		  (unsigned long)mem_share_svc_workqueue);
	if (!mem_share_svc_workqueue)
		return;

	mem_share_svc_handle = qmi_handle_create(qmi_mem_share_svc_ntfy, NULL);
	msh_trace("init_worker: qmi_handle=%lx\n",
		  (unsigned long)mem_share_svc_handle);
	if (!mem_share_svc_handle) {
		pr_err("%s: Creating mem_share_svc qmi handle failed\n",
			__func__);
		destroy_workqueue(mem_share_svc_workqueue);
		return;
	}
	rc = qmi_svc_register(mem_share_svc_handle, &mem_share_svc_ops_options);
	if (rc < 0) {
		pr_err("%s: Registering mem share svc failed %d\n",
			__func__, rc);
		qmi_handle_destroy(mem_share_svc_handle);
		destroy_workqueue(mem_share_svc_workqueue);
		return;
	}
	msh_trace("init_worker: QMI service registered, rc=%d\n", rc);
	pr_debug("memshare: memshare_init successful\n");
}

static int memshare_child_probe(struct platform_device *pdev)
{
	int rc;
	uint32_t size, client_id;
	const char *name;
	struct memshare_child *drv;

	drv = devm_kzalloc(&pdev->dev, sizeof(struct memshare_child),
							GFP_KERNEL);

	if (!drv) {
		pr_err("Unable to allocate memory to driver\n");
		return -ENOMEM;
	}

	drv->dev = &pdev->dev;
	memsh_child = drv;
	msh_trace("child_probe: node=%s\n",
		  of_node_full_name(pdev->dev.of_node));
	platform_set_drvdata(pdev, memsh_child);

	rc = of_property_read_u32(pdev->dev.of_node, "qcom,peripheral-size",
						&size);
	if (rc) {
		pr_err("In %s, Error reading size of clients, rc: %d\n",
				__func__, rc);
		return rc;
	}

	rc = of_property_read_u32(pdev->dev.of_node, "qcom,client-id",
						&client_id);
	if (rc) {
		pr_err("In %s, Error reading client id, rc: %d\n",
				__func__, rc);
		return rc;
	}

	memblock[num_clients].guarantee = of_property_read_bool(
							pdev->dev.of_node,
							"qcom,allocate-boot-time");

	memblock[num_clients].client_request = of_property_read_bool(
							pdev->dev.of_node,
							"qcom,allocate-on-request");

	rc = of_property_read_string(pdev->dev.of_node, "label",
						&name);
	if (rc) {
		pr_err("In %s, Error reading peripheral info for client, rc: %d\n",
					__func__, rc);
		return rc;
	}

	if (strcmp(name, "modem") == 0)
		memblock[num_clients].peripheral = DHMS_MEM_PROC_MPSS_V01;
	else if (strcmp(name, "adsp") == 0)
		memblock[num_clients].peripheral = DHMS_MEM_PROC_ADSP_V01;
	else if (strcmp(name, "wcnss") == 0)
		memblock[num_clients].peripheral = DHMS_MEM_PROC_WCNSS_V01;

	memblock[num_clients].size = size;
	memblock[num_clients].client_id = client_id;
	msh_trace("child_probe: idx=%u client=%u proc=%u size=%u guarantee=%u client_request=%u label=%s\n",
		  num_clients, memblock[num_clients].client_id,
		  memblock[num_clients].peripheral, memblock[num_clients].size,
		  memblock[num_clients].guarantee,
		  memblock[num_clients].client_request, name);

  /*
   *	Memshare allocation for guaranteed clients
   */
	msh_trace("child_probe: boot-time alloc idx=%u size=%u guard=%d\n",
		  num_clients, size, client_id == 1);
	if (memblock[num_clients].guarantee && size > 0) {
		if (client_id == 1)
			size += MEMSHARE_GUARD_BYTES;
		rc = memshare_alloc(memsh_child->dev,
				size,
				&memblock[num_clients]);
		if (rc) {
			pr_err("In %s, Unable to allocate memory for guaranteed clients, rc: %d\n",
							__func__, rc);
			return rc;
		}
		memblock[num_clients].alloted = 1;
		shared_hyp_mapping(num_clients);
		msh_trace("child_probe: boot-time block idx=%u alloted=%u phy=%pa\n",
			  num_clients, memblock[num_clients].alloted,
			  &memblock[num_clients].phy_addr);
	}

	/*
	 *  call for creating ramdump dev handlers for
	 *  memshare clients
	 */

	memshare_dev[num_clients] = &pdev->dev;
	msh_trace("child_probe: ramdump dev idx=%u = %s\n", num_clients,
		  dev_name(memshare_dev[num_clients]));

	if (!memblock[num_clients].file_created) {
		rc = mem_share_configure_ramdump();
		if (rc)
			pr_err("In %s, cannot collect dumps for client id: %d\n",
					__func__,
					memblock[num_clients].client_id);
		else
			memblock[num_clients].file_created = 1;
	}

	msh_trace("child_probe: done idx=%u, num_clients=%u\n",
		  num_clients, num_clients + 1);
	num_clients++;

	return 0;
}

static int memshare_probe(struct platform_device *pdev)
{
	int rc;
	struct memshare_driver *drv;

	msh_trace("probe: node=%s\n", of_node_full_name(pdev->dev.of_node));
	drv = devm_kzalloc(&pdev->dev, sizeof(struct memshare_driver),
							GFP_KERNEL);

	if (!drv) {
		pr_err("Unable to allocate memory to driver\n");
		return -ENOMEM;
	}

	/* Memory allocation has been done successfully */
	mutex_init(&drv->mem_free);
	mutex_init(&drv->mem_share);

	INIT_WORK(&drv->memshare_init_work, memshare_init_worker);
	schedule_work(&drv->memshare_init_work);
	msh_trace("probe: QMI init work scheduled\n");

	drv->dev = &pdev->dev;
	memsh_drv = drv;
	platform_set_drvdata(pdev, memsh_drv);
	initialize_client();
	num_clients = 0;

	rc = of_platform_populate(pdev->dev.of_node, NULL, NULL, &pdev->dev);
	msh_trace("probe: of_platform_populate rc=%d\n", rc);

	if (rc) {
		pr_err("In %s, error populating the devices\n", __func__);
		return rc;
	}

	subsys_notif_register_notifier("modem", &nb);
	msh_trace("probe: modem notifier registered (return value ignored)\n");
	msh_trace("probe: success\n");

	return 0;
}

static int memshare_remove(struct platform_device *pdev)
{
	msh_trace("remove: start\n");
	if (!memsh_drv)
		return 0;

	qmi_svc_unregister(mem_share_svc_handle);
	flush_workqueue(mem_share_svc_workqueue);
	qmi_handle_destroy(mem_share_svc_handle);
	destroy_workqueue(mem_share_svc_workqueue);
	msh_trace("remove: done\n");

	return 0;
}

static int memshare_child_remove(struct platform_device *pdev)
{
	msh_trace("child_remove: node=%s\n",
		  of_node_full_name(pdev->dev.of_node));
	if (!memsh_child)
		return 0;

	return 0;
}

static struct of_device_id memshare_match_table[] = {
	{
		.compatible = "qcom,memshare",
	},
	{}
};

static struct of_device_id memshare_match_table1[] = {
	{
		.compatible = "qcom,memshare-peripheral",
	},
	{}
};


static struct platform_driver memshare_pdriver = {
	.probe          = memshare_probe,
	.remove         = memshare_remove,
	.driver = {
		.name   = MEMSHARE_DEV_NAME,
		.owner  = THIS_MODULE,
		.of_match_table = memshare_match_table,
	},
};

static struct platform_driver memshare_pchild = {
	.probe          = memshare_child_probe,
	.remove         = memshare_child_remove,
	.driver = {
		.name   = MEMSHARE_CHILD_DEV_NAME,
		.owner  = THIS_MODULE,
		.of_match_table = memshare_match_table1,
	},
};

module_platform_driver(memshare_pdriver);
module_platform_driver(memshare_pchild);

MODULE_DESCRIPTION("Mem Share QMI Service Driver");
MODULE_LICENSE("GPL v2");
