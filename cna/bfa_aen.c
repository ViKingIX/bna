/*
 * Copyright (c) 2005-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corporation.
 * All rights reserved
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */
#include "aen/bfa_aen.h"
#include "cs/bfa_cs.h"

#define	BFA_TRC_AEN_AEN	1

BFA_TRC_FILE(AEN, AEN);

int bfa_aen_max_cfg_entry = BFA_AEN_MAX_ENTRY;

/**
 * Assumes bfad has already allocated memory for this aen instance
 */
int
bfa_aen_init(struct bfa_aen_s *aen, struct bfa_trc_mod_s *trcmod, void *bfad,
		int bfad_num, void (*aen_cb_notify)(void *),
		void (*gettimeofday)(struct bfa_timeval_s *))
{
	aen->trcmod = trcmod;
	aen->bfad = bfad;
	aen->write_index = 0;
	aen->read_index = 0;
	aen->bfad_num = bfad_num;
	aen->seq_num = 0;
	aen->aen_cb_notify = aen_cb_notify;
	aen->gettimeofday = gettimeofday;
	aen->max_entry = bfa_aen_get_max_cfg_entry();
	bfa_os_memset(aen->app_ri, 0, sizeof(aen->app_ri));

	if (aen->max_entry < 2) {
		/*
		 * Log this error
		 */
		bfa_trc(aen, aen->max_entry);
		bfa_assert(0);
		return -1;
	}

	return 0;
}

/**
 * BFA/FCS/OS driver API for posting an Async Event
 */
void
bfa_aen_post(struct bfa_aen_s *aen, enum bfa_aen_category aen_category,
	     int aen_type, union bfa_aen_data_u *aen_data)
{
	int i, count;
	int write_index = aen->write_index;

	aen->gettimeofday(&aen->list[write_index].aen_tv);
	aen->list[write_index].bfad_num = aen->bfad_num;
	aen->list[write_index].seq_num = ++aen->seq_num;
	aen->list[write_index].aen_category = aen_category;
	aen->list[write_index].aen_type = aen_type;
	if (aen_data) {
		memcpy(&aen->list[write_index].aen_data, aen_data,
		       sizeof(union bfa_aen_data_u));
	}

	switch (aen_category) {
	case BFA_AEN_CAT_ADAPTER:
	case BFA_AEN_CAT_PORT:
	case BFA_AEN_CAT_LPORT:
	case BFA_AEN_CAT_RPORT:
	case BFA_AEN_CAT_ITNIM:
	case BFA_AEN_CAT_TIN:
	case BFA_AEN_CAT_IPFC:
	case BFA_AEN_CAT_AUDIT:
	case BFA_AEN_CAT_IOC:
	case BFA_AEN_CAT_ETHPORT:
	case BFA_AEN_CAT_TEAM:
	case BFA_AEN_CAT_VLAN:
		break;
	default:
		bfa_trc(aen, aen_category);
		bfa_assert(0);
	}

	/* Now put in the list */
	aen->write_index = (write_index + 1) % aen->max_entry;

	if (aen->write_index == aen->read_index) {
		// Overwrite condition, update master ri
		aen->read_index = (aen->read_index + 1) % aen->max_entry;
		// and update ri of each app
		for (i = 0; i < BFA_AEN_MAX_APP; i++) {
			if (aen->write_index == aen->app_ri[i])
				aen->app_ri[i] = aen->read_index;
		}
	}

	/* And let OS driver(the consumer) know */
	aen->aen_cb_notify(aen->bfad);

	bfa_trc(aen, aen_category);
	count = ((aen->write_index + aen->max_entry) - aen->read_index) %
			aen->max_entry;
	bfa_trc(aen, count);
}

/**
 * After each event read, this must be called to update master read_index
 * app_ri[] values are not updated here
 */
void
bfa_aen_update_ri(struct bfa_aen_s *aen)
{
	int i, ri_min;
	int wi = aen->write_index;
	int ri = aen->read_index;
	int ri_arr[BFA_AEN_MAX_APP];

	memcpy(ri_arr, aen->app_ri, sizeof(ri_arr));

	// First level up all values
	if (ri > wi) {
		for (i = 0; i < BFA_AEN_MAX_APP; i++) {
			if (ri_arr[i] < wi)
				ri_arr[i] += aen->max_entry;
		}
	}

	// Now find min
	for (i = 0, ri_min = ri_arr[i]; i < BFA_AEN_MAX_APP; i++) {
		if (ri_arr[i] < ri_min)
			ri_min = ri_arr[i];
	}

	// And then restore all values
	if (ri > wi) {
		for (i = 0; i < BFA_AEN_MAX_APP; i++) {
			ri_arr[i] %= aen->max_entry;
		}
	}

	aen->read_index = (ri_min % aen->max_entry);
}

/**
 * OS driver API for fetching an Event from the list
 */
bfa_status_t
bfa_aen_fetch(struct bfa_aen_s *aen, struct bfa_aen_entry_s *aen_entry,
		int entry_req, enum bfa_aen_app  app_id, int *entry_ret)
{
	int fetch_count = 0;
	int tmp_ri;

	bfa_trc(aen, app_id);

	bfa_assert(aen_entry);
	bfa_assert((app_id < BFA_AEN_MAX_APP) && (app_id >= bfa_aen_app_bcu));

	tmp_ri = aen->app_ri[app_id];
	while ((entry_req - fetch_count) > 0) {
		if (aen->write_index == tmp_ri) {
			// First update this app's ri with this tmp_ri
			aen->app_ri[app_id] = tmp_ri;
			// And then update master ri
			bfa_aen_update_ri(aen);
			*entry_ret = fetch_count;
			return BFA_STATUS_OK;
		} else {
			memcpy(&aen_entry[fetch_count],
			       &aen->list[tmp_ri],
			       sizeof(struct bfa_aen_entry_s));
			tmp_ri = (tmp_ri + 1) % aen->max_entry;
			fetch_count++;
		}
	}
	// First update this app's ri with this tmp_ri
	aen->app_ri[app_id] = tmp_ri;
	// And then update master ri
	bfa_aen_update_ri(aen);
	*entry_ret = fetch_count;

	return BFA_STATUS_OK;
}

int
bfa_aen_get_inst(struct bfa_aen_s *aen)
{
	return aen->bfad_num;
}

static inline uint8_t
n2b(uint8_t n)
{
	if (n <= 0x09)
		return (n + '0');
	else
		return (n - 0x0a + 'a');
}

char *
convert2str(char *ptr, uint8_t *byte, int i)
{

	bfa_assert(ptr);

	*--ptr = '\0';

	while (i > 0) {
		i--;
		*--ptr = n2b(byte[i] & 0x0f);
		*--ptr = n2b((byte[i] >> 4) & 0x0f);
		if (i > 0)
			*--ptr = ':';
	}

	return ptr;
}

char *
wwn2str(char *buf, int bufsize, uint64_t wwn)
{
	char *ptr;
	union {
		wwn_t	wwn;
		uint8_t	byte[sizeof(wwn_t)];
	} w;

	w.wwn = wwn;
	ptr = buf + bufsize;
	return convert2str(ptr, w.byte, sizeof(wwn_t));
}

char *
mac2str(char *buf, int bufsize, mac_t mac)
{
	char *ptr;
	union {
		mac_t	mac;
		uint8_t	byte[sizeof(mac_t)];
	} m;

	m.mac = mac;
	ptr = buf + bufsize;
	return convert2str(ptr, m.byte, sizeof(mac_t));
}
