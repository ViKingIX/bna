/*
 * Copyright (c) 2010-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corporation.
 * All rights reserved
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */

/**
 * @file bfa_cs.h BFA common services
 */

#ifndef __BFA_CS_H__
#define __BFA_CS_H__

#include <bfa_os_inc.h>
#include "bfa_q.h"

/**
 * BFA TRC
 */

#ifndef BFA_TRC_MAX
#define BFA_TRC_MAX	(4 * 1024)
#endif

#ifndef BFA_TRC_TS
#define BFA_TRC_TS(_trcm)	((_trcm)->ticks ++)
#endif

struct bfa_trc_s {
#ifdef __BIGENDIAN
	uint16_t	fileno;
	uint16_t	line;
#else
	uint16_t	line;
	uint16_t	fileno;
#endif
	uint32_t	timestamp;
	union {
		struct {
			uint32_t	rsvd;
			uint32_t	u32;
		} u32;
		uint64_t	u64;
	} data;
};
typedef struct bfa_trc_s bfa_trc_t;

/**
 * @brief Embed this in main driver structure
 */
struct bfa_trc_mod_s {
	uint32_t	head;
	uint32_t	tail;
	uint32_t	ntrc;
	uint32_t	stopped;
	uint32_t	ticks;
	uint32_t	rsvd[3];
	struct bfa_trc_s trc[BFA_TRC_MAX];
};
typedef struct bfa_trc_mod_s bfa_trc_mod_t;

/**
 * @brief separate modules that are traced
 */
enum {
	BFA_TRC_FW   = 1,	/* !< firmware modules */
	BFA_TRC_HAL  = 2,	/* !< BFA modules */
	BFA_TRC_FCS  = 3,	/* !< BFA FCS modules */
	BFA_TRC_LDRV = 4,	/* !< Linux driver modules */
	BFA_TRC_SDRV = 5,	/* !< Solaris driver modules */
	BFA_TRC_VDRV = 6,	/* !< vmware driver modules */
	BFA_TRC_WDRV = 7,	/* !< windows driver modules */
	BFA_TRC_AEN  = 8,	/* !< AEN module */
	BFA_TRC_BIOS = 9,	/* !< bios driver modules */
	BFA_TRC_EFI  = 10,	/* !< EFI driver modules */
	BNA_TRC_WDRV = 11,	/* !< BNA windows driver modules */
	BNA_TRC_VDRV = 12,	/* !< BNA vmware driver modules */
	BNA_TRC_SDRV = 13,	/* !< BNA Solaris driver modules */
	BNA_TRC_LDRV = 14,	/* !< BNA Linux driver modules */
	BNA_TRC_HAL  = 15,	/* !< BNA modules */
	BFA_TRC_CNA  = 16,	/* !< Common modules */
	BNA_TRC_IMDRV = 17	/* !< BNA windows intermediate driver modules */
};
#define BFA_TRC_MOD_SH	10
#define BFA_TRC_MOD(__mod)	((BFA_TRC_ ## __mod) << BFA_TRC_MOD_SH)

/**
 * @brief
 * Define a new tracing file (module). Module should match one defined above.
 */
#define BFA_TRC_FILE(__mod, __submod)					\
	static int __trc_fileno = ((BFA_TRC_ ## __mod ## _ ## __submod) | \
						 BFA_TRC_MOD(__mod))

/**
 * @brief Trace a word
 */
#define bfa_trc32(_trcp, _data)	\
	__bfa_trc((_trcp)->trcmod, __trc_fileno, __LINE__, (uint32_t)_data)

/**
 * @brief Trace a long word (64 bits)
 */
#ifndef BFA_BOOT_BUILD
#define bfa_trc(_trcp, _data)	\
	__bfa_trc((_trcp)->trcmod, __trc_fileno, __LINE__, (uint64_t)_data)
#else
void bfa_boot_trc(struct bfa_trc_mod_s *trcmod, uint16_t fileno,
			uint16_t line, uint32_t data);
#define bfa_trc(_trcp, _data)	\
	bfa_boot_trc((_trcp)->trcmod, __trc_fileno, __LINE__, (uint32_t)_data)
#endif

/**
 * @brief Tracing initialize or restart.
 */
static inline void
bfa_trc_init(struct bfa_trc_mod_s *trcm)
{
	trcm->head = trcm->tail = trcm->stopped = 0;
	trcm->ntrc = BFA_TRC_MAX;
}

/**
 * @brief Stop tracing.
 */
static inline void
bfa_trc_stop(struct bfa_trc_mod_s *trcm)
{
	trcm->stopped = 1;
}

#ifdef FWTRC
extern void dc_flush(void *data);
#else
#define dc_flush(data)
#endif

/**
 * @brief Internal - use bfa_trc() or bfa_trcl()
 */
static inline void
__bfa_trc(struct bfa_trc_mod_s *trcm, int fileno, int line, uint64_t data)
{
	int		tail = trcm->tail;
	struct bfa_trc_s 	*trc = &trcm->trc[tail];

	if (trcm->stopped)
		return;

	trc->fileno = (uint16_t) fileno;
	trc->line = (uint16_t) line;
	trc->data.u64 = data;
	trc->timestamp = BFA_TRC_TS(trcm);
	dc_flush(trc);

	trcm->tail = (trcm->tail + 1) & (BFA_TRC_MAX - 1);
	if (trcm->tail == trcm->head)
		trcm->head = (trcm->head + 1) & (BFA_TRC_MAX - 1);
	dc_flush(trcm);
}

/**
 * @brief Internal - use bfa_trc() or bfa_trcl()
 */
static inline void
__bfa_trc32(struct bfa_trc_mod_s *trcm, int fileno, int line, uint32_t data)
{
	int		tail = trcm->tail;
	struct bfa_trc_s *trc = &trcm->trc[tail];

	if (trcm->stopped)
		return;

	trc->fileno = (uint16_t) fileno;
	trc->line = (uint16_t) line;
	trc->data.u32.u32 = data;
	trc->timestamp = BFA_TRC_TS(trcm);
	dc_flush(trc);

	trcm->tail = (trcm->tail + 1) & (BFA_TRC_MAX - 1);
	if (trcm->tail == trcm->head)
		trcm->head = (trcm->head + 1) & (BFA_TRC_MAX - 1);
	dc_flush(trcm);
}

#ifndef BFA_PERF_BUILD
#define bfa_trc_fp(_trcp, _data)	bfa_trc(_trcp, _data)
#else
#define bfa_trc_fp(_trcp, _data)
#endif

/**
 * @ BFA checksum utilities
 */
static inline uint32_t
bfa_checksum_u32(uint32_t *buf, int sz)
{
	int		i, m = sz >> 2;
	uint32_t	sum = 0;

	for (i = 0; i < m; i++)
		sum ^= buf[i];

	return (sum);
}

static inline uint16_t
bfa_checksum_u16(uint16_t *buf, int sz)
{
	int		i, m = sz >> 1;
	uint16_t	sum = 0;

	for (i = 0; i < m; i++)
		sum ^= buf[i];

	return (sum);
}

static inline uint8_t
bfa_checksum_u8(uint8_t *buf, int sz)
{
	int		i;
	uint8_t		sum = 0;

	for (i = 0; i < sz; i++)
		sum ^= buf[i];

	return (sum);
}

/**
 * @ BFA debug interfaces
 */

#define bfa_assert(__cond)	do {					\
	if (!(__cond)) 							\
		bfa_panic(__LINE__, __FILE__, #__cond);			\
} while (0)

#ifndef BFA_BOOT_BUILD
#define bfa_sm_fault(__mod, __event)	do {				\
	bfa_trc(__mod, (((uint32_t)0xDEAD << 16) | __event));		\
	bfa_sm_panic((__mod)->logm, __LINE__, __FILE__, __event);	\
} while (0)
#else
#define bfa_sm_fault(__mod, __event)	do {				\
	bfa_sm_panic((__mod)->logm, __LINE__, __FILE__, __event);	\
} while (0)
#endif

#ifndef BFA_PERF_BUILD
#define bfa_assert_fp(__cond)	bfa_assert(__cond)
#else
#define bfa_assert_fp(__cond)
#endif

struct bfa_log_mod_s;
char *bfa_file_basename(char *file);
void bfa_panic(int line, char *file, char *panicstr);
void bfa_sm_panic(struct bfa_log_mod_s *logm, int line, char *file, int event);

/**
 * @ BFA hash table
 */

#define bfa_ht_t struct bfa_q_s *

/**
 * @brief
 * Declares and defines a function that inits a hash table
 *
 * @param[in] name	prefix name for the hash init functions.
 *
 * @return
 * NA
 *
 * Special Considerations:
 *
 * @note
 * When the module specific hash_init function is called (for example,
 * xyz_hash_init()), the ht pointer should be point to a block of memory which
 * is (sizeof(bfa_q_t) * n_elem)
 */
#define BFA_HT_INIT(name)					\
	int name##_hash_init(bfa_ht_t ht, int n_elem);		\
								\
	int name##_hash_init(bfa_ht_t ht, int n_elem)		\
	{							\
		int i;						\
								\
		if (n_elem <= 0)				\
			return (-1);				\
								\
		if (ht == NULL)					\
			return (-1);				\
								\
		for (i = 0; i < n_elem; i++)			\
			bfa_q_init(&ht[i]);			\
								\
		return 0;					\
	}							\

/**
 * @brief
 * Declares and defines a function that adds an entry to a hash table
 *
 * @param[in] name	prefix name for the hash init functions.
 * @param[in] e_type	hash entry type
 * @param[in] h_fn	is a macro or function that hashes an element to
 * 			an integer. This hash function should return a
 * 			value that is less than n_elem value (that was used
 * 			in hash_init() call).
 *
 * @return
 * NA
 *
 * Special Considerations:
 *
 * @note
 */
#define BFA_HT_ADD(name, e_type, h_fn)					\
	void name##_hash_add(bfa_ht_t ht_hdl, e_type entry);		\
									\
	void								\
	name##_hash_add(bfa_ht_t ht_hdl, e_type entry)			\
	{								\
		int h_val;						\
									\
		bfa_assert(ht_hdl != NULL && entry != NULL);		\
									\
		/* we want to enforce this */				\
		/* bfa_assert(name##_hash_lookup(ht_hdl, entry) == NULL); */\
									\
	h_val = h_fn(entry);						\
	bfa_assert(h_val >= 0);						\
	bfa_q_qe_init(entry);						\
	bfa_q_enq_head(&ht_hdl[h_val], entry);				\
									\
	}								\

/**
 * @brief
 * Declares and defines a function that removes an entry from a hash table
 *
 * @param[in] name	prefix name for the hash init functions.
 * @param[in] e_type	hash entry type
 *
 * @return
 * NA
 *
 * Special Considerations:
 *
 * @note
 */
#define BFA_HT_REMOVE(name, e_type)					\
	void name##_hash_remove(bfa_ht_t ht_hdl, e_type entry);		\
									\
	void								\
	name##_hash_remove(bfa_ht_t ht_hdl, e_type entry)		\
	{								\
		bfa_assert(ht_hdl != NULL && entry != NULL);		\
									\
		/* we want to enforce this */				\
		bfa_assert(name##_hash_lookup(ht_hdl, entry) != NULL);	\
		bfa_q_qe_deq(entry);					\
	}								\

/**
 * @brief
 * Declares and defines a function that removes an entry from a hash table
 *
 * @param[in] name	prefix name for the hash init functions.
 * @param[in] e_type	hash entry type
 * @param[in] h_fn	is a macro or function that hashes an element to
 * 			an integer. This hash function should return a
 * 			value that is less than n_elem value (that was used
 * 			in hash_init() call).
 * @param[in] c_fn	is a macro or function that takes two e_type args
 * 			and compares the key values embedded in that
 * 			structure. Should return 1 if the keys match and
 * 			0 otherwise.
 *
 * @return
 * NA
 *
 * Special Considerations:
 *
 * @note
 */
#define BFA_HT_LOOKUP(name, e_type, h_fn, c_fn)				\
	e_type								\
	name##_hash_lookup(bfa_ht_t ht_hdl, e_type he_with_key);	\
									\
	e_type								\
	name##_hash_lookup(bfa_ht_t ht_hdl, e_type he_with_key)		\
	{								\
		struct bfa_q_s *q, *qe;					\
		int h_val;						\
									\
		/* Basic checking */					\
		bfa_assert(ht_hdl != NULL && he_with_key != NULL);	\
									\
		/* Hash the data */					\
		h_val = h_fn(he_with_key);				\
		bfa_assert(h_val >= 0);					\
									\
		q = &ht_hdl[h_val];					\
		qe = bfa_q_next(q);					\
									\
		/* Search for exact match */				\
		while (qe != q) {					\
			if (c_fn(((e_type)qe), he_with_key))		\
				return (e_type)qe;			\
			else						\
				qe = bfa_q_next(qe);			\
		}							\
									\
		return NULL;						\
	}

/**
 * @ BFA state machine interfaces
 */

typedef void (*bfa_sm_t)(void *sm, int event);

/**
 * oc - object class eg. bfa_ioc
 * st - state, eg. reset
 * otype - object type, eg. struct bfa_ioc_s
 * etype - object type, eg. enum ioc_event
 */
#define bfa_sm_state_decl(oc, st, otype, etype)		\
	static void oc ## _sm_ ## st(otype * fsm, etype event)

#define bfa_sm_set_state(_sm, _state)	((_sm)->sm = (bfa_sm_t)(_state))
#define bfa_sm_send_event(_sm, _event)	((_sm)->sm((_sm), (_event)))
#define bfa_sm_get_state(_sm)		((_sm)->sm)
#define bfa_sm_cmp_state(_sm, _state)	((_sm)->sm == (bfa_sm_t)(_state))

/**
 * For converting from state machine function to state encoding.
 */
struct bfa_sm_table_s {
	bfa_sm_t	sm;	/*!< state machine function	*/
	int		state;	/*!< state machine encoding	*/
	char		*name;	/*!< state name for display	*/
};
typedef struct bfa_sm_table_s bfa_sm_table_t;
#define BFA_SM(_sm)	((bfa_sm_t)(_sm))

/**
 * State machine with entry actions.
 */
typedef void (*bfa_fsm_t)(void *fsm, int event);

/**
 * oc - object class eg. bfa_ioc
 * st - state, eg. reset
 * otype - object type, eg. struct bfa_ioc_s
 * etype - object type, eg. enum ioc_event
 */
#define bfa_fsm_state_decl(oc, st, otype, etype)		\
	static void oc ## _sm_ ## st(otype * fsm, etype event);	\
	static void oc ## _sm_ ## st ## _entry(otype * fsm)

#define bfa_fsm_set_state(_fsm, _state) do {	\
	(_fsm)->fsm = (bfa_fsm_t)(_state);	\
	_state ## _entry(_fsm);			\
} while (0)

#define bfa_fsm_send_event(_fsm, _event)	((_fsm)->fsm((_fsm), (_event)))
#define bfa_fsm_get_state(_fsm)			((_fsm)->fsm)
#define bfa_fsm_cmp_state(_fsm, _state)		\
	((_fsm)->fsm == (bfa_fsm_t)(_state))

static inline int
bfa_sm_to_state(struct bfa_sm_table_s *smt, bfa_sm_t sm)
{
	int	i = 0;

	while (smt[i].sm && smt[i].sm != sm)
		i++;
	return smt[i].state;
}

/**
 * @ Generic wait counter.
 */

typedef void (*bfa_wc_resume_t) (void *cbarg);

struct bfa_wc_s {
	bfa_wc_resume_t wc_resume;
	void		*wc_cbarg;
	int		wc_count;
};
typedef struct bfa_wc_s bfa_wc_t;

static inline void
bfa_wc_up(struct bfa_wc_s *wc)
{
	wc->wc_count++;
}

static inline void
bfa_wc_down(struct bfa_wc_s *wc)
{
	wc->wc_count--;
	if (wc->wc_count == 0)
		wc->wc_resume(wc->wc_cbarg);
}

/**
 * Initialize a waiting counter.
 */
static inline void
bfa_wc_init(struct bfa_wc_s *wc, bfa_wc_resume_t wc_resume, void *wc_cbarg)
{
	wc->wc_resume = wc_resume;
	wc->wc_cbarg = wc_cbarg;
	wc->wc_count = 0;
	bfa_wc_up(wc);
}

/**
 * Wait for counter to reach zero
 */
static inline void
bfa_wc_wait(struct bfa_wc_s *wc)
{
	bfa_wc_down(wc);
}

#endif /* __BFA_CS_H__ */
