/*
 * Copyright (c) 2005-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corporation.
 * All rights reserved
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */

/**
 * @file bfa_q.h Circular queue definitions.
 */

#ifndef __BFA_Q_H__
#define __BFA_Q_H__

struct bfa_q_s {
	struct bfa_q_s	*next;
	struct bfa_q_s	*prev;
};
typedef struct bfa_q_s bfa_q_t;

#define bfa_q_first(_q) ((void *)(((struct bfa_q_s *) (_q))->next))
#define bfa_q_next(_qe)	(((struct bfa_q_s *) (_qe))->next)
#define bfa_q_prev(_qe) (((struct bfa_q_s *) (_qe))->prev)

/*
 * bfa_q_init - to initialize a queue
 */
#define bfa_q_init(_q) {						\
	bfa_q_next(_q) = (struct bfa_q_s *) (_q);			\
	bfa_q_prev(_q) = (struct bfa_q_s *) (_q);			\
}

/*
 * bfa_q_qe_init - to initialize a queue element
 */
#define bfa_q_qe_init(_qe) {						\
	bfa_q_next(_qe) = (struct bfa_q_s *) NULL;			\
	bfa_q_prev(_qe) = (struct bfa_q_s *) NULL;			\
}

/*
 * bfa_q_enq - enqueue an element at the end of the list
 */
#define bfa_q_enq(_q, _qe) do {						\
	bfa_assert_fp((bfa_q_next(_qe) == NULL) && 			\
					(bfa_q_prev(_qe) == NULL));	\
	bfa_q_next(_qe) = (struct bfa_q_s *) (_q);			\
	bfa_q_prev(_qe) = bfa_q_prev(_q);				\
	bfa_q_next(bfa_q_prev(_q)) = (struct bfa_q_s *) (_qe);		\
	bfa_q_prev(_q) = (struct bfa_q_s *) (_qe);			\
} while (0)

/*
 * bfa_q_enq_head - enqueue an element at the head of queue
 */
#define bfa_q_enq_head(_q, _qe) {					\
	bfa_assert_fp((bfa_q_next(_qe) == NULL) && 			\
					(bfa_q_prev(_qe) == NULL));	\
	bfa_q_next(_qe) = bfa_q_next(_q);				\
	bfa_q_prev(_qe) = (struct bfa_q_s *) (_q);			\
	bfa_q_prev(bfa_q_next(_q)) = (struct bfa_q_s *) (_qe);		\
	bfa_q_next(_q) = (struct bfa_q_s *) (_qe);			\
}

/*
 * bfa_q_is_empty - returns TRUE if queue is empty
 */
#define bfa_q_is_empty(_q)						\
	(bfa_q_next(_q) == ((struct bfa_q_s *) (_q)))

/*
 * bfa_q_enq_q - enqueue another queue at the tail
 */
#define bfa_q_enq_q(_dstq, _srcq) {					\
	if (!bfa_q_is_empty(_srcq)) {					\
		bfa_q_next(bfa_q_prev(_srcq)) = (struct bfa_q_s *) (_dstq); \
		bfa_q_prev(bfa_q_next(_srcq)) = bfa_q_prev(_dstq);	\
		bfa_q_next(bfa_q_prev(_dstq)) = bfa_q_next(_srcq);	\
		bfa_q_prev(_dstq) = bfa_q_prev(_srcq);			\
		bfa_q_init(_srcq);					\
	}								\
}

/*
 * bfa_q_enq_q_head - enqueue another queue at the head
 */
#define bfa_q_enq_q_head(_dstq, _srcq) {				\
	if (!bfa_q_is_empty(_srcq)) {					\
		bfa_q_next(bfa_q_prev(_srcq)) = bfa_q_next(_dstq);	\
		bfa_q_prev(bfa_q_next(_srcq)) = (struct bfa_q_s *) (_dstq); \
		bfa_q_prev(bfa_q_next(_dstq)) = bfa_q_prev(_srcq);	\
		bfa_q_next(_dstq) = bfa_q_next(_srcq);			\
		bfa_q_init(_srcq);					\
	}								\
}

/*
 * bfa_q_qe_deq - dequeue a queue element from a queue
 */
#define bfa_q_qe_deq(_qe) {						\
	bfa_assert_fp(bfa_q_next(_qe) && bfa_q_prev(_qe));		\
	bfa_q_next(bfa_q_prev(_qe)) = bfa_q_next(_qe);			\
	bfa_q_prev(bfa_q_next(_qe)) = bfa_q_prev(_qe);			\
	BFA_Q_DBG_INIT(_qe);						\
}

/*
 * bfa_q_qe_mov - Move a queued element to given queue
 */
#define bfa_q_qe_mov(_qe, _q) {						\
	bfa_q_qe_deq(_qe);						\
	bfa_q_enq(_q, _qe);						\
}

/*
 * bfa_q_deq - dequeue an element from head of the queue
 */
#define bfa_q_deq(_q, _qe) do {						\
	if (!bfa_q_is_empty(_q)) {					\
		(*((struct bfa_q_s **) (_qe))) = bfa_q_next(_q);	\
		bfa_q_prev(bfa_q_next(*((struct bfa_q_s **) _qe))) =	\
						(struct bfa_q_s *) (_q); \
		bfa_q_next(_q) = bfa_q_next(*((struct bfa_q_s **) _qe)); \
		BFA_Q_DBG_INIT(*((struct bfa_q_s **) _qe));		\
	} else {							\
		*((struct bfa_q_s **) (_qe)) = (struct bfa_q_s *) NULL;	\
	}								\
} while (0)

/*
 * bfa_q_deq_tail - dequeue an element from tail of the queue
 */
#define bfa_q_deq_tail(_q, _qe) {					\
	if (!bfa_q_is_empty(_q)) {					\
		*((struct bfa_q_s **) (_qe)) = bfa_q_prev(_q);		\
		bfa_q_next(bfa_q_prev(*((struct bfa_q_s **) _qe))) = 	\
						(struct bfa_q_s *) (_q); \
		bfa_q_prev(_q) = bfa_q_prev(*(struct bfa_q_s **) _qe);	\
		BFA_Q_DBG_INIT(*((struct bfa_q_s **) _qe));		\
	} else {							\
		*((struct bfa_q_s **) (_qe)) = (struct bfa_q_s *) NULL;	\
	}								\
}

/*
 * bfa_q_mv - move all queue elements from one to another queue
 */
#define bfa_q_mv(_srcq, _dstq) {					\
	bfa_assert_fp(bfa_q_is_empty(_dstq));				\
									\
	if (!bfa_q_is_empty(_srcq)) {					\
		bfa_q_next(_dstq) = bfa_q_next(_srcq);			\
		bfa_q_prev(_dstq) = bfa_q_prev(_srcq);			\
		bfa_q_prev(bfa_q_next(_srcq)) = (struct bfa_q_s *) _dstq; \
		bfa_q_next(bfa_q_prev(_srcq)) = (struct bfa_q_s *) _dstq; \
		bfa_q_init(_srcq);					\
	}								\
}

/*
 * bfa_q_iter - iterator macros, use only if it is known that list
 * does not change.
 */
#define bfa_q_iter(_q, _qe)						\
	for (_qe = bfa_q_first(_q); _qe != (_q); _qe = bfa_q_next(_qe))

#define bfa_q_iter_safe(_q, _qe, _qe_next) 				\
	for (_qe = bfa_q_first(_q), _qe_next = bfa_q_next(_qe); 	\
		 _qe != (_q); _qe = _qe_next, _qe_next = bfa_q_next(_qe))

/*
 * #ifdef BFA_DEBUG (Using bfa_assert to check for debug_build is not
 * consistent across modules)
 */
#ifndef BFA_PERF_BUILD
#define BFA_Q_DBG_INIT(_qe)	bfa_q_qe_init(_qe)
#else
#define BFA_Q_DBG_INIT(_qe)
#endif

#define bfa_q_is_on_q(_q, _qe)	bfa_q_is_on_q_func(_q, (struct bfa_q_s *)(_qe))
extern int bfa_q_is_on_q_func(struct bfa_q_s *q, struct bfa_q_s *qe);

#endif
