/*
 * Copyright (c) 2005-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corporation.
 * All rights reserved
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */

/**
 * @file pbind.c BFA FC Persistent target binding
 */

#include <cs/bfa_cs.h>
#include <log/bfa_log_hal.h>

/**
 * @ig cs_debug_api
 * @{
 */

/**
 * 	@brief
 */

char *
bfa_file_basename(char *file)
{
	int i;

	for (i = strlen(file) - 1; i > 0; i--)
		if (file[i] == '/' || file[i] == '\\')
			return file + i + 1;
	return file;
}

void
bfa_panic(int line, char *file, char *panicstr)
{
	bfa_log(NULL, BFA_LOG_HAL_ASSERT, bfa_file_basename(file), line,
			panicstr);
	bfa_os_panic();
}

void
bfa_sm_panic(struct bfa_log_mod_s *logm, int line, char *file, int event)
{
	bfa_log(logm, BFA_LOG_HAL_SM_ASSERT, bfa_file_basename(file), line,
			event);
	bfa_os_panic();
}

int
bfa_q_is_on_q_func(struct bfa_q_s *q, struct bfa_q_s *qe)
{
	struct bfa_q_s *tqe = bfa_q_next(q);

	for (; tqe != NULL && tqe != q; tqe = bfa_q_next(tqe))
		if (tqe == qe)
			return (1);
	return (0);
}

/**
 * @}
 */
