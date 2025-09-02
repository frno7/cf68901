// SPDX-License-Identifier: GPL-2.0
/* Copyright (C) 2025 Fredrik Noring */

#define _POSIX_C_SOURCE 1

#include <ctype.h>
#include <inttypes.h>
#include <limits.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cf68901/assert.h"
#include "cf68901/macro.h"
#include "cf68901/types.h"

#include "cf68901/module/assert.h"
#include "cf68901/module/cf68901.h"

#include "cf68901/test/suite.h"

#define ERROR_LEVEL MODULE_ERR

static bool error;

static void report(const char *fmt, ...)
{
	va_list ap;

	va_start(ap, fmt);

	if (fmt[0] == MODULE_SOH[0]) {
		if (fmt[1] <= (ERROR_LEVEL)[1])
			error = true;
		fmt = &fmt[fmt[1] ? 2 : 1];
	}

	vfprintf(stderr, fmt, ap);

	va_end(ap);
}

int main(int argc, char *argv[])
{
	const char *error = test(report);

	if (error) {
		fprintf(stderr, "%s\n", error);
		return EXIT_FAILURE;
	}

	return EXIT_SUCCESS;
}
