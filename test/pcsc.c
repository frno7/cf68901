// SPDX-License-Identifier: GPL-2.0
/* Copyright (C) 2025 Fredrik Noring */

#define _POSIX_C_SOURCE 1

#include <ctype.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>

#include "cf68901/assert.h"
#include "cf68901/macro.h"
#include "cf68901/types.h"

enum pcs_token_type {
	PCS_TOKEN_TYPE_EOF = 0,
	PCS_TOKEN_TYPE_CYCLE,
	PCS_TOKEN_TYPE_ASSIGN,
	PCS_TOKEN_TYPE_ASSERT,
};

struct pcs_token {
	enum pcs_token_type type;
	int line;
	char cmd[100];
	char arg[100];
};

static struct pcs_token pcs_lex(FILE *file, int *line, const char *path)
{
	enum {
		STATE_SPACE,
		STATE_COMMENT,
		STATE_COMMAND,
	};

	struct pcs_token t = { .line = *line };
	int state = STATE_SPACE;
	size_t cmd_len = 0;
	size_t arg_len = 0;

	while (!feof(file)) {
		const int c = fgetc(file);

		if (c == EOF) {
			if (ferror(file)) {
				perror(path);
				exit(EXIT_FAILURE);
			}
			break;
		}

		if (c == '\n')
			(*line)++;
		if (state == STATE_SPACE && isspace(c))
			continue;
		if (state == STATE_SPACE && c == '-') {
			state = STATE_COMMENT;
			continue;
		}
		if (state == STATE_COMMENT) {
			if (c == '\n')
				state = STATE_SPACE;
			continue;
		}
		if (isspace(c))
			break;

		state = STATE_COMMAND;
		if (c == '#') { t.type = PCS_TOKEN_TYPE_CYCLE;  continue; }
		if (c == '=') { t.type = PCS_TOKEN_TYPE_ASSIGN; continue; }
		if (c == '!') { t.type = PCS_TOKEN_TYPE_ASSERT; continue; }

		if (cmd_len + 1 >= ARRAY_SIZE(t.cmd) ||
		    arg_len + 1 >= ARRAY_SIZE(t.arg)) {
			fprintf(stderr, "%s: Token too long\n", path);
			exit(EXIT_FAILURE);
		}

		if (t.type)
			t.arg[arg_len++] = c;
		else
			t.cmd[cmd_len++] = c;
	}

	return t;
}

static void pcsc(FILE *out_file, FILE *pcs_file, const char *pcs_path)
{
	fprintf(out_file,
		"#include \"cf68901/test/suite.h\"\n"
		"#include \"cf68901/test/test.h\"\n"
		"\n"
		"CF68901_TEST_PROTOTYPE\n"
		"{\n"
		"\tTEST_INIT;\n"
		"\n"
		"\tconst char *source = \"%s\";\n"
		"\n"
		"\n", pcs_path);
	int line = 1;

	for (;;) {
		const struct pcs_token t = pcs_lex(pcs_file, &line, pcs_path);

		if (!t.type)
			break;

		if (t.type == PCS_TOKEN_TYPE_CYCLE)
			fprintf(out_file, "\tcycle(%s);\n", t.arg);
		else if (t.type == PCS_TOKEN_TYPE_ASSIGN)
			fprintf(out_file, "\tassign(%s, %s);\n", t.cmd, t.arg);
		else if (t.type == PCS_TOKEN_TYPE_ASSERT)
			fprintf(out_file, "\tassert(%s, %s, %d);\n",
				t.cmd, t.arg, t.line);
		else {
			fprintf(stderr, "%s: Unknown token type %d\n",
				pcs_path, t.type);
			exit(EXIT_FAILURE);
		}
	}

	fprintf(out_file,
		"\n"
		"\treturn NULL;\n"
		"}\n");
}

int main(int argc, char *argv[])
{
	if (argc != 4 || argv[1][0] != '-'
		      || argv[1][1] != 'o'
		      || argv[1][2] != '\0') {
		fprintf(stderr, "usage: pcsc -o <c-file> <pcs-file>\n");

		return EXIT_FAILURE;
	}

	const char * const out_path = argv[2];
	const char * const pcs_path = argv[3];
	char tmp_path[PATH_MAX];

	snprintf(tmp_path, sizeof(tmp_path), "%s.tmp", out_path);

	FILE *pcs_file = fopen(pcs_path, "r");
	if (!pcs_file) {
		perror(pcs_path);
		return EXIT_FAILURE;
	}

	FILE *out_file = fopen(tmp_path, "wt");
	if (!out_file) {
		perror(tmp_path);
		return EXIT_FAILURE;
	}

	pcsc(out_file, pcs_file, pcs_path);

	if (ferror(pcs_file)) {
		perror(pcs_path);
		return EXIT_FAILURE;
	}

	if (ferror(out_file)) {
		perror(tmp_path);
		return EXIT_FAILURE;
	}

	fclose(out_file);
	fclose(pcs_file);

	if (rename(tmp_path, out_path) == -1) {
		perror(tmp_path);
		return EXIT_FAILURE;
	}

	return EXIT_SUCCESS;
}
