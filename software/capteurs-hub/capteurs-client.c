/*
 * Copyright (c) 2025 Manuel Bouyer
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE NETBSD FOUNDATION, INC. AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE FOUNDATION OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <string.h>
#include <err.h>
#include <sys/socket.h>
#include <sys/un.h>

#define SOCKET_PATH "/var/run/c_hub.sock"

#define LINESZ 128

static void
usage(void)
{
	fprintf(stderr, "usage: %s [-n] [command] ...\n", getprogname());
	exit(1);
}

int
main(int argc, char * const argv[])
{
	int ch;
	int sockfd;
	int nflag = 0;
	int sflag = 0;
	FILE *sock_f;
	struct sockaddr_un saddr;
	int i;

	while ((ch = getopt(argc, argv, "hns")) != -1) {
		switch(ch) {
		case 'n':
			nflag++;
			break;
		case 's':
			sflag++;
			break;
		case 'h':
		case '?':
		default:
			usage();
		}
	}
	argc -= optind;
	argv += optind;

	if ((sockfd = socket(AF_UNIX, SOCK_STREAM, 0)) == -1) {
		err(EXIT_FAILURE, "socket");
	}
	memset(&saddr, 0, sizeof(saddr));
	saddr.sun_family = AF_UNIX;
	strncpy(saddr.sun_path, SOCKET_PATH, sizeof(saddr.sun_path) - 1);

	if (connect(sockfd, (struct sockaddr *)&saddr, sizeof(saddr)) == -1) {
		err(EXIT_FAILURE, "connect %s", SOCKET_PATH);
	}

	sock_f = fdopen(sockfd, "w+");
	if (sock_f == NULL)
		err(1, "fopen %s", SOCKET_PATH);

	/* send strings from command line */
	for (i = 0; i < argc; i++) {
		if (fprintf(sock_f, "%s\n", argv[i]) < 0 ||
		    fflush(sock_f) < 0) {
			err(1, "send failed");
		}
	}

	if (nflag && sflag) {
		fclose(sock_f);
		exit(0);
	}

	while (1) {
		struct pollfd fds[2];
		static char linebuf[LINESZ];

		fds[0].fd = sockfd;
		fds[0].events = POLLRDNORM;
		fds[1].fd = STDIN_FILENO;
		fds[1].events = POLLRDNORM;

		if (poll(fds, (nflag ? 1 : 2), -1) < 0) {
			err(1, "poll");
		}
		if (nflag == 0 && fds[1].revents & POLLRDNORM) {
			if (fgets(linebuf, LINESZ, stdin) == NULL) {
				fclose(sock_f);
				exit(0);
			}
			if (fprintf(sock_f, "%s", linebuf) < 0 ||
			    fflush(sock_f) < 0) {
				err(1, "can't write to socket");
			}
		}
		if (fds[0].revents & POLLRDNORM) {
			if (fgets(linebuf, LINESZ, sock_f) == NULL) {
				errx(1, "can't read from socket");
			}
			if (fprintf(stdout, "%s", linebuf) < 0 ||
			    fflush(stdout) < 0) {
				err(1, "write to stdout");
			}
		}
	}
}
