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
#include <ctype.h>
#include <fcntl.h>
#include <poll.h>
#include <string.h>
#include <err.h>
#include <errno.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <grp.h>
#include <pwd.h>
#include <termios.h>
#include <syslog.h>
#include <util.h>

#define SOCKET_PATH "/var/run/e_hub.sock"

static void
usage(void)
{
	fprintf(stderr, "usage: %s <linky tty>\n", getprogname());
	exit(1);
}

static int opt_f = 0;
#define MAX_CLIENTS 128
#define LINESZ 80

static void mylog(int, const char *, ...) __sysloglike(2, 3);

static void
mylog(int l, const char *fmt, ...)
{
        va_list ap; 
	char buf[LINESZ];

	va_start(ap, fmt);
	(void)vsnprintf(buf, sizeof(buf), fmt, ap);
	va_end(ap);

	if (opt_f) {
		fprintf(stderr, "%s\n", buf);
	}
	syslog(l | LOG_LOCAL2, "%s", buf);
}

static struct clients_fds {
	int fd;
	FILE *f;
} clients_fds[MAX_CLIENTS] = {0};
static int nclients = 0;

static int linky;
static FILE *linky_f;

static struct timeval msg_ts; /* timespamp of messages */

static void
linky_write(char *buf)
{
	int s = strlen(buf);
	unsigned int sum = 0;
	int i;

	/* strip newline */
	if (buf[s - 1] == '\n') {
		buf[s - 1]  = '\0';
		s--;
	}
	printf("write to linky %d %s\n", s, buf);
	/* strip 8th bit, compute checksum */
	for (i = 0; i < s; i++) {
		buf[i] = buf[i] & 0x7f;
		if (buf[i] < 0x20) {
			printf("write ignore %d %d\n", i, buf[i]);
			
			return; /* ignore bad chars */
		}
		sum += buf[i];
	}
	if (fprintf(linky_f, buf) < 0) {
		mylog(LOG_ERR, "linky write: %s", strerror(errno));
		exit (1);
	}

	sum = (sum & 0x3f) + 0x20;
	if (fprintf(linky_f, " %c\r\n", sum) < 0 || fflush(linky_f) < 0) {
		mylog(LOG_ERR, "linky write: %s", strerror(errno));
		exit (1);
	}
}

static void
clients_write(char *buf)
{
	int s = strlen(buf);
	unsigned int sum = 0;
	int i;

	/* verify checksum */
	for (i = 0; i < s - 2; i++) {
		buf[i] = buf[i] & 0x7f;
		sum += buf[i];
	}
	sum = (sum & 0x3f) + 0x20;
	if (s < 2 || buf[s - 2] != ' ' || buf[s - 1] != sum) {
		mylog(LOG_NOTICE, "linky sum mismatch: %s %d %d != %d",
		    buf, s, sum, buf[s - 1]);
		return;
	}
	/* strip checksum */
	buf[s - 2] = '\0';
	/* send to clients */
	for (i = 0; i < nclients; i++) {
		if (clients_fds[i].f == NULL)
			continue;
		printf("send to client %d\n", i);
		if (fprintf(clients_fds[i].f, "%d %s\n", msg_ts.tv_sec, buf) < 0 ||
		    fflush(clients_fds[i].f) < 0) {
			printf("close client %d\n", i);
			fclose(clients_fds[i].f);
			clients_fds[i].f = NULL;
			close(clients_fds[i].fd);
		}
	}
}

int
main(int argc, char * const argv[])
{
	int ch;
	int main_socket;
	struct sockaddr_un saddr;
	struct termios cntrl;
	int i;
	long l;
	static char linebuf[LINESZ];
	static char linkybuf[LINESZ];
	int linkybufi = 0;
	struct pollfd fds[MAX_CLIENTS + 2];
	int nfds;

	uid_t uid = 0;
	gid_t gid = 0; 
	char *user = NULL;
	char *group = NULL;

	char *endp;
	struct group   *gr;
	struct passwd  *pw;

	while ((ch = getopt(argc, argv, "fu:g:")) != -1) {
		switch(ch) {
		case 'f':
			opt_f++;
			break;
		case 'g':
			group = optarg;
			if (*group == '\0')
				usage();
			break;
		case 'u':
			user = optarg;
			if (*user == '\0')
				usage();
			break;

		case '?':
		default:
			usage();
		}
	}
	argc -= optind;
	argv += optind;
	if (argc != 1)
		usage();

	if (user != NULL) {
		if (isdigit((unsigned char)*user)) {
			errno = 0;
			endp = NULL;
			l = strtoul(user, &endp, 0);
			if (errno || *endp != '\0')
				goto getuser;
			uid = (uid_t)l;
		} else {
getuser:
			if ((pw = getpwnam(user)) != NULL) {
				uid = pw->pw_uid;
			} else {
				errno = 0;
				errx(1, "Cannot find user `%s'", user);
			}
		}
	}
	if (group != NULL) {
		if (isdigit((unsigned char)*group)) {
			errno = 0;
			endp = NULL;
			l = strtoul(group, &endp, 0);
			if (errno || *endp != '\0')
				goto getgroup;
			gid = (gid_t)l;
		} else {
getgroup:
			if ((gr = getgrnam(group)) != NULL) {
				gid = gr->gr_gid;
			} else {
				errno = 0;
				errx(1, "Cannot find group `%s'", group);
			}
		}
	}

	unlink(SOCKET_PATH);
	/* need to open with O_NONBLOCK, util we set CLOCAL */
	linky = open(argv[0], O_RDWR | O_NOCTTY | O_NONBLOCK);
	if (linky < 0)
		err(1, "open %s", argv[0]);

	if (flock(linky, (LOCK_EX|LOCK_NB)) != 0)
		err(1, "flock %s", argv[0]);

	tcgetattr(linky, &cntrl);
	cfsetospeed(&cntrl, 57600);
	cfsetispeed(&cntrl, 57600);
	cntrl.c_cflag &= ~(CSIZE|PARENB);
	cntrl.c_cflag |= CS8;
	cntrl.c_cflag |= CLOCAL;
	cntrl.c_iflag &= ~(ISTRIP|ICRNL);
	cntrl.c_oflag &= ~OPOST;
	cntrl.c_lflag &= ~(ICANON|ISIG|IEXTEN|ECHO);
	cntrl.c_cc[VMIN] = 1;
	cntrl.c_cc[VTIME] = 0;
	if (tcsetattr(linky, TCSAFLUSH, &cntrl) != 0) {
		err(1, "tcsetattr %s", argv[0]);
	}
	if ((i = fcntl(linky, F_GETFL, 0)) < 0)  {
		err(1, "F_GETFL %s", argv[0]);
	}
	if (fcntl(linky, F_SETFL, i & ~O_NONBLOCK) < 0) {
		err(1, "F_SETFL %s", argv[0]);
	}

	linky_f = fdopen(linky, "w+");
	if (linky_f == NULL)
		err(1, "fopen %s", argv[1]);

	main_socket = socket(AF_UNIX, SOCK_STREAM, 0);
	if (main_socket < 0)
		err(1, "create %s", SOCKET_PATH);

	memset(&saddr, 0, sizeof(saddr));
	saddr.sun_family = AF_UNIX;
	strncpy(saddr.sun_path, SOCKET_PATH, sizeof(saddr.sun_path) - 1);

	if (bind(main_socket, (struct sockaddr *)&saddr, sizeof(saddr)) != 0)
		err(1, "bind %s",  SOCKET_PATH);

	if (listen(main_socket, 10) != 0)
		err(1, "listen %s",  SOCKET_PATH);

	if (lchown(SOCKET_PATH, uid, gid) < 0)
		err(1, "chown %s to %d:%d", SOCKET_PATH, uid, gid);

	if (lchmod(SOCKET_PATH, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP) < 0)
		err(1, "chmod %s to 0660", SOCKET_PATH);

	if (opt_f == 0) {
		if (daemon(0, 0) != 0)
			err(1, "daemon()");
		pidfile(NULL);
	}

	if (setgid(gid) || setegid(gid)) {
		mylog(LOG_ERR, "Failed to set gid to %d: %s", gid,
		    strerror(errno));
		exit(1);
	}
	if (setuid(uid) || seteuid(uid)) { 
		mylog(LOG_ERR, "Failed to set uid to %d: %s", uid,
		    strerror(errno));
		exit(1);
	}

	printf("ready\n");
	while (1) {
		for (i = 0; i < nclients; i++) {
			fds[i].fd = clients_fds[i].fd;
			fds[i].events = POLLRDNORM;
		}
		fds[i].fd = linky;
		fds[i].events = POLLRDNORM;
		i++;
		fds[i].fd = main_socket;
		fds[i].events = POLLRDNORM;
		nfds = i + 1;

		if (poll(fds, nfds, -1) < 0) {
			mylog(LOG_ERR, "poll: %s", strerror(errno));
			exit(1);
		}
		for (i = 0; i < nclients; i++) {
			if (fds[i].revents & POLLRDNORM) {
				printf("read from client %d\n", i);
				if (fgets(linebuf, LINESZ, clients_fds[i].f)
				    == NULL) {
					printf("close client %d\n", i);
					fclose(clients_fds[i].f);
					clients_fds[i].f = NULL;
					close(clients_fds[i].fd);
				} else {
					linky_write(linebuf);
				}
			}
		}
		if (fds[i].revents & POLLRDNORM) {
			if (linkybufi == 0) {
				if (gettimeofday(&msg_ts, NULL) != 0) {
					mylog(LOG_ERR, "gettimeofday: %s",
					    strerror(errno));
					exit(1);
				}
			}
			switch(read(linky, &linkybuf[linkybufi], 1)) {
			case 0:
				/* ignore short read */
				break;
			case -1:
				mylog(LOG_ERR, "can't read linky: %s",
				    strerror(errno));
				exit(1);
			default:
				if (linkybuf[linkybufi] == '\r') {
					/* complete line */
					linkybuf[linkybufi] = '\0';
					clients_write(linkybuf);
					linkybufi = 0;
				} else if (linkybuf[linkybufi] < 0x20) {
					/* ignore */
				} else if (linkybufi < LINESZ - 2) {
					linkybufi++;
				}
			}
		}
		i++;
		if (fds[i].revents & POLLRDNORM) {
			int new_s;
			socklen_t socksize = sizeof(saddr);
			printf("accept new client\n");
			new_s = accept4(main_socket, (struct sockaddr *)&saddr,
			    &socksize, SOCK_NOSIGPIPE);
			if (nclients < MAX_CLIENTS) {
				clients_fds[nclients].fd = new_s;
				clients_fds[nclients].f =
				    fdopen(new_s, "w+");
				if (clients_fds[nclients].f == NULL) {
					mylog(LOG_ERR, "fdopen new client: %s",
				    	    strerror(errno));
					close(new_s);
				} else {
					printf("added client %d\n", nclients);
					nclients++;
				}
			} else {
				close(new_s);
			}
		}
		/* cleanup closed clients, compact array */
		for (i = 0 ; i < nclients; i++) {
			if (clients_fds[i].f != NULL)
				continue;
			printf("cleanup client %d\n", i);
			memmove(&clients_fds[i], &clients_fds[i+1],
			    sizeof(struct clients_fds) * (nclients - (i + 1)));
			nclients--;
		}
			    
	}
}
