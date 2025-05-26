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
#include <sys/ioctl.h>
#include <dev/i2c/i2c_io.h>

#include "bme280.h"

struct bme280_dev bme280_dev;
uint8_t dev_addr = BME280_I2C_ADDR_PRIM;


#define SOCKET_PATH "/var/run/c_hub.sock"

static void
usage(void)
{
	fprintf(stderr, "usage: %s [-f] [-u user] [-g group] <LIN tty> <i2c>\n", getprogname());
	exit(1);
}

static int opt_f = 0;
#define MAX_CLIENTS 128
#define LINESZ 80

static void mylog(int, const char *, ...) __sysloglike(2, 3);
static void myprintf(const char *, ...) __printflike(1, 2);

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

static void
myprintf(const char *fmt, ...)
{
	va_list ap;

	if (opt_f) {
		va_start(ap, fmt);
		vprintf(fmt, ap);
		va_end(ap);
	}
}

static struct clients_fds {
	int fd;
	FILE *f;
} clients_fds[MAX_CLIENTS] = {0};
static int nclients = 0;

static int lin_fd, iic_fd;
static FILE *lin_f;

enum lin_state { 
	IDLE,
	TXPID,
	TXDATA,
	RXDATA
} lin_state;

static struct timeval msg_ts; /* timespamp of messages */
#define SENSORS_PERIOD 30 /* seconds */

static void clients_write(char *);

static void
user_delay_us(uint32_t period, void *intf_ptr)
{
	usleep(period);
}

static int8_t
user_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
	i2c_ioctl_exec_t iie;

	iie.iie_op = I2C_OP_READ_WITH_STOP;
	iie.iie_addr = dev_addr;
	iie.iie_cmd = &reg_addr;
	iie.iie_cmdlen = 1;
	iie.iie_buf = reg_data;
	iie.iie_buflen = len;

	if (ioctl(iic_fd, I2C_IOCTL_EXEC, &iie) == -1) {
		perror("ioctl read");
		return BME280_E_COMM_FAIL;
	}
	return BME280_OK;

}

static int8_t
user_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len,
    void *intf_ptr)
{
	i2c_ioctl_exec_t iie;

	iie.iie_op = I2C_OP_WRITE_WITH_STOP;
	iie.iie_addr = dev_addr;
	iie.iie_cmd = &reg_addr;
	iie.iie_cmdlen = 1;
	iie.iie_buf = __UNCONST(reg_data);
	iie.iie_buflen = len;

	if (ioctl(iic_fd, I2C_IOCTL_EXEC, &iie) == -1) {
		perror("ioctl write");
		return BME280_E_COMM_FAIL;
	}
	return BME280_OK;
}

static int
do_bme280_init()
{
	int rslt;
	uint8_t settings_sel;

	bme280_dev.intf_ptr = &dev_addr;
	bme280_dev.intf = BME280_I2C_INTF;
	bme280_dev.read = user_i2c_read;
	bme280_dev.write = user_i2c_write;
	bme280_dev.delay_us = user_delay_us;

	rslt = bme280_init(&bme280_dev);
	if (rslt != BME280_OK) {
		warnx("bme280_init failed: %d\n", rslt);
		return rslt;
	}

	bme280_dev.settings.osr_h = BME280_OVERSAMPLING_1X;
	bme280_dev.settings.osr_p = BME280_OVERSAMPLING_16X;
	bme280_dev.settings.osr_t = BME280_OVERSAMPLING_2X;
	bme280_dev.settings.filter = BME280_FILTER_COEFF_16;
	bme280_dev.settings.standby_time = BME280_STANDBY_TIME_62_5_MS;

	settings_sel = BME280_OSR_PRESS_SEL;
	settings_sel |= BME280_OSR_TEMP_SEL;
	settings_sel |= BME280_OSR_HUM_SEL;
	settings_sel |= BME280_STANDBY_SEL;
	settings_sel |= BME280_FILTER_SEL;

	rslt = bme280_set_sensor_settings(settings_sel, &bme280_dev);
	if (rslt != BME280_OK) {
		warn("bme280_set_sensor_settings failed: %d\n", rslt);
		return rslt;
	}
	rslt = bme280_set_sensor_mode(BME280_NORMAL_MODE, &bme280_dev);
	if (rslt != BME280_OK) {
		warn("bme280_set_sensor_mode failed: %d\n", rslt);
		return rslt;
	}
	return rslt;
}

static void
do_bme280()
{
	int rslt;
	struct bme280_data comp_data;
	char buf[LINESZ];

	if (iic_fd < 0)
		return;

	rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &bme280_dev);
	if (rslt != BME280_OK) {
		mylog(LOG_ERR, "bme280_get_sensor_data failed: %d\n", rslt);
	}
	snprintf(buf, LINESZ, "ITEMP %.2f", comp_data.temperature);
	clients_write(buf);
	snprintf(buf, LINESZ, "IHUM %.2f", comp_data.humidity);
	clients_write(buf);
	snprintf(buf, LINESZ, "IPRES %.2f", comp_data.pressure / 100.0);
	clients_write(buf);
}

#define STATUS_VALID 0x1
#define STATUS_VMC 0x02 /* 0 = close, 1 = open */
#define STATUS_BUTTON 0x04 /* the current state comes from button */

#define ACTION_OPEN     0x01
#define ACTION_CLOSE    0x02
#define ACTION_RESET    0x80

#define CH_ID 0x09
#define SA_ID 0x08
#define CU_ID 0x01
#define SB_ID 0x03
#define ALL_ID 0x3c

static int
lin_write_and_check(uint8_t c)
{
	struct pollfd fd[1];
	uint8_t b;

	if (write(lin_fd, &c, 1) != 1) {
		mylog(LOG_ERR, "LIN send 0x%x: %s", c, strerror(errno));
		return -1;
	}
	/* read back, wait 10ms */
	fd[0].fd = lin_fd;
	fd[0].events = POLLRDNORM;
	if (poll(fd, 1, 10) < 0) {
		mylog(LOG_ERR, "lin 0x%x back poll: %s", c, strerror(errno));
		return -1;
	}
	if ((fd[0].revents & POLLRDNORM) == 0) {
		mylog(LOG_ERR, "lin 0x%x back poll: timeout", c);
		errno = ETIMEDOUT;
		return -1;
	}
	if (read(lin_fd, &b, 1) != 1) {
		mylog(LOG_ERR, "lin 0x%x back read: %s", c, strerror(errno));
		return -1;
	}
	if (c != b) {
		mylog(LOG_ERR, "lin back read: 0x%x != 0x%x", c, b);
		return -1;
	}
	return 0;
}

static int
lin_send_pid(uint8_t pid)
{
	uint8_t c;

#define GETBIT(i, x) (((i) >> (x)) & 0x01)

	/* compute checksum */
	c = GETBIT(pid, 0) ^ GETBIT(pid, 1) ^ GETBIT(pid, 2) ^ GETBIT(pid, 4);
	pid |= (c << 6);
	c = GETBIT(pid, 1) ^ GETBIT(pid, 3) ^ GETBIT(pid, 4) ^ GETBIT(pid, 5) ^ 0x01;
	pid |= (c << 7);
	/* send break */
	if (tcsendbreak(lin_fd, 1) < 0) {
		mylog(LOG_ERR, "LIN send break: %s", strerror(errno));
		return -1;
	}
	/* send sync */
	if (lin_write_and_check(0x55) != 0)
		return -1;
	
	/* send PID */
	if (lin_write_and_check(pid) != 0)
		return -1;
	return 0;
}

static int
lin_send_data(uint8_t pid, const char *buf, int len)
{
	uint16_t csum = 0;
	if (lin_send_pid(pid) != 0) {
		return -1;
	}
	for (int i = 0; i < len; i++) {
		if (lin_write_and_check(buf[i]) != 0) {
			return -1;
		}
		csum += (uint8_t)buf[i];
		if (csum >= 256)
			csum = csum - 255;
	}
	csum = ~csum;
	if (lin_write_and_check(csum) != 0) {
		return -1;
	}
}

static int
lin_receive_data(uint8_t pid, char *buf, int len)
{
	uint16_t csum = 0;
	uint8_t rcsum;
	uint8_t *p;
	struct pollfd fd[1];

	if (lin_send_pid(pid) != 0) {
		return -1;
	}
	for (int i = 0; i <= len; i++) {
		/* read data, wait 100ms */
		fd[0].fd = lin_fd;
		fd[0].events = POLLRDNORM;
		if (poll(fd, 1, 100) < 0) {
			mylog(LOG_ERR, "lin 0x%x read poll: %s", pid,
			    strerror(errno));
			errno = ETIMEDOUT;
			return -1;
		}
		if ((fd[0].revents & POLLRDNORM) == 0) {
			mylog(LOG_ERR, "lin 0x%x rd poll: timeout", pid);
			errno = ETIMEDOUT;
			return -1;
		}
		if (i == len)
			p = &rcsum;
		else
			p = &buf[i];
		if (read(lin_fd, p, 1) != 1) {
			mylog(LOG_ERR, "lin 0x%x read: %s", pid, strerror(errno));
			return -1;
		}
		csum += *p;
		if (csum >= 256)
			csum = csum - 255;
	}
	if (csum != 0xff) {
			mylog(LOG_ERR, "lin 0x%x read: csum 0x%x != 0x%x", pid, csum, rcsum);
			errno = EIO;
			return -1;
	}
	return 0;
}

void lin_read_sensor(int lin_id)
{
	static char buf[LINESZ];
	char *sensor;
	char *act;
	int i;
	if (lin_receive_data(0x10|lin_id, buf, 5) == 0) {
		int16_t temp, hum;
		uint8_t flags = buf[0];
		myprintf("lin 0x%x:", lin_id);
		for (i = 0; i < 5; i++) {
			myprintf(" %x", buf[i]);
		}
		temp = buf[2] << 8 | buf[1];
		myprintf(" %f", (float)temp / 100.0);
		hum = buf[4] << 8 | buf[3];
		myprintf(" %f", (float)hum / 100.0);
		myprintf("\n");
		switch(lin_id) {
		case CU_ID:
			sensor = act = "Cu";
			break;
		case SB_ID:
			sensor = act = "Sb";
			break;
		case SA_ID:
			sensor = "E";
			act = "Sa";
			break;
		case CH_ID:
			sensor = act = "Ch";
			break;
		default:
			return;
		}
		if (flags & STATUS_VALID) {
			snprintf(buf, LINESZ, "%sTEMP %.2f",
			    sensor, (float)temp / 100.0);
			clients_write(buf);
			snprintf(buf, LINESZ, "%sHUM %.2f",
			    sensor, (float)hum / 100.0);
			clients_write(buf);
		}
		snprintf(buf, LINESZ, "%sSTAT %s%s",
		    act, (flags & STATUS_VMC) ? "O" : "F",
		    (flags & STATUS_BUTTON) ? " B" : "");
		clients_write(buf);
	}
}


static void
do_sensors()
{
	do_bme280();
	lin_read_sensor(CU_ID);
	lin_read_sensor(SB_ID);
	lin_read_sensor(SA_ID);
	lin_read_sensor(CH_ID);
}

static void
do_client(char *buf)
{
	uint8_t b, id;

	myprintf("client %s\n", buf);
	if (buf[2] != ' ' || buf[4] != '\n')
		return;

	switch(buf[3]) {
	case 'O':
		b = ACTION_OPEN;
		break;
	case 'F': 
		b = ACTION_CLOSE;
		break;
	case 'R': 
		b = ACTION_RESET;
		break;
	default:
		return;
	}
	if (strncmp(buf, "Cu", 2) == 0) {
		id = CU_ID;
	} else if (strncmp(buf, "Sb", 2) == 0) {
		id = SB_ID;
	} else if (strncmp(buf, "Sa", 2) == 0) {
		id = SA_ID;
	} else if (strncmp(buf, "Ch", 2) == 0) {
		id = CH_ID;
	} else if (strncmp(buf, "AA", 2) == 0) {
		id = ALL_ID;
	}
	myprintf("lin id 0x%x action 0x%x\n", id, b);
	lin_send_data(id, &b, 1);
}

static void
clients_write(char *buf)
{
	int i;

	printf("clients_write %s\n", buf);

	for (i = 0; i < nclients; i++) {
		if (clients_fds[i].f == NULL)
			continue;
		printf("send to client %d %s\n", i, buf);
		if (fprintf(clients_fds[i].f, "%" PRIu64 " %s\n",
		    msg_ts.tv_sec, buf) < 0 ||
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
	if (argc != 2)
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
	lin_fd = -1;
	lin_f = NULL;

	/* need to open with O_NONBLOCK, util we set CLOCAL */
	lin_fd = open(argv[0], O_RDWR | O_NOCTTY | O_NONBLOCK);
	if (lin_fd < 0)
		err(1, "open %s", argv[0]);

	if (flock(lin_fd, (LOCK_EX|LOCK_NB)) != 0)
		err(1, "flock %s", argv[0]);

	tcgetattr(lin_fd, &cntrl);
	cfsetospeed(&cntrl, 19200);
	cfsetispeed(&cntrl, 19200);
	cntrl.c_cflag &= ~(CSIZE|PARENB);
	cntrl.c_cflag |= CS8;
	cntrl.c_cflag |= CLOCAL;
	cntrl.c_iflag &= ~(ISTRIP|ICRNL|IXON|IXOFF|IEXTEN);
	cntrl.c_oflag &= ~(OPOST);
	cntrl.c_lflag &= ~(ICANON|ISIG|IEXTEN|ECHO);
	cntrl.c_cc[VMIN] = 1;
	cntrl.c_cc[VTIME] = 0;
	if (tcsetattr(lin_fd, TCSAFLUSH, &cntrl) != 0) {
		err(1, "tcsetattr %s", argv[0]);
	}
	if ((i = fcntl(lin_fd, F_GETFL, 0)) < 0)  {
		err(1, "F_GETFL %s", argv[0]);
	}
	if (fcntl(lin_fd, F_SETFL, i & ~O_NONBLOCK) < 0) {
		err(1, "F_SETFL %s", argv[0]);
	}

	lin_f = fdopen(lin_fd, "w+");
	if (lin_f == NULL)
		err(1, "fopen %s", argv[1]);

	iic_fd = open(argv[1], O_RDWR);

	if (iic_fd < 0) {
		err(1, "open %s", argv[1]);
	}

	if (do_bme280_init() != BME280_OK)
		iic_fd = -1;

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

	if (gettimeofday(&msg_ts, NULL) != 0) {
		mylog(LOG_ERR, "gettimeofday: %s",
		    strerror(errno));
		exit(1);
	}
	printf("ready\n");
	while (1) {
		struct timeval current_tv;
		time_t pollt;
		for (i = 0; i < nclients; i++) {
			fds[i].fd = clients_fds[i].fd;
			fds[i].events = POLLRDNORM;
		}
		fds[i].fd = main_socket;
		fds[i].events = POLLRDNORM;
		nfds = i + 1;
		if (gettimeofday(&current_tv, NULL) != 0) {
			mylog(LOG_ERR, "gettimeofday: %s",
			    strerror(errno));
			exit(1);
		}
		pollt = SENSORS_PERIOD - (current_tv.tv_sec - msg_ts.tv_sec);
		printf("pollt %" PRIu64, pollt);
		if (pollt < 0)
			pollt = 0;
		printf(" %" PRIu64 "\n", pollt * 1000);
		if (poll(fds, nfds, pollt * 1000) < 0) {
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
					do_client(linebuf);
				}
			}
		}
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
		/* see if we need a sensors report */
		if (gettimeofday(&current_tv, NULL) != 0) {
			mylog(LOG_ERR, "gettimeofday: %s",
			    strerror(errno));
			exit(1);
		}
		pollt = SENSORS_PERIOD - (current_tv.tv_sec - msg_ts.tv_sec);
		printf("pollt %" PRIu64 "\n", pollt);
		if (pollt <= 0) {
			msg_ts = current_tv;
			do_sensors();
		}
	}
}
