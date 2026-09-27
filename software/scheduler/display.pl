#!/usr/pkg/bin/perl

# Copyright (c) 2025 Manuel Bouyer
#
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE NETBSD FOUNDATION, INC. AND CONTRIBUTORS
# ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
# TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
# PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE FOUNDATION OR CONTRIBUTORS
# BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

use strict;
use IO::Socket::UNIX;
use Socket qw(SOL_SOCKET SO_KEEPALIVE);
use IO::Select;
use DateTime;
use Data::Dumper qw(Dumper);
use Getopt::Std;

use Proc::Daemon;
use Sys::Syslog qw(:standard :macros);
use sigtrap 'handler' => \&exit_handler, 'normal-signals';


my $c_sockpath = "/var/run/c_hub.sock";
my $e_sockpath = "/var/run/e_hub.sock";

my $dt;
my $timestamp;

my $lum_min = 2304;

my $debug = 0;
my $isdaemon = 0;
my ($uid, $gid);

openlog('display', 'pid', 'local5');

our($opt_c, $opt_e, $opt_d, $opt_F, $opt_P, $opt_u, $opt_g, $opt_C);
getopts("c:e:s:d:FP:Cu:g:") or usage();
usage() unless  (@ARGV == 0); 

$c_sockpath = $opt_c if defined($opt_c);
$e_sockpath = $opt_e if defined($opt_e);
$debug = $opt_d if defined($opt_d);

if (defined($opt_u)) {
        my ($n, $p, $u, $g, $q, $c, $gcos, $dir, $shell) =
	    getpwnam($opt_u) or mydie("bad user " . $opt_u);
	$uid = $u;
	$gid = $g;
}
if (defined($opt_g)) {
	my ($n, $p, $g, $m) =
	    getgrnam($opt_g) or mydie("bad group " . $opt_g);
	$gid = $g;
}

update_led("--", "today");
update_led("--", "tomorow");

closelog();
exit 0 if $opt_C;

if ($opt_F ne "1") {
        my $daemon;
        if (defined($opt_P)) {
		$daemon = Proc::Daemon->new(
		    pid_file => $opt_P
		);
        } else {
		$daemon = Proc::Daemon->new();
        }
        my $pid = $daemon->init;      
        if ($pid) {
                exit(0);
        }
        $isdaemon = 1;
}
openlog('display', 'pid', 'local5');
mylog(LOG_INFO, "display starting with pid " . $$);
#switch user
if (defined($uid)) {
	POSIX::setgid($gid);
	POSIX::setuid($uid);
}

END {update_led("--", "today"); update_led("--", "tomorow") }

while (1) {
	my $lum = 0;
	my $c_sock = new IO::Socket::UNIX (
		Type => SOCK_STREAM,
		Peer => $c_sockpath
	) or mylog(LOG_ERR, "can't connect to $c_sockpath: $!");
	if (defined($c_sock)) {
		setsockopt($c_sock, SOL_SOCKET, SO_KEEPALIVE, 1);
		my $seen = 0;

		while ((my $line = <$c_sock>) && $seen < 1) {
			chop $line;
			if ($line =~ /^(\d+) (\S+) (\S+)$/) {
				my $name = $2;
				my $value = $3;
				if ($name eq "CLUM") {
					$lum = $value;
					$seen++;
				}
			} else {
				mylog(LOG_DEBUG, "c_sock $line") if $debug > 1;
			}
		}
		close($c_sock);
	}
	mylog(LOG_DEBUG, "lum $lum") if $debug;
	if ($lum <= $lum_min) {
		update_led("--", "today");
		update_led("--", "tomorow");
		next;
	}

	my $e_sock = new IO::Socket::UNIX (
		Type => SOCK_STREAM,
		Peer => $e_sockpath
	) or mylog(LOG_ERR, "can't connect to $e_sockpath: $!");
	if (defined($e_sock)) {
		setsockopt($e_sock, SOL_SOCKET, SO_KEEPALIVE, 1);
		my $seen = 0;


		while ((my $line = <$e_sock>) && $seen < 2) {
			chop $line;
			if ($line =~ /^(\d+) (\S+) (\S+)$/) {
				my $name = $2;
				my $value = substr($3, 2);
				if ($name eq "PTEC") {
					update_led($value, "today");
					$seen++;
				}
				if ($name eq "DEMAIN") {
					update_led($value, "tomorow");
					$seen++;
				}
			} else {
				mylog(LOG_DEBUG, "e_sock $line") if $debug > 1;
				flush STDOUT;
			}
		}
		close($e_sock);
	} else {
		update_led("--", "today");
		update_led("--", "tomorow");
	}
	sleep 60;
}

sub update_led
{
	my %led_cmds = (
	    'today' => {
		'--' => ["D2_R 0", "D2_G 0", "D2_B 0"],
		'JB' => ["D2_R 0", "D2_G 0", "D2_B 1"],
		'JW' => ["D2_R 1", "D2_G 1", "D2_B 1"],
		'JR' => ["D2_R 1", "D2_G 0", "D2_B 0"],
	      },
	    'tomorow' => {
		'--' => ["D1_R 0", "D1_G 0", "D1_B 0"],
		'JB' => ["D1_R 0", "D1_G 0", "D1_B 1"],
		'JW' => ["D1_R 1", "D1_G 1", "D1_B 1"],
		'JR' => ["D1_R 1", "D1_G 0", "D1_B 0"],
	      }
	);

	my ($color, $day) = @_;
	my $cmds = $led_cmds{$day}{$color};
	if (!@$cmds) {
		mylog(LOG_DEBUG, "wrong day $day or color $color");
	} else{
		for my $cmd (@$cmds) {
			if ($debug) {
				mylog(LOG_DEBUG, "/usr/sbin/gpioctl gpio0 " . $cmd);
			}
			system("/usr/sbin/gpioctl -q gpio0 " . $cmd);
		}
	}
}

sub usage {
	print STDERR "usage: display [-u <uid>] [-g <gid>] [-c <path>] [-e <path>] [-d level] [-F] [-P <file>] [-C]\n";
	exit 1;
}

sub mylog {
	my ($level, $str) = @_;
	if ($isdaemon != "1") {
		print STDERR "$str\n";
	}
	syslog $level, $str;
}

sub mydie {     
	mylog(LOG_CRIT, @_);
	update_led("--", "today");
	update_led("--", "tomorow");
	exit(1);
}

sub exit_handler {
	update_led("--", "today");
	update_led("--", "tomorow");
	exit(0);
}
