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
use Safe;
use Schedule::Cron::Events;
use Time::Local;
use IO::Socket::UNIX;
use Socket qw(SOL_SOCKET SO_KEEPALIVE);
use IO::Select;
use Getopt::Std;

use Proc::Daemon;
use Sys::Syslog qw(:standard :macros);

my $c_sockpath = "/var/run/c_hub.sock";
my $e_sockpath = "/var/run/e_hub.sock";
my $s_sockpath = "/var/run/s.sock";

my ($c_sock, $e_sock, $s_sock);
my @clients;

my $debug = 0;
my $isdaemon = 0;
my ($uid, $gid);

openlog('scheduler', 'pid', 'local5');

our($opt_c, $opt_e, $opt_d, $opt_s, $opt_F, $opt_P, $opt_u, $opt_g);
getopts("c:e:s:d:FP:u:g:") or usage();
usage() unless  (@ARGV == 1); 

$c_sockpath = $opt_c if defined($opt_c);
$e_sockpath = $opt_e if defined($opt_e);
$s_sockpath = $opt_s if defined($opt_s);
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

open SCHED, '<', $ARGV[0] or mydie("can't open $ARGV[0]: $@");
my $err = 0;

my @events;
my %state;
my @c_pending_cmd;
my @e_pending_cmd;

my @expected = ( 'CuSTAT', 'SaSTAT', 'ChSTAT', 'SbSTAT', 'PTEC', 'DEMAIN',
    'O0', 'O1', 'P0', 'P1', 'P2', 'P3');
my @cmd_capteur = ('Cu', 'Sa', 'Ch', 'Sb');
my @cmd_energie = ('O0', 'O1', 'P0', 'P1', 'P2', 'P3');

print "sched $ARGV[0] capteur $c_sockpath energie $e_sockpath sched $s_sockpath debug $debug\n" if $debug;

while (my $line = <SCHED>) {
	chop $line;
	next if $line =~ /^\s*#/;
	if ($line =~ /^\s*(\S+[0-9,\-\*\s]+\S+)\s*<>\s*(\S+[0-9,\-\*\s]+\S+)\s*\|\s*(\S.*)$/) {
		print "start $1 end $2 cmd \"$3\"\n" if $debug;
		my $start = new Schedule::Cron::Events($1) or mydie("can't schedule $1");
		my $end = new Schedule::Cron::Events($2) or mydie("can't schedule $2");
		my $cmd = $3;
		my $comp = new Safe;
		$comp->permit_only(qw(:base_core :base_mem :base_orig));
		$comp->share(%state);
		my $res = $comp->reval($cmd);
		if (!defined($res)) {
			if (defined($@)) {
				mylog(LOG_ERR, "<$cmd> failed: $@");
			} else {
				mylog(LOG_ERR, "<$cmd> failed");
			}
			$err = 1;
		} else {
			push @events,
			    { 'start' => $start, 'end' => $end, 'cmd' => $cmd};
		}
	} else {
		mylog(LOG_ERR, "invalid entry $line");
		$err = 1;
	}
}
close SCHED;
closelog();
exit 1 if ($err > 0);

unlink $s_sockpath;
$s_sock = new IO::Socket::UNIX (
	Type => SOCK_STREAM,
	Local => $s_sockpath,
	Listen => 16,
	Reuse => 1,
);
mydie("Could not create $s_sockpath socket: $!") unless $s_sock;

if (defined($uid)) {
	chown $uid, $gid, $s_sockpath;
	chmod 0660, $s_sockpath;
}

if ($opt_F ne "1") {
        my $daemon;
        if (defined($opt_P)) {
		$daemon = Proc::Daemon->new(
		    pid_file => $opt_P,
		    dont_close_fh => [ $s_sock ]
		);
        } else {
		$daemon = Proc::Daemon->new( dont_close_fh => [ $s_sock ]);
        }
        my $pid = $daemon->init;      
        if ($pid) {
                exit(0);
        }
        $isdaemon = 1;
}
openlog('scheduler', 'pid', 'local5');
mylog(LOG_INFO, "scheduler starting with pid " . $$);
#switch user
if (defined($uid)) {
	POSIX::setuid($uid);
	POSIX::setgid($gid);
}

my $next = time();
while (1) {
	if (!defined($c_sock) && defined($e_sock)) {
		close($e_sock);
	}
	if (defined($c_sock) && !defined($e_sock)) {
		close($c_sock);
	}
	if (!defined($c_sock)) {
		$c_sock = new IO::Socket::UNIX (
		    Type => SOCK_STREAM,          
		    Peer => $c_sockpath
		);
		mylog(LOG_ERR, 
		    "can't connect do $c_sockpath: $!") unless $c_sock;
	}
	if (!defined($e_sock)) {
		$e_sock = new IO::Socket::UNIX (
		    Type => SOCK_STREAM,          
		    Peer => $e_sockpath
		);
		mylog(LOG_ERR, 
		    "can't connect do $e_sockpath: $!") unless $e_sock;
	}
	if (!defined($c_sock) || !defined($e_sock)) {
		sleep(10);
		$next = time();
		next;
	}
	do_select($next);
	my $valid = 1;
	foreach my $e (@expected) {
		if (!defined($state{$e})) {
			mylog(LOG_DEBUG, "state $e missing") if $debug;
			$valid = 0;
		} elsif ($state{$e}{t} < $next - 300) {
			mylog(LOG_DEBUG, "state $e old") if $debug;
			$valid = 0;
		}
	}
	$next = $next + 60;
	if ($debug > 1) {
		foreach my $key ( sort keys %state ) {
			mylog(LOG_DEBUG, "state $key $state{$key}{v} $state{$key}{t}");
		}
	}
		
	run_schedules() if $valid == 1;
}
exit(0);

sub run_schedules {
	my %out;

	mylog(LOG_DEBUG, "schedule: " . @events . " entries") if $debug;

	my $now = time();

	for my $event (@events) {
		my $start = $event->{start};
		my $end = $event->{end};
		my $cmd = $event->{cmd};

		$start->setCounterToDate((localtime($now))[0..5]);
		$end->setCounterToDate((localtime($now))[0..5]);

		my $sprev = timelocal($start->previousEvent);
		my $eprev = timelocal($end->previousEvent);

		$start->setCounterToDate((localtime($now))[0..5]);
		$end->setCounterToDate((localtime($now))[0..5]);

		my $snext = timelocal($start->nextEvent);
		my $enext = timelocal($end->nextEvent);

		my $str = "$cmd: $sprev <> $eprev $snext <> $enext";
		if ($sprev >= $eprev && $snext >= $enext) {
			my $comp = new Safe;
			$comp->permit_only(
			    qw(:base_core :base_mem :base_orig));
			$comp->share(%state);
			my $result = $comp->reval($cmd);
			if (!defined($result)) {
				if (defined($@)) {
					mylog(LOG_ERR, "<$cmd> failed: $@");
				} else {
					mylog(LOG_ERR, "<$cmd> failed");
				}
			} else {
				$str = $str . " RUN {";
				foreach my $key ( keys %$result ) {
					$str = $str .  " $key => " . %{$result}{$key} . ",";
				}
				mylog(LOG_DEBUG, $str . "}") if $debug;
				%out = (%out, %$result);
			}
		} else {
			mylog(LOG_DEBUG, $str) if $debug;
		}
	}
	if ($debug > 1) {
		mylog(LOG_DEBUG, "out:");
		foreach my $key ( keys %out ) {
			mylog(LOG_DEBUG, "    $key => $out{$key}");
		}
	}
	foreach my $key (keys %out ) {
		if (grep $_ eq $key, @cmd_capteur) {
			my $statekey = $key . "STAT";
			if ($state{$statekey}{v} =~ /^([OF]),B/) {
				if ($out{$key} eq $1) {
					# reset state
					push @c_pending_cmd, $key . " " . $1;
				}
			} elsif ($state{$statekey}{v} ne $out{$key}) {
				push @c_pending_cmd, $key . " " . $out{$key};
			}
		} elsif (grep $_ eq $key, @cmd_energie) {
			push @e_pending_cmd, $key . " " . $out{$key};
		} else {
			mylog(LOG_NOTICE, "unknown key $key($out{$key}) from scheduler");
		}
	}

	if ($debug) {
		my $s = "c_pending_cmd:";
		foreach my $c (@c_pending_cmd) {
			$s = $s . " \"$c\"";
		}
		mylog(LOG_DEBUG, $s);
		$s = "e_pending_cmd:";
		foreach my $c (@e_pending_cmd) {
			$s = $s . " \"$c\"";
		}
		mylog(LOG_DEBUG, $s);
	}
}

sub do_select {
	my ($endtime) = @_;
	my $read_set = new IO::Select();
	$read_set->add($c_sock);
	$read_set->add($e_sock);
	$read_set->add($s_sock);
	foreach my $c (@clients) {
		$read_set->add($c);
	}
	my $delay = $endtime - time();
	my $write_set = new IO::Select();

	while ($delay > 0) {
		$write_set->add($c_sock) if @c_pending_cmd > 0;
		$write_set->add($e_sock) if @e_pending_cmd > 0;

		my ($rh_set, $wh_set) = IO::Select->select($read_set, $write_set, undef, $delay);
		foreach my $rh (@$rh_set) {
			if ($rh == $s_sock) { #new client
				my $ns = $rh->accept();
				next unless defined($ns);
				push @clients, $ns;
				$read_set->add($ns);
			} elsif ($rh == $c_sock) {
				do_capteur($rh);
			} elsif ($rh == $e_sock) {
				do_energie($rh);
			} else {
				do_client($rh, $read_set);
			}
		}
		foreach my $wh (@$wh_set) {
			if ($wh == $c_sock) {
				my $c = pop @c_pending_cmd;
				print $c_sock "$c\n" or mylog(LOG_ERR, "can't write to capteur: $!");
				$c_sock->flush;
				$write_set->remove($c_sock);
			} elsif ($wh == $e_sock) {
				my $c = pop @e_pending_cmd;
				print $e_sock "$c\n" or mylog(LOG_ERR, "can't write to energie: $!");
				$e_sock->flush;
				$write_set->remove($e_sock);
			}
		}
		$delay = $endtime - time();
		return if (!defined($c_sock) || !defined($e_sock));
	}
}

sub do_capteur {
	my ($f) = @_;

	my $buf = <$f>;
	if ($buf) {
		chomp $buf;
		if ($buf =~ /^(\d+) (\S+STAT) (\S+)$/) {
			$state{$2}{v} = $3;
			$state{$2}{t} = $1;
		}
	} else {
		mylog(LOG_INFO, "$c_sockpath closed");
		close($c_sock);
		undef $c_sock;
	}
}

sub do_energie {
	my ($f) = @_;

	my $buf = <$f>;
	if ($buf) {
		chomp $buf;
		if ($buf =~ /^(\d+) (PTEC|DEMAIN) (\S+)$/) {
			$state{$2}{v} = $3;
			$state{$2}{t} = $1;
		} elsif ($buf =~ /^(\d+) EE (\d)(\d)(\d)(\d)(\d)(\d)$/) {
			$state{O0}{v} = $2;
			$state{O0}{t} = $1;
			$state{O1}{v} = $3;
			$state{O1}{t} = $1;
			$state{P0}{v} = $4;
			$state{P0}{t} = $1;
			$state{P1}{v} = $5;
			$state{P1}{t} = $1;
			$state{P2}{v} = $6;
			$state{P2}{t} = $1;
			$state{P3}{v} = $7;
			$state{P3}{t} = $1;
		}
	} else {
		mylog(LOG_INFO, "$e_sockpath closed");
		close($e_sock);
		undef $e_sock;
	}
}

sub do_client {
	my ($f, $read_set) = @_;

	my $buf = <$f>;
	if ($buf) {
		chomp $buf;
		print "got client $buf\n";
	} else {
		print "client close\n";
		$read_set->remove($f);
		@clients = grep { $_ != $f } @clients;
		close($f);
	}
}

sub usage {
	print STDERR "usage: scheduler [-u <uid>] [-g <gid>] [-c <path>] [-e <path>] [-s <path] [-d level] [-F] [-P <file>] <file>\n";
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
	exit(1);
}
