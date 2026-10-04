#!/usr/pkg/bin/perl

# Copyright (c) 2026 Manuel Bouyer
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
use Time::Local;
use IO::Socket::UNIX;
use Getopt::Std;

my $s_sockpath = "/var/run/s.sock";

my $s_sock;

our($opt_s);
getopts("s:") or usage();
usage() unless  (@ARGV == 1); 

$s_sockpath = $opt_s if defined($opt_s);

$s_sock = new IO::Socket::UNIX (
	Type => SOCK_STREAM,
	Peer => $s_sockpath
);
die("Could not open $s_sockpath socket: $!") unless $s_sock;

my $now = time();

if ($ARGV[0] =~ /^dump$/) {
	print $s_sock "dump\n";
	$s_sock->flush;
	while (my $l = <$s_sock>) {
		print "$l";
		chomp $l;
		if ($l =~ /^OK$/) {
			exit(0);
		}
	}
};
if ($ARGV[0] =~ /^\s*([+\d]+) ([+\d]+)\s+(.*\S)\s*$/) {
	my $start = $1;
	my $end = $2;
	my $cmd = $3;
	if ($start =~ /^\+(\d+)$/) {
		$start = $now + $1 * 60;
	}
	if ($end =~ /^\+(\d+)$/) {
		$end = $now + $1 * 60;
	}
	print $s_sock "$start $end $cmd\n";
	$s_sock->flush;
	my $l = <$s_sock>;
	chomp $l;
	if ($l =~ /^OK$/) {
		exit(0);
	}
	print STDERR "$start $end $cmd returned error\n";
	exit(1);
}
print STDERR $ARGV[0] . " not parsed\n";
exit(1);

sub usage {
	print STDERR "usage: schedule [-s <path>] <cmd>\n";
	exit 1;
}
