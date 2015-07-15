#!/usr/bin/perl -w

##############################################################################
#
# Program    :  write_char_set.pl
# Author     :  Philippe Vanhaesendonck
# Description:  Uploads MAX7456 Character Set on MinimOSD with MinOPOSD
# 	            firmware (Probably works on original MinimOSD as well)
#
#############################################################################
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
# or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
# for more details.
#
# You should have received a copy of the GNU General Public License along
# with this program; if not, see <http://www.gnu.org/licenses/> or write to the 
# Free Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
#
#############################################################################

use strict;
use Getopt::Std;
use Device::SerialPort;

# Get character set and device
our($opt_c, $opt_d);
if ( ! getopts('c:d:') )
{
  die "Usage: write_char_set.pl -c charset.mcm -d device\n";
}
if ( !$opt_c || !$opt_d ) {
  die "Usage: write_char_set.pl -c charset.mcm -d device\n";
}

# Open and validate character Set file
open(CHARSET, "<:crlf", $opt_c)
  or die "cannot open $opt_c: $!";
chomp(my $line = <CHARSET>);
$line eq 'MAX7456' 
  or die "Invalid font file\n";

# Open and setup the serial device
my $port = Device::SerialPort->new($opt_d) 
  or die "Can't open $opt_d: $!\n";
$port->databits(8);
$port->baudrate(57600);
$port->parity("none");
$port->stopbits(1);
$port->handshake("none");

# Reboot the board and wait for the 'flashing window':
print "Restarting OSD\n";
$port->pulse_dtr_off(50);
$port->pulse_rts_off(50);
sleep 7;
$port->purge_rx;

# Send the 'magic words' to enter font mode.
$port->write("\n");
$port->write("\n");
$port->write("\n");
$port->write("\n");
$port->write("\n");
$port->write("\n");

# Read prodile: 0.5 sec total timeout, no wait between chars
$port->read_char_time(0);
$port->read_const_time(500);
 
# Wait ready message from MinimOSD
print "Waiting for OSD to be ready...\n";

my $timeout = 12;
my $buffer = "";
while ($timeout>0) {
  my ($count,$saw) = $port->read(255);
  if ($count > 0) {
    $buffer .= $saw;
    # Check if we got ready message
    $buffer =~ /Ready for Font/ && last;
  } else {
    $timeout--;
  }
}
if ($timeout == 0) {
  die "Error entering font mode - No Data\n";
}


# Upload all characters.
# We have 256 characters to upload, each of them described by 64 bytes 
# Each byte is a binary acii string on a separete line

# Read prodile: 0.01 sec total timeout, no wait between chars
$port->read_char_time(0);
$port->read_const_time(10);
# Unbuffer stdout for feedback
$|=1;

print "OSD ready starting upload.\n";

# The parsing starts after a CR, so we nned to send one before sending data...
$port->write("\r");

# Character loop
my $character = 256;
while ($character--) {
  # Byte loop
  my $byte = 64;
  while ($byte--) {
    defined ($line = <CHARSET>) or die "Unexpected en of Charset file\n";
	chomp $line;
    $port->write($line);
    $port->write("\r");
    $port->write_drain;
  }
  # Wait character acknowlegement from board
  $buffer = "";
  $timeout = 10;
  while ($timeout>0) {
    my ($count,$saw) = $port->read(255);
    if ($count > 0) {
      $buffer .= $saw;
      # Check for ack message
      $buffer =~ /Char Done/ && last;
    } else {
      $timeout--;
    }
  }
  if ($timeout == 0) {
    die "\nCharSet upload failed - no response\n";
  }
  printf "Loading char %03d\r", 256 - $character;
}
print "\nAll done, restarting OSD\n";
$port->pulse_dtr_off(50);
$port->pulse_rts_off(50);
sleep 1;

# Cleanup
$port->close;
close(CHARSET);
print "Exiting\n";
