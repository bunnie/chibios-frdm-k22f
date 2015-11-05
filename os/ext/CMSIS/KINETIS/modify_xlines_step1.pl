#!/usr/bin/perl
use strict;

my $infile = "mk22f12.h";
my $outfile = $infile . ".intermediate";

open (IN, $infile) || die "can't open $infile";
open (OUT, ">$outfile") || die "can't open $outfile";

my $line;
my $replace_portd = 0;
my $mask_value = "";
my $var_name = "";
my $var_address = ""; 
my $num_replaced = 0;
my ($mask_line, $shift_line, $width_line, $x_line, $expected_x_var) = "";
my $type = "";

while ($line = <IN>) {
	my @fields = split (/\s+/, $line);
	
	if ($fields[1] =~ /_MASK$/) {
		$mask_value = $fields[2];
		$fields[1] =~ /(\S+)_MASK$/; 
		$var_name = $1;
		$var_address = $fields[2];
		$mask_line = $line;
		print OUT $mask_line;
#		print $var_name . " from " . $fields[1] . "\n";	
	} elsif ($fields[1] eq $var_name . "_SHIFT") {
		$shift_line = $line;
		print OUT $shift_line;
	} elsif ($fields[1] eq $var_name . "_WIDTH") {
		if ($fields[2] == 1) {
			$replace_portd = 1;
		} else { 
			$replace_portd = 0;
		}
		$width_line = $line;	
		print OUT $width_line;
	} elsif ($fields[1] eq $var_name . "(x)") {
#	} elsif ($fields[2] =~ /PORTD\(x\)$/) { 
#		print "replace id\n";
        	$x_line  = $line;
	        $type = "";
		if($fields[2] =~ /uint8_t/) {
		    $type = "uint8_t";
		} elsif($fields[2] =~ /uint16_t/) {
		    $type = "uint16_t";
		} elsif($fields[2] =~ /uint32_t/) {
		    $type = "uint32_t";
		}
		if ($replace_portd == 1) {
		    if($type) {
			$x_line = "#define " . $var_name . "\t\t((" . $type . ")" . $var_address . ")\n";
		    } else {
			$x_line = "#define " . $var_name . "\t\t" . $var_address . "\n";
		    }
#			$width_line = "";
#			$mask_line = "";
#			$shift_line = "";
#			print "replacing \n";
			$num_replaced++;			

		}
#		print "CHECK replace_portd = $replace_portd for $var_name\n"; 
#		print OUT $mask_line . $shift_line . $width_line . $x_line; 
		#re-initialize
		print OUT $x_line;
		$mask_line = "";
		$shift_line = $width_line = $x_line = "";
		$replace_portd = 0;
                $var_name = "";
                $var_address = "";

	} else {
		print OUT $line;
	}

}

print "number_replaced is $num_replaced. CHECK exactly " . ($num_replaced *3) . "lines have been removed in step 2\n";

close (IN);
close (OUT);
 
