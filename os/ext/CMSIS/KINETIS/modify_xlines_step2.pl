#!/usr/bin/perl
use strict;

my $infile = "mk22f12.h.intermediate";
my $outfile = $infile . ".final";

open (IN, $infile) || die "can't open $infile";
open (OUT, ">$outfile") || die "can't open $outfile";

my @lines = <IN>;

my $num_removed = 0;

#starting at 3 because it must be preceded by 3 lines
for (my $i = 3; $i < @lines; $i++) {
	my @fields = split (/\s+/, $lines[$i]);
	my $var_name = $fields[1];	
	my $mask_name = $var_name . "_MASK";
	my $shift_name = $var_name . "_SHIFT";
	my $width_name = $var_name . "_WIDTH";

	my @maybe_mask_fields = split (/\s+/, $lines[$i-3] );
	my @maybe_shift_fields = split (/\s+/, $lines[$i-2] );
	my @maybe_width_fields = split (/\s+/, $lines[$i-1] );

	if ($maybe_mask_fields[1] eq $mask_name && $maybe_shift_fields[1] eq $shift_name && $maybe_width_fields[1] eq $width_name) {
		$lines[$i-3] = "";
		$lines[$i-2] = "";
		$lines[$i-1] = "";
		$num_removed += 3;
	}


}

print OUT @lines;
print "number_removed is $num_removed. CHECK exactly matches lines to be removed in step 1\n";

close (IN);
close (OUT);
 
