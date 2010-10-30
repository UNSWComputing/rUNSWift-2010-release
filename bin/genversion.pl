#!/usr/bin/perl -w

print "#include <string>\n";
my $tag = `git describe --always`;
chomp $tag;
print "std::string git_version = \"$tag\";\n";

