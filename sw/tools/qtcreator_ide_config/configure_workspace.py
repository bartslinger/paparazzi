#!/usr/bin/python

import sys
import os
import time

# Set home directory
PAPARAZZI_HOME = os.path.realpath(os.path.dirname(os.path.dirname(os.path.realpath(__file__))) + '../../../')

# Aircraft name is argument and should be equal to project name
AIRCRAFT = sys.argv[1]

# Check if the IDE files exist with that name
if not os.path.isfile(PAPARAZZI_HOME + "/" + AIRCRAFT + ".creator"):
	raise Exception("Aircraft name does not match project name.")

# First, do make clean on the aircraft
cmd = "make -C " + PAPARAZZI_HOME + " -f Makefile.ac AIRCRAFT=" + AIRCRAFT + " clean_ac"
print "========================"
print "Cleaning"
reply = os.popen(cmd).read()
#print reply
print "========================"

# Remove project files
# If the interpeter runs for the first time, it regenerates these files. Only the first time
config_file   = PAPARAZZI_HOME + "/" + AIRCRAFT + ".config"
includes_file = PAPARAZZI_HOME + "/" + AIRCRAFT + ".includes"
files_file    = PAPARAZZI_HOME + "/" + AIRCRAFT + ".files"
if os.path.isfile(config_file):
	os.remove(config_file)
if os.path.isfile(includes_file):
	os.remove(includes_file)

# create empty files file
if os.path.isfile(files_file):
	os.remove(files_file)
new_files_file = open(files_file, "w")
new_files_file.close()

# Then, do a full build on the aircraft, but replace compiler with python script
print "Analyzing make output.."
cmd = "make -f Makefile.ac PAPARAZZI_HOME=" + PAPARAZZI_HOME + " PAPARAZZI_SRC=" + PAPARAZZI_HOME + " AIRCRAFT=" + AIRCRAFT + " CC='python " + PAPARAZZI_HOME + "/sw/tools/qtcreator_ide_config/gcc_interpeter.py " + AIRCRAFT + "' CP='python " + PAPARAZZI_HOME + "/sw/tools/qtcreator_ide_config/skip_command.py' LD='python " + PAPARAZZI_HOME + "/sw/tools/qtcreator_ide_config/skip_command.py' DMP='python " + PAPARAZZI_HOME + "/sw/tools/qtcreator_ide_config/skip_command.py' NM='python " + PAPARAZZI_HOME + "/sw/tools/qtcreator_ide_config/skip_command.py' NPROCS=1 ap.compile"
print os.popen(cmd).read()

