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
test_config_file   = PAPARAZZI_HOME + "/tests/unittest/" + AIRCRAFT + "Tester.config"
test_includes_file = PAPARAZZI_HOME + "/tests/unittest/" + AIRCRAFT + "Tester.includes"
if os.path.isfile(config_file):
	os.remove(config_file)
if os.path.isfile(includes_file):
	os.remove(includes_file)
if os.path.isfile(test_config_file):
	os.remove(test_config_file)
if os.path.isfile(test_includes_file):
	os.remove(test_includes_file)

# create empty files file
if os.path.isfile(files_file):
	os.remove(files_file)
new_files_file = open(files_file, "w")
new_files_file.close()

# Then, do a full build on the aircraft, but replace compiler with python script
print "Analyzing make output.."
cmd = "make -f Makefile.ac PAPARAZZI_HOME=" + PAPARAZZI_HOME + " PAPARAZZI_SRC=" + PAPARAZZI_HOME + " AIRCRAFT=" + AIRCRAFT + " CC='python " + PAPARAZZI_HOME + "/sw/tools/qtcreator_ide_config/gcc_interpeter.py " + AIRCRAFT + "' CP='python " + PAPARAZZI_HOME + "/sw/tools/qtcreator_ide_config/skip_command.py' LD='python " + PAPARAZZI_HOME + "/sw/tools/qtcreator_ide_config/skip_command.py' DMP='python " + PAPARAZZI_HOME + "/sw/tools/qtcreator_ide_config/skip_command.py' NM='python " + PAPARAZZI_HOME + "/sw/tools/qtcreator_ide_config/skip_command.py' NPROCS=1 ap.compile"
print os.popen(cmd).read()

# Get all the defines from config file
UNITTEST_HOME = PAPARAZZI_HOME + "/tests/unittest"
test_config_file = open(PAPARAZZI_HOME + "/tests/unittest/" + AIRCRAFT + "Tester.config")
macro_text = test_config_file.read()
test_config_file.close()

macro_flags = macro_text.split('\n')
for i, s in enumerate(macro_flags):
	macro_flags[i] = s.replace("#define ", "    - -D").replace(" ", "='").replace("='='='='-='-D", "    - -D") + "'" #beun

macro_flags.pop()

# Get include directories from testing includes file
test_includes_file = open(UNITTEST_HOME + "/" + AIRCRAFT + "Tester.includes")
test_includes_content = test_includes_file.read().split('\n')
for i, s in enumerate(test_includes_content):
	test_includes_content[i] = "      - '" + s + "/'"
yml_include_items = '\n'.join(test_includes_content)

# Configure the yml script for unit testing
original_yml = open(UNITTEST_HOME + "/ymlmodel")
original_content = original_yml.read()
original_yml.close()

yml_macro_flags = '\n'.join(macro_flags)
new_content = original_content.replace("MACRO_ITEMS", yml_macro_flags).replace("INCLUDE_ITEMS", yml_include_items)
new_yml = open(UNITTEST_HOME + "/" + AIRCRAFT + ".yml", "w")
new_yml.write(new_content)
new_yml.close()
