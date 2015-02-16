#!/usr/bin/python

import sys
import os
import re

# Set home directory
PAPARAZZI_HOME = os.path.realpath(os.path.dirname(os.path.dirname(os.path.realpath(__file__))) + '../../../')

# Aircraft name substituted in arguments
#if not sys.argv[2] == "-MMD" :

# Only look at gcc command
if not (sys.argv[2] == "-MMD"):
	quit()

AIRCRAFT = sys.argv[1]

# Update config+includes only once (this script is called many times, but config and includes are always the same)
if not os.path.isfile(PAPARAZZI_HOME + "/" + AIRCRAFT + ".config"):
	print "Generating config and includes file"
	config   = open(PAPARAZZI_HOME + "/" + AIRCRAFT + ".config", "w")
	includes = open(PAPARAZZI_HOME + "/" + AIRCRAFT + ".includes", "w")
	test_config   = open(PAPARAZZI_HOME + "/tests/unittest/" + AIRCRAFT + "Tester.config", "w")
	test_includes = open(PAPARAZZI_HOME + "/tests/unittest/" + AIRCRAFT + "Tester.includes", "w")

	for argument in sys.argv:
		matchObj = re.match(r'-D(.+)=(.*)', argument)
		if(matchObj) :
			newDefine = '#define ' + matchObj.group(1) + ' ' + matchObj.group(2) + '\n'
			newDefine = newDefine.replace('\\"', '"')
			newDefine = newDefine.replace('\'', '')
			config.write(newDefine)
			test_config.write(newDefine)
			continue

		matchObj = re.match(r'-D(.+)', argument)
		if(matchObj) :
			newDefine = '#define ' + matchObj.group(1) + ' 1\n'
			config.write(newDefine)
			test_config.write(newDefine)
			continue

		matchObj = re.match(r'-I(.+)', argument)
		if(matchObj) :
			newInclude = os.path.normpath(os.path.join(PAPARAZZI_HOME, './sw/airborne', matchObj.group(1)))
			includes.write(newInclude + '\n')
			test_includes.write(newInclude + '\n')
			newInclude = os.path.normpath(os.path.join(PAPARAZZI_HOME + "/tests/unittest", './sw/airborne', matchObj.group(1)))
			test_includes.write(newInclude + '\n')
			#incc.write('-I' + newInclude + ' ')

	# extra includes for unittest
	test_includes.write('/home/bart/unity/src\n')
	test_includes.write('/home/bart/unity/extras/fixture/src\n')
	# close the project files
	config.close()
	includes.close()
	test_config.close()
	test_includes.close()

# The files-file must be update every every call, to resolve all dependencies
# First, extract all files currently in there
files_file = open(PAPARAZZI_HOME + "/" + AIRCRAFT + ".files", "r")
all_files = files_file.read().split('\n')
all_files.remove('')
discovered_files = []

# Generate includeflags from projects includes file
includes = open(PAPARAZZI_HOME + "/" + AIRCRAFT + ".includes")
includescontent = includes.read().split('\n')
includescontent.remove('')
includeflags = ''
for incdir in includescontent:
	includeflags += '-I' + incdir + ' '
includes.close()

# Extract c file from arguments
for argument in sys.argv:
	if(argument.endswith('.c')):
		c_file = PAPARAZZI_HOME + "/sw/airborne/" + argument
		discovered_files.append(c_file)
		
		# For this c-file, resolve all dependencies
		cmd = "gcc -MM " + c_file + " " + includeflags + " -include" + PAPARAZZI_HOME + "/" + AIRCRAFT + ".config"
		headers = os.popen(cmd).read().split('\\')
		# Start at 2 because 0=.c and 1=.config
		for i in range(2, len(headers)-1):
			header = headers[i].strip()
			discovered_files.append(header)

# Append discovered files to all files
for newfile in discovered_files:
	if(all_files.count(newfile) == 0):
		all_files.append(newfile)

# Rewrite the files-file
files_file.close()
files_file = open(PAPARAZZI_HOME + "/" + AIRCRAFT + ".files", "w")
for xfile in all_files:
	files_file.write(xfile + '\n')

files_file.close()
