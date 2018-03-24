#!/usr/bin/python

import glob
import os
import sys
import re

# This script is run from Build/<target>_default.build/$(PX4_BASE)/Firmware/src/systemcmds/topic_listener

# argv[1] must be the full path of the top Firmware dir
# argv[2] (optional) is the full path to the EXTERNAL_MODULES_LOCATION

raw_messages = glob.glob(sys.argv[1]+"/msg/*.msg")
if len(sys.argv) > 2:
	external_raw_messages = glob.glob(sys.argv[2]+"/msg/*.msg")
	raw_messages += external_raw_messages # Append the msgs defined in the EXTERNAL_MODULES_LOCATION to the normal msg list
messages = []
topics = []
message_elements = []

# large and not worth printing (find better solution)
raw_messes = [raw_messages.remove(x) for x in raw_messages if 'qshell_req' in x]
raw_messes = [raw_messages.remove(x) for x in raw_messages if 'ulog_stream' in x]
raw_messes = [raw_messages.remove(x) for x in raw_messages if 'gps_inject_data' in x]
raw_messes = [raw_messages.remove(x) for x in raw_messages if 'gps_dump' in x]

for index,m in enumerate(raw_messages):
	topic_list = []
	f = open(m,'r')
	for line in f.readlines():
		items = re.split('\s+', line.strip())

		if '# TOPICS' == ' '.join(items[:2]):
			for topic in items[2:]:
				topic_list.append(topic)

	f.close()

	(m_head, m_tail) = os.path.split(m)
	message = m_tail.split('.')[0]

	if len(topic_list) == 0:
		topic_list.append(message)

	for topic in topic_list:
		messages.append(message)
		topics.append(topic)

num_messages = len(messages);

print("""

/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file topic_listener.cpp
 *
 * Autogenerated by Tools/generate_listener.py
 *
 * Tool for listening to topics when running flight stack on linux.
 */

#include <string.h>

#include <px4_defines.h>
#include <uORB/uORB.h>
#include <systemcmds/topic_listener/listener_generated.hpp>
#include <systemcmds/topic_listener/listener_print.hpp>

""")
for m in set(messages):
	print("#include <uORB/topics/%s.h>" % m)

print ("""
int listener_generated(char* name, unsigned num_msgs, unsigned interval, unsigned instance) {

""")

for index, (m, t) in enumerate(zip(messages, topics)):
	if index == 0:
		print("\tif (strncmp(name, \"%s\", %d) == 0) {" % (t, len(t)))
	else:
		print("\t} else if (strncmp(name, \"%s\", %d) == 0) {" % (t, len(t)))

	print("\t\t if (orb_exists(ORB_ID(%s), instance) != 0) { PX4_WARN(\"%s: %%d never published\", instance); return -1; }" % (t, m))
	print("\t\t PX4_INFO(\"%s instance %%d\", instance);" % t)
	print("\t\t uORB::Subscription<%s_s> subscription{ORB_ID(%s), interval, instance};" % (m, t))
	print("\t\t listener_print(subscription, num_msgs);")

print("\t} else {")
print("\t\t PX4_WARN(\" Topic did not match any known topics\");")
print("\t}")

print("\t return 0;")
print("}\n")
