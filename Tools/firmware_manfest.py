#!/usr/bin/env python3
############################################################################
#
#   Copyright (C) 2020 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

#
# PX4 firmware image generator
#
# The PX4 firmware file is a JSON-encoded Python object, containing
# metadata fields and a zlib-compressed base64-encoded firmware image.
#

import argparse
import json
import base64
import zlib
import time
import subprocess


# command line option  -- official, beta, dev
#    append to existing


#  separately upload metadata?

# board_id
# mav-type: Copter, "Default"? Skip?
# format: "ELF", "abin", "apj", "hex", "px4", "bin"
# url:
# mav-firmware-version-type: OFFICIAL, BETA, DEV
# USBID: eg 0x1209/0x5740   0x26AC/0x0032
# mav-firmware-version: eg "4.0.5"
# bootloader_str: eg "PX4 BL FMU v5.x" or Pixhawk4-BL
# latest: 0 or 1
# platform: Pixhawk4 or px4_fmu-v5?
# brand_name: "CUAV X7"



# "mav-autopilot": "ARDUPILOTMEGA",
# "git-sha": "e22170628d5a03a18c0445c5af2d3f3688c37ed4",
# "image_size": 989600,
# "manufacturer": "Holybro", Hex/ProfiCNC,  BOARD_VENDOR


# from .px4: image_size, board_id, board_revision

# AP_HW_DURANDAL                        139  = 0x8B
# USB_VENDOR 0x3162                  = CONFIG_CDCACM_VENDORID
# USB_PRODUCT 0x004E                 = CONFIG_CDCACM_PRODUCTID
# USB_STRING_MANUFACTURER "Holybro"  = CONFIG_CDCACM_VENDORSTR
# USB_STRING_PRODUCT "pix32v5"   or USB_STRING_PRODUCT "pix32v5-BL" = CONFIG_CDCACM_PRODUCTSTR?

# make TARGET update_manifest  + manifest.json.gz
#    s3 upload?   master/beta/stable + version tags

# "build_time": 1606680557,
# "summary": "PX4FMUv4",
# "version": "0.1",
# "image_size": 1886757,
# "image_maxsize": 2080768,
#"git_identity": "b'v1.11.0-rc3-745-g5f0ecc893e'",
#"git_hash": "b'5f0ecc893e915116b65c373cc45a66481ff22666'",

# make artifacts
# artifacts/manifest.json
# artifacts/manifest.json.gz
# artifacts/v1.11.0

# set latest=1 if most recent tag

firmware_dict = {}
firmware_dict["firmware"] = []

with open('manifest.json') as f:
  firmware_dict = json.load(f)

firmware_entry = {}
firmware_entry["mav_autopilot"] = 12 # 12 = MAV_AUTOPILOT_PX4
firmware_entry["mav_type"] = "Generic"  # CAN_PERIPHERAL
firmware_entry["mav-firmware-version-type"] = "BETA"
firmware_entry["mav-firmware-version"] = "1.11.2"
firmware_entry["git-sha"] = "abcdefg"
firmware_entry["format"] = "px4"
firmware_entry["brand_name"] = "BOARD_VENDOR"    # pretty name?
firmware_entry["platform"] = "BOARD_VENDOR_BOARD_MODEL"
firmware_entry["USBID"] = "0x1234/0x5678" # CONFIG_CDCACM_PRODUCTID/CONFIG_CDCACM_VENDORID
firmware_entry["url"] = "TODO"
firmware_entry["latest"] = 0

firmware_dict["firmware"].append(firmware_entry)

firmware_dict["format-version"] = "1.0.0"

# Pretty Printing JSON string back
#print(json.dumps(firmware_dict, indent=4, sort_keys=True))

# appending the data
firmware_dict.update(firmware_dict)

with open('manifest.json', 'w') as json_file:
  json.dump(firmware_dict, json_file, indent=4)


# Parse commandline
parser = argparse.ArgumentParser(description="Firmware generator for the PX autopilot system.")
parser.add_argument("--prototype", action="store", help="read a prototype description from a file")
parser.add_argument("--board_id", action="store", help="set the board ID required")
parser.add_argument("--version",	action="store", help="set a version string")
parser.add_argument("--git_identity",	action="store", help="the working directory to check for git identity")
args = parser.parse_args()

# Fetch the firmware descriptor prototype if specified
if args.prototype != None:
	f = open(args.prototype,"r")
	desc = json.load(f)
	f.close()
else:
	desc = mkdesc()

if args.board_id != None:
	desc['board_id']	= int(args.board_id)
if args.board_revision != None:
	desc['board_revision']	= int(args.board_revision)
if args.version != None:
	desc['version']		= str(args.version)
if args.summary != None:
	desc['summary']		= str(args.summary)
if args.description != None:
	desc['description']	= str(args.description)
if args.git_identity != None:
	cmd = "git --git-dir '{:}/.git' describe --always --tags".format(args.git_identity)
	p = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE).stdout
	desc['git_identity']	= str(p.read().strip())
	p.close()
	cmd = "git --git-dir '{:}/.git' rev-parse --verify HEAD".format(args.git_identity)
	p = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE).stdout
	desc['git_hash']	= str(p.read().strip())
	p.close()
if args.parameter_xml != None:
	f = open(args.parameter_xml, "rb")
	bytes = f.read()
	desc['parameter_xml_size'] = len(bytes)
	desc['parameter_xml'] = base64.b64encode(zlib.compress(bytes,9)).decode('utf-8')
	desc['mav_autopilot'] = 12 # 12 = MAV_AUTOPILOT_PX4
if args.airframe_xml != None:
	f = open(args.airframe_xml, "rb")
	bytes = f.read()
	desc['airframe_xml_size'] = len(bytes)
	desc['airframe_xml'] = base64.b64encode(zlib.compress(bytes,9)).decode('utf-8')
if args.image != None:
	f = open(args.image, "rb")
	bytes = f.read()
	desc['image_size'] = len(bytes)
	desc['image'] = base64.b64encode(zlib.compress(bytes,9)).decode('utf-8')

print(json.dumps(desc, indent=4))
