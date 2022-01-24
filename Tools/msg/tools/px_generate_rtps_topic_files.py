#!/usr/bin/env python3
#############################################################################
#
#   Copyright (C) 2013-2018 PX4 Pro Development Team. All rights reserved.
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
#############################################################################

"""
px_generate_rtps_topic_files.py
Generates c/cpp header/source files for uorb topics from .msg (ROS syntax)
message files
"""

import os
import shutil
import filecmp
import argparse
import re
import sys

try:
    import em
except ImportError as e:
    print("Failed to import em: " + str(e))
    print("")
    print("You may need to install it using:")
    print("    pip3 install --user empy")
    print("")
    sys.exit(1)

try:
    import genmsg.template_tools
except ImportError as e:
    print("Failed to import genmsg: " + str(e))
    print("")
    print("You may need to install it using:")
    print("    pip3 install --user pyros-genmsg")
    print("")
    sys.exit(1)

try:
    from packaging import version
except ImportError as e:
    print("Failed to import packaging: " + str(e))
    print("")
    print("You may need to install it using:")
    print("    pip3 install --user packaging")
    print("")
    sys.exit(1)


__author__ = "Sergey Belash, Thomas Gubler, Beat Kueng"
__copyright__ = "Copyright (C) 2013-2022 PX4 Development Team."
__license__ = "BSD"
__email__ = "thomasgubler@gmail.com"


TEMPLATE_FILE = ['msg.h.em', 'msg.cpp.em']
OUTPUT_FILE_EXT = ['.h', '.cpp']
INCL_DEFAULT = ['std_msgs:./msg/std_msgs']
PACKAGE = 'px4'
IDL_TEMPLATE_FILE = 'msg.idl.em'

class MsgScope:
    NONE = 0
    SEND = 1
    RECEIVE = 2

def generate_output_from_file(format_idx, filename, outputdir, package, templatedir, includepath):
    """
    Converts a single .msg file to an uorb header/source file
    """
    msg_context = genmsg.msg_loader.MsgContext.create_default()
    full_type_name = genmsg.gentools.compute_full_type_name(package, os.path.basename(filename))

    file_base_name = os.path.basename(filename).replace(".msg", "")

    full_type_name_snake = re.sub(r'(?<!^)(?=[A-Z])', '_', file_base_name).lower()

    spec = genmsg.msg_loader.load_msg_from_file(msg_context, filename, full_type_name)

    field_name_and_type = {}
    for field in spec.parsed_fields():
        field_name_and_type.update({field.name: field.type})

    if includepath:
        search_path = genmsg.command_line.includepath_to_dict(includepath)
    else:
        search_path = {}

    genmsg.msg_loader.load_depends(msg_context, spec, search_path)
    md5sum = genmsg.gentools.compute_md5(msg_context, spec)

    em_globals = {
        "name_snake_case": full_type_name_snake,
        "file_name_in": filename,
        "md5sum": md5sum,
        "search_path": search_path,
        "msg_context": msg_context,
        "spec": spec
    }

    # Make sure output directory exists:
    if not os.path.isdir(outputdir):
        os.makedirs(outputdir)

    template_file = os.path.join(templatedir, TEMPLATE_FILE[format_idx])
    output_file = os.path.join(outputdir, full_type_name_snake + OUTPUT_FILE_EXT[format_idx])

    return generate_by_template(output_file, template_file, em_globals)


def generate_idl_file(filename_msg, msg_dir, alias, outputdir, templatedir, package, includepath, fastrtps_version, ros2_distro, msgs):
    """
    Generates an .idl from .msg file
    """
    msg = os.path.join(msg_dir, filename_msg + ".msg")

    if (alias != ""):
        em_globals = get_em_globals(msg, alias, package, includepath, msgs, fastrtps_version, ros2_distro, MsgScope.NONE)
        spec_short_name = alias
    else:
        em_globals = get_em_globals(
            msg, "", package, includepath, msgs, fastrtps_version, ros2_distro, MsgScope.NONE)
        spec_short_name = em_globals["spec"].short_name

    # Make sure output directory exists:
    if not os.path.isdir(outputdir):
        os.makedirs(outputdir)

    template_file = os.path.join(templatedir, IDL_TEMPLATE_FILE)
    if version.parse(fastrtps_version) <= version.parse('1.7.2'):
        output_file = os.path.join(outputdir, IDL_TEMPLATE_FILE.replace("msg.idl.em", str(spec_short_name + "_.idl")))
    else:
        output_file = os.path.join(outputdir, IDL_TEMPLATE_FILE.replace("msg.idl.em", str(spec_short_name + ".idl")))

    return generate_by_template(output_file, template_file, em_globals)


def generate_uRTPS_general(send_topics, receive_topics, msg_dir, outputdir, templatedir, package, includepath, msgs, fastrtps_version, ros2_distro, template_name):
    """
    Generates source file by msg content
    """

    em_globals_list = []

    if send_topics:
        em_globals_list.extend([get_em_globals(f.key(), f.value(), package, includepath, msgs, fastrtps_version, ros2_distro, MsgScope.SEND) for f in send_topics])

    if receive_topics:
        em_globals_list.extend([get_em_globals(f[0], f[1], package, includepath, msgs, fastrtps_version, ros2_distro, MsgScope.RECEIVE) for f in receive_topics])


    merged_em_globals = {}
    for name in em_globals_list[0]:
        merged_em_globals[name] = [em_globals[name] for em_globals in em_globals_list]


    # Make sure output directory exists:
    if not os.path.isdir(outputdir):
        os.makedirs(outputdir)

    template_file = os.path.join(templatedir, template_name)
    output_file = os.path.join(outputdir, template_name.replace(".em", ""))

    return generate_by_template(output_file, template_file, merged_em_globals)


def get_em_globals(filename_msg, alias, package, includepath, msgs, fastrtps_version, ros2_distro, scope):
    """
    Generates em globals dictionary
    """
    msg_context = genmsg.msg_loader.MsgContext.create_default()

    full_type_name = genmsg.gentools.compute_full_type_name(package, os.path.basename(filename_msg))

    spec = genmsg.msg_loader.load_msg_from_file(msg_context, filename_msg, full_type_name)

    topics = get_multi_topics(filename_msg)

    if includepath:
        search_path = genmsg.command_line.includepath_to_dict(includepath)
    else:
        search_path = {}

    genmsg.msg_loader.load_depends(msg_context, spec, search_path)

    md5sum = genmsg.gentools.compute_md5(msg_context, spec)

    if len(topics) == 0:
        topics.append(spec.short_name)

    em_globals = {
        "file_name_in": filename_msg,
        "md5sum": md5sum,
        "search_path": search_path,
        "msg_context": msg_context,
        "spec": spec,
        "topics": topics,
        "msgs": msgs,
        "scope": scope,
        "package": package,
        "alias": alias,
        "fastrtps_version": fastrtps_version,
        "ros2_distro": ros2_distro
    }

    return em_globals


def generate_by_template(output_file, template_file, em_globals):
    """
    Invokes empy intepreter to geneate output_file by the
    given template_file and predefined em_globals dict
    """
    # check if folder exists:
    folder_name = os.path.dirname(output_file)
    if not os.path.exists(folder_name):
        os.makedirs(folder_name)

    ofile = open(output_file, 'w')

    # todo, reuse interpreter
    interpreter = em.Interpreter(output=ofile, globals=em_globals, options={em.RAW_OPT: True, em.BUFFERED_OPT: True})

    try:
        interpreter.file(open(template_file))
    except OSError as e:
        ofile.close()
        os.remove(output_file)
        raise

    interpreter.shutdown()
    ofile.close()

    return True

def append_to_include_path(path_to_append, curr_include, package):
    for p in path_to_append:
        curr_include.append('%s:%s' % (package, p))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Convert msg files to RTPS headers/sources')
    parser.add_argument('--headers', help='Generate header files', action='store_true')
    parser.add_argument('--sources', help='Generate source files', action='store_true')
    parser.add_argument('-d', dest='dir', help='directory with msg files')
    parser.add_argument('-f', dest='file', help="files to convert (use only without -d)", nargs="+")
    parser.add_argument('-i', dest="include_paths", help='Additional Include Paths', nargs="*", default=None)
    parser.add_argument('-e', dest='templatedir', help='directory with template files',)
    parser.add_argument('-k', dest='package', default=PACKAGE, help='package name')
    parser.add_argument('-o', dest='outputdir', help='output directory for header files')
    parser.add_argument('-p', dest='prefix', default='', help='string added as prefix to the output file name when converting directories')
    parser.add_argument('-q', dest='quiet', default=False, action='store_true', help='string added as prefix to the output file name when converting directories')

    args = parser.parse_args()

    if args.include_paths:
        append_to_include_path(args.include_paths, INCL_DEFAULT, args.package)

    if args.headers:
        generate_idx = 0
    elif args.sources:
        generate_idx = 1
    else:
        print('Error: either --headers or --sources must be specified')
        exit(-1)

    if args.file is not None:
        for f in args.file:
            generate_output_from_file(generate_idx, f, args.outputdir, args.package, args.templatedir, INCL_DEFAULT)
