#!/usr/bin/env python3
################################################################################
#
# Copyright (c) 2018-2021 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors
# may be used to endorse or promote products derived from this software without
# specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
################################################################################

import argparse
import difflib
import errno
import os
from typing import Dict, List, Tuple
import yaml

if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument("--send-topics", dest='send_topics', action="store_true", help="Get topics to be sent")
    parser.add_argument("--send-msgs", dest='send_msgs', action="store_true", help="Get msg files for topics to be sent")
    parser.add_argument("--receive-topics", dest='receive_topics', action="store_true", help="Get topics to be received")
    parser.add_argument("--receive-msgs", dest='receive_msgs', action="store_true", help="Get msg files for topics to be received")
    parser.add_argument("-p", "--path", dest='path', action="store_true", help="Get msgs and its paths")
    parser.add_argument("-m", "--msg-dir", dest='msgdir', type=str, help="message dir, by default msg/", default="msg")
    parser.add_argument("-y", "--rtps-ids-file", dest='yaml_file', type=str,
                        help="RTPS msg IDs definition file absolute path, by default use relative path to PX4, Tools/msg/urtps_bridge_topics.yaml",
                        default='Tools/msg/urtps_bridge_topics.yaml')

    # Parse arguments
    args = parser.parse_args()

    msg_dir = args.msgdir
    if args.msgdir == 'msg':
        msg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    else:
        msg_dir = os.path.abspath(args.msgdir)

    with open(args.yaml_file, 'r') as file:
        topic_map = yaml.safe_load(file)

        # Get messages to send and to receive
        topics_to_send: Dict[str, str] = dict()
        topics_to_receive: Dict[str, str] = dict()

        topics_list: List[str] = []

        #print(topic_map)

        # Setup dictionary with an ID map for the messages
        for topic in topic_map['rtps']:
            #print("topic ", topic, topic.keys())

            if 'send' in list(topic.keys()):
                topics_to_send[topic['topic']] = topic['msg']

            if 'receive' in list(topic.keys()):
                topics_to_receive[topic['topic']] = topic['msg']

            topics_list.append(topic['topic'])


        msg_files_send    = [os.path.join(msg_dir, msg + ".msg") for msg in list(topics_to_send.values())]
        msg_files_receive = [os.path.join(msg_dir, msg + ".msg") for msg in list(topics_to_receive.values())]

        if args.send_topics:
            print((', '.join(str(msg) for msg in sorted(topics_to_send))))

        elif args.send_msgs:
            print(('send files: ' + ', '.join(str(msg_file) for msg_file in msg_files_send) + '\n'))

        elif args.receive_topics:
            print((', '.join(str(msg) for msg in sorted(topics_to_receive))))

        elif args.receive_msgs:
            print(('receive files: ' + ', '.join(str(msg_file) for msg_file in msg_files_receive) + '\n'))


