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


class Classifier():
    """Class to classify RTPS msgs as to send, receive and set their IDs."""

    def __init__(self, yaml_file, msg_folder) -> None:
        self.msg_folder = msg_folder
        self.topic_map = self.parse_yaml_msgs_file(yaml_file)

        # Get messages to send and to receive
        self.topics_to_send: List[Tuple[str, str]] = []
        self.topics_to_receive: List[Tuple[str, str]] = []
        self.topics_list: List[str] = []

        # Create message map
        self.setup_topic_map()

        self.msg_files_send    = [os.path.join(self.msg_folder, msg + ".msg") for msg in list(self.topics_to_send.keys())]
        self.msg_files_receive = [os.path.join(self.msg_folder, msg + ".msg") for msg in list(self.topics_to_receive.keys())]

    def setup_topic_map(self) -> None:
        """Setup dictionary with an ID map for the messages."""
        for topic in self.topic_map['rtps']:
            if 'send' in list(topic.keys()):
                self.topics_to_send.append((topic['topic'], topic['msg']))

            if 'receive' in list(topic.keys()):
                self.topics_to_receive.append((topic['topic'], topic['msg']))

            self.topics_list.append(topic['topic'])

    @staticmethod
    def parse_yaml_msgs_file(yaml_file) -> dict:
        """Parses a yaml file into a dict."""
        try:
            with open(yaml_file, 'r') as file:
                return yaml.safe_load(file)
        except OSError as err:
            if err.errno == errno.ENOENT:
                raise IOError(errno.ENOENT, os.strerror(errno.ENOENT), yaml_file)
            raise


if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument("-s", "--send", dest='send', action="store_true", help="Get topics to be sent")
    parser.add_argument("-r", "--receive", dest='receive', action="store_true", help="Get topics to be received")
    parser.add_argument("-i", "--ignore", dest='ignore', action="store_true", help="Get topics to be ignored")
    parser.add_argument("-p", "--path", dest='path', action="store_true", help="Get msgs and its paths")
    parser.add_argument("-m", "--msg-dir", dest='msgdir', type=str, help="message dir, by default msg/", default="msg")
    parser.add_argument("-y", "--rtps-ids-file", dest='yaml_file', type=str,
                        help="RTPS msg IDs definition file absolute path, by default use relative path to msg, tools/urtps_bridge_topics.yaml",
                        default='tools/urtps_bridge_topics.yaml')

    # Parse arguments
    args = parser.parse_args()

    msg_dir = args.msgdir
    if args.msgdir == 'msg':
        msg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    else:
        msg_dir = os.path.abspath(args.msgdir)


    if os.path.isabs(args.yaml_file):
        yaml_file = os.path.abspath(args.yaml_file)
    else:
        yaml_file = os.path.join(msg_dir, args.yaml_file)


    print(yaml_file)

    with open(yaml_file, 'r') as file:
        topic_map = yaml.safe_load(file)

        # Get messages to send and to receive
        topics_to_send: List[Tuple[str, str]] = []
        topics_to_receive: List[Tuple[str, str]] = []
        topics_list: List[str] = []

        # Setup dictionary with an ID map for the messages
        for topic in topic_map['rtps']:
            print("topic ", topic)

            if 'send' in list(topic.keys()):
                topics_to_send.append((topic['topic'], topic['msg']))

            if 'receive' in list(topic.keys()):
                topics_to_receive.append((topic['topic'], topic['msg']))

            topics_list.append(topic['topic'])


        msg_files_send    = [os.path.join(msg_dir, msg + ".msg") for msg in list(topics_to_send.keys())]
        msg_files_receive = [os.path.join(msg_dir, msg + ".msg") for msg in list(topics_to_receive.keys())]


        if args.send:
            if args.path:
                print(('send files: ' + ', '.join(str(msg_file) for msg_file in msg_files_send) + '\n'))
            else:
                print((', '.join(str(msg) for msg in sorted(topics_to_send))))

        if args.receive:
            if args.path:
                print(('receive files: ' + ', '.join(str(msg_file) for msg_file in msg_files_receive) + '\n'))
            else:
                print((', '.join(str(msg) for msg in sorted(topics_to_receive))))
