#! /usr/bin/env python

# Imports from Jinja2
from jinja2 import Environment, FileSystemLoader

# Import YAML from PyYAML
import yaml
import os
import sys

path = sys.argv[1]
if os.path.exists(path):
	# Load data from YAML file into Python dictionary
	config = yaml.load(open(path))

	# Load Jinja2 template
	env = Environment(loader = FileSystemLoader('./Tools/board_manifest/'), trim_blocks=True, lstrip_blocks=True)
	template = env.get_template('template.jinja2')

	# Render template using data and print the output
	print(template.render(config))
