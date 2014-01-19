#!/usr/bin/python
import sys
import os
import yaml
try: import airspeed
except ImportError:
    import sys, os
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
    import airspeed
from airspeed import CachingFileLoader
loader = CachingFileLoader(sys.argv[1])
template = loader.load_template(sys.argv[2])
template_out_f = open(sys.argv[3], 'w')
template_list_f = open(sys.argv[4], 'r')
template_list = yaml.safe_load(template_list_f);
template_out_f.write(template.merge(template_list, loader=loader))