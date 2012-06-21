#!/usr/bin/env python

from distutils.core import setup

setup(name='flowsim',
      version='1.0',
      description='Flowsim simulation for Tencon',
      author='Brice Rebsamen',
      author_email='brice.rebsamen@gmail.com',
      package_dir={'': 'src'},
      packages=['tencon_sim'],
      scripts=['src/flow_sim_unit.py']
    )