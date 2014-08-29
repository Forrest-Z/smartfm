from fabric.api import *
from fabtools import require
from fabtools.python import virtualenv
import fabtools
from fabtools.utils import run_as_root
from fabric.contrib.project import rsync_project as rsync
from fabric.contrib import files

env.use_ssh_config = True

@task
def ros():
    require.deb.source('ubuntu-multiverse', 'http://ap-southeast-1.ec2.archive.ubuntu.com/ubuntu/', 'precise', 'multiverse')
    require.deb.source('ubuntu-multiverse-update', 'http://ap-southeast-1.ec2.archive.ubuntu.com/ubuntu/', 'precise-updates', 'multiverse')
    require.deb.source('ros', 'http://packages.ros.org/ros/ubuntu', 'precise', 'main')
    require.deb.key('B01FA116', url='http://packages.ros.org/ros.key')
    require.deb.uptodate_index()
    #require.deb.packages(['ros-fuerte-desktop-full', 'libbullet-dev'])
    require.deb.packages(['ros-fuerte-desktop-full', 'ros-fuerte-octomap'])


SIMDIR = '~/smartfm/smart-ros-pkg/utown_plaza/run_simulation'

@task
def pedbags():
    rsync(remote_dir=SIMDIR + '/pedbags/', local_dir='pedbags/')

@task
def repo():
    require.deb.packages(['colorgcc', 'python-virtualenv'])
    require.git.working_copy('shaojun@bigbird:~/smartfm.git', path='smartfm/', branch='simulation')

    pedbags()

    with cd('~/smartfm'):
        run('./build.sh')

    with cd(SIMDIR):
        require.python.virtualenv('py', system_site_packages=True)
        with virtualenv('py'):
            require.python.requirements('requirements.txt')


#@task
#def setup():

