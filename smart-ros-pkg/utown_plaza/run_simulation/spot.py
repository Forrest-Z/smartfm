#!/usr/bin/env python
import boto.ec2
import argh
import time

REGION='ap-southeast-1'
#AMI = 'ami-c0ebb192'
#AMI = 'ami-1e2f0b4c'
AMI = 'ami-46183c14'
INSTANCE_TYPE = 'c3.xlarge'
PRICES = {
        'c3.xlarge': 0.048,
        'c3.2xlarge': 0.088,
        'c3.8xlarge': 0.33,
        }


def connect():
    c = boto.ec2.connect_to_region(REGION)
    return c

SCRIPT="""#!/bin/bash
su ubuntu -c "git --git-dir=/home/ubuntu/smartfm/.git --work-tree=/home/ubuntu/smartfm/ pull"
nohup su ubuntu -c "/home/ubuntu/smartfm/run.sh %s" 2>&1 &
"""

SCRIPT_EVAL="""#!/bin/bash
su ubuntu -c "git --git-dir=/home/ubuntu/smartfm/.git --work-tree=/home/ubuntu/smartfm/ pull"
nohup su ubuntu -c "/home/ubuntu/smartfm/tmworker.sh" 2>&1 &
"""

def launch(simtype, number=1, instance=INSTANCE_TYPE):
    if simtype == 'eval':
        user_data = SCRIPT_EVAL
    else:
        user_data = SCRIPT % simtype
    price = PRICES[instance]
    c = connect()
    reqs = c.request_spot_instances(price=price, image_id=AMI, count=number,
            instance_type=instance, user_data=user_data,
            key_name='golfcar', type='persistent')

    print reqs
    for req in reqs:
        print 'Tagging', req.id
        for i in range(6):
            try:
                c.create_tags(req.id, {'Name': simtype})
                break
            except boto.exception.EC2ResponseError:
                time.sleep(2)
                if i>=5:
                    raise

def taginsts():
    c = connect()
    spots = c.get_all_spot_instance_requests()
    acts = [s for s in spots if s.state == 'active']
    for s in acts:
        print 'Tagging ', s.instance_id
        name = s.tags['Name']
        c.create_tags(s.instance_id, {'Name': name})

def spotprice(instance_type):
    c = connect(REGION)
    print c.get_spot_price_history(instance_type=instance_type)

if __name__=='__main__':
    argh.dispatch_commands([launch, spotprice, taginsts])

