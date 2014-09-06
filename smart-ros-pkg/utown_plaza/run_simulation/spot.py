#!/usr/bin/env python
import boto.ec2
import argh

REGION='ap-southeast-1'
AMI = 'ami-c0ebb192'
INSTANCE_TYPE = 'c3.xlarge'
PRICE = 0.044


def connect():
    c = boto.ec2.connect_to_region(REGION)
    return c

def launch(number=1, script='default.sh'):
    user_data = open(script).read()
    c = connect()
    r = c.request_spot_instances(price=PRICE, image_id=AMI, count=number,
            instance_type=INSTANCE_TYPE, user_data=user_data,
            key_name='golfcar', type='persistent')
    print r

def spotprice(instance_type):
    c = connect(REGION)
    print c.get_spot_price_history(instance_type=instance_type)

if __name__=='__main__':
    argh.dispatch_commands([launch, spotprice])

