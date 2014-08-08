import os.path
import boto
import argh
import bags

s3conn = boto.s3.connect_to_region('ap-southeast-1')

BUCKET = 'golfcar.test'

def get_list(prefix):
    bucket = s3conn.get_bucket(BUCKET)
    keys = bucket.list(prefix)
    return keys

def show(prefix):
    keys = get_list(prefix)
    for k in keys:
        print k.name, k.size

def ensure_path_exists(path):
    d = os.path.dirname(path)
    if not os.path.exists(d):
        os.makedirs(d)

def analyze_key(key):
    bagfn = '/tmp/' + key.name
    ensure_path_exists(bagfn)
    key.get_contents_to_filename(bagfn)
    #a = bags.Analyzer(bagfn)
    #a()
    bags.analyze(bagfn)

def analyze(prefix):
    keys = get_list(prefix)
    for k in keys:
        analyze_key(k)

if __name__=='__main__':
    argh.dispatch_commands([show, analyze])

