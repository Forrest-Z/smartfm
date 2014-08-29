import os.path
import boto
import argh
import bags
from pymongo import MongoClient

BUCKET = 'golfcar.test'
MONGO_URL = 'mongodb://bhy:bhy@ds063869.mongolab.com:63869/golfcar'
#MONGO_URL = 'mongodb://bhy:bhy@ds063869.mongolab.com:63869/'

s3conn = boto.s3.connect_to_region('ap-southeast-1')
mongo = MongoClient(MONGO_URL).golfcar


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

def analyze_key(prefix, key):
    bagfn = '/tmp/' + key.name
    ensure_path_exists(bagfn)
    key.get_contents_to_filename(bagfn)
    #a = bags.Analyzer(bagfn)
    #a()
    r = bags.analyze(bagfn)
    r['prefix'] = prefix
    r['key'] = key.name
    #result_table.put_item(r)
    mongo.results.insert(r)

def analyze(prefix):
    prefix = prefix.strip('/')
    keys = get_list(prefix)
    for k in keys:
        analyze_key(prefix, k)

def testdb():
    mongo.test.insert({'bar': 1.234})
    mongo.test.insert({'foo': 'bar'})

if __name__=='__main__':
    argh.dispatch_commands([show, analyze, testdb])

