#!/usr/bin/env python
import os.path
import boto
import argh
import bags
from pymongo import MongoClient

BUCKET = 'golfcar.v3'
MONGO_URL = 'mongodb://bhy:bhy@ds063869.mongolab.com:63869/golfcar'
#MONGO_URL = 'mongodb://bhy:bhy@ds063869.mongolab.com:63869/'

PREFIXS = ['default', 'reactive', 'p100', 'p500', 'p10000']

s3conn = boto.s3.connect_to_region('ap-southeast-1')
mongo = MongoClient(MONGO_URL).golfcar


def get_list(prefix):
    bucket = s3conn.get_bucket(BUCKET)
    keys = bucket.list(prefix + '/')
    return keys

def show(prefix):
    keys = get_list(prefix)
    for k in keys:
        print k.name, k.size

def ensure_path_exists(path):
    d = os.path.dirname(path)
    if not os.path.exists(d):
        os.makedirs(d)

def download(path):
    bucket = s3conn.get_bucket(BUCKET)
    key = bucket.get_key(path)
    fn = os.path.basename(path)
    key.get_contents_to_filename(fn)
    print 'Downloaded', fn

def analyze_key(prefix, key):
    bagfn = '/tmp/' + key.name

    ensure_path_exists(bagfn)
    key.get_contents_to_filename(bagfn)
    #a = bags.Analyzer(bagfn)
    #a()
    r = bags.analyze_file(bagfn)
    r['prefix'] = prefix
    r['key'] = key.name
    #result_table.put_item(r)
    mongo.results.insert(r)
    os.remove(bagfn)

def get_task_keys(prefix, force=False):
    prefix = prefix.strip('/')
    keys = get_list(prefix)
    for k in keys:
        if not force and mongo.results.find({"key": k.name}).count() > 0:
            print 'skipping ', k.name
            # result already exists
            continue
        yield k

def analyze(prefix, force=False):
    for k in get_task_keys(prefix, force):
        print 'analyzing ', k.name
        analyze_key(prefix, k)

def get_jobs(last=0):
    for p in PREFIXS:
        for k in get_task_keys(p, force=True):
            yield (p, k)

def handle_job(j):
    prefix, key = j
    if mongo.results.find({"key": key.name}).count() > 0:
        print 'skipping ', key.name
    else:
        analyze_key(prefix, key)


def summary_prefix(prefix):
    import numpy as np
    from scipy.stats import sem
    rs = list(mongo.results.find({'prefix': prefix}))
    fails = [r for r in rs if r['timelen'] == bags.FAIL_TIMELEN]
    rs = [r for r in rs if r['timelen'] != bags.FAIL_TIMELEN]
    total = len(rs)
    print 'prefix = ', prefix
    print 'total = ', total
    if total == 0:
        return
    print 'fails = ', len(fails) / float(total)

    collision_rate = len([r for r in rs if r['max_collision_speed'] > 1.0]) / float(total)
    times = [r['timelen'] for r in rs]
    avg_time = np.mean(times)
    err_time = sem(times)
    err_rate =  2 * np.sqrt(collision_rate*(1-collision_rate)/total)

    #accs = [r['totalacc'] / r['timelen'] for r in rs]
    #avg_acc = np.mean(accs)
    #err_acc = sem(accs)
    totalaccs = [r['totalacc'] for r in rs]
    avg_totalacc = np.mean(totalaccs)
    err_totalacc = sem(totalaccs)

    print 'collision_rate = ', collision_rate, '+/-', err_rate
    print 'avg_time = ', avg_time, '+/-', err_time
    #print 'avg_acc = ', avg_acc, '+/-', err_acc
    print 'avg_totalacc = ', avg_totalacc, '+/-', err_totalacc
    print

def summary():
    for p in PREFIXS:
        summary_prefix(p)

def count():
    for p in PREFIXS:
        print p, ':', len(list(get_list(p)))

def testdb():
    mongo.test.insert({'bar': 1.234})
    mongo.test.insert({'foo': 'bar'})

if __name__=='__main__':
    argh.dispatch_commands([show, analyze, summary, count, download, testdb])

