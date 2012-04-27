# -*- coding: utf-8 -*-
'''A class to collect data from the simulation, save it to a xml file, and
load it from an xml file.

Information collected is:
- vehicles position, velocity and messages
- pedestrians position and velocity
'''

import numpy as np
import xml.etree.cElementTree as ET


class Record:
    '''Container for the result of a simulation.
    
    Basically a sequence of results, one for each step of the simulation.
    At each step, it holds the position and velocity of all vehicles and 
    pedestrians, as well as messages associated with each vehicle.
    
    The number of messages and the number of vehicles is equal, but a message
    can be None.
    '''
    
    def __init__(self, t, vehs, peds, msg):
        '''Constructor:
        @param t the time step
        @param vehs the vehicles (array). Each item can either be a tuple giving
        its position and velocity, or an object with fields x and v.
        @param peds the pedestrians (array). Each item can either be a tuple giving
        its position and velocity, or an object with fields x and v.
        @param msg the messages (array of same length as vehs).
        '''
        
        self.t = t
        self.msg = msg
        self.vehs = []
        self.peds = []
        
        for v in vehs:
            if isinstance(v, dict) and v.has_key('x') and v.has_key('v'):
                self.vehs.append(v)
            else:
                self.vehs.append({'x':v.x, 'v':v.v})
                
        for p in peds:
            if isinstance(p, dict) and p.has_key('x') and p.has_key('v'):
                self.peds.append(p)
            else:
                self.peds.append({'x':p.x, 'v':p.v})

        
    def __str__(self):
        sv = 'vehs:[' + ', '.join(['(x=%g, v=%g)'  % (round(v['x'],1), round(v['v'],1)) for v in self.vehs]) + ']'
        sp = 'peds:[' + ', '.join(['%g' % round(p['x'],1) for p in self.peds]) + ']'
        s = ', '.join(['t=%g' % round(self.t,1), sv, sp])
        sm = ['%i: %s' % (i,m) for i,m in enumerate(self.msg) if m is not None]
        if len(sm)>0:
            s = s + '. ' + ', '.join(sm)
        return s
    
    
class Records:
    '''Container for the results of a simulation set.
    
    The same simulation is ran several times so as to explore different values
    for the random variables. For each set of conditions, we run two models:
    the base model (vehicles do not have access to the infrastructure sensor,
    and the infra model (vehicles have access to the infrastructure sensor).
    '''
    
    def __init__(self):
        self.records = []
        
    def append(self, base, infra):
        self.records.append({'base':base, 'infra':infra})
        
    def print_dt_stats(self):
        dts = []
        for r in self.records:
            dts.append(r['base'][-1].t - r['infra'][-1].t)
        print 'dt: avg=%g, std=%g' % (round(np.mean(dts),1), round(np.std(dts),1))
       
    def to_xml(self):
        root_ = ET.Element("xml")
        
        # for each simulation
        for record in self.records:
            sim_ = ET.SubElement(root_, "sim")
            
            # there are 2 parts: base and infra
            for k in ['base', 'infra']:
                rec = record[k]
                record_ = ET.SubElement(sim_, k)
                
                # for each time step
                for r in rec:
                    step_ = ET.SubElement(record_, 'step')
                    step_.set('t', '%g' % round(r.t,3))
                    
                    # log info for each vehicle
                    for i,v in enumerate(r.vehs):
                        veh_ = ET.SubElement(step_, 'vehicle')
                        veh_.set('x', '%g' % round(v['x'],3))
                        veh_.set('v', '%g' % round(v['v'],3))
                        if r.msg[i] is not None:
                            veh_.set('msg', r.msg[i])
                            
                    # log info for each pedestrian
                    for p in r.peds:
                        ped_ = ET.SubElement(step_, 'pedestrian')
                        ped_.set('x', '%g' % round(p['x'],3))
                        ped_.set('v', '%g' % round(p['v'],3))
                        
        return ET.ElementTree(root_)
         
    def save_as_xml(self, file_or_filename):
        '''Save the record as XML into a file.'''
        
        def indent(elem, level=0):
            '''Indent the tree for pretty printing.
            
            Taken from ElementTree's documentation: 
            http://effbot.org/zone/element-lib.htm
            
            FIX: With very large trees, such as the ones we are dealing
            with here, the recursive model fails (MemoryError).
            '''
            
            i = "\n" + level*"  "
            if len(elem):
                if not elem.text or not elem.text.strip():
                    elem.text = i + "  "
                for e in elem:
                    indent(e, level+1)
                    if not e.tail or not e.tail.strip():
                        e.tail = i + "  "
                if not e.tail or not e.tail.strip():
                    e.tail = i
            else:
                if level and (not elem.tail or not elem.tail.strip()):
                    elem.tail = i

        tree = self.to_xml()
        indent(tree.getroot())        
        tree.write(file_or_filename)
    
        
def load_record(file_or_filename):
    
    def parse_sim(el):
        records = []
        for step_ in el.findall('step'):
            t = float(step_.get('t'))
            v = []
            m = []
            p = []
            for v_ in step_.findall('vehicle'):
                v.append( {'x':float(v_.get('x')), 'v':float(v_.get('v'))} )
                m.append( v_.get('msg', None) )
            for p_ in step_.findall('pedestrian'):
                p.append( {'x':float(p_.get('x')), 'v':float(p_.get('v'))} )
            records.append( Record(t, v, p, m) )
        return records
    
    tree = ET.parse(file_or_filename)
    records = Records()
    for record_ in tree.findall('sim'):
        records.append( parse_sim(record_.find('base')), 
                        parse_sim(record_.find('infra')) )
        
    return records