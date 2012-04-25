import unittest
from StringIO import StringIO
import difflib
from record import *


log='''<xml>
  <sim>
    <base>
      <step t="0">
        <vehicle v="3" x="-30" />
        <pedestrian v="1" x="-7" />
        <pedestrian v="1" x="-16" />
      </step>
      <step t="7.7">
        <vehicle msg="through" v="3" x="0.1" />
        <pedestrian v="1" x="0.7" />
        <pedestrian v="1" x="-8.3" />
      </step>
    </base>
    <infra>
      <step t="0">
        <vehicle v="3" x="-30" />
        <pedestrian v="1" x="-7" />
        <pedestrian v="1" x="-16" />
      </step>
      <step t="6.1">
        <vehicle v="3" x="0.1" />
        <pedestrian v="1" x="-6.9" />
        <pedestrian v="1" x="-15.9" />
      </step>
    </infra>
  </sim>
  <sim>
    <base>
      <step t="0">
        <vehicle v="3" x="-30" />
        <pedestrian v="1" x="-4" />
        <pedestrian v="1" x="-3" />
      </step>
      <step t="8.7">
        <vehicle v="3" x="0.1" />
        <pedestrian v="1" x="0.3" />
        <pedestrian v="1" x="-2.3" />
      </step>
    </base>
    <infra>
      <step t="0">
        <vehicle v="3" x="-30" />
        <pedestrian v="1" x="-7" />
        <pedestrian v="1" x="-16" />
      </step>
      <step t="6.5">
        <vehicle v="3" x="0.1" />
        <pedestrian v="1" x="-6.9" />
        <pedestrian v="1" x="-15.9" />
      </step>
    </infra>
  </sim>
</xml>'''


class Test(unittest.TestCase):
    
    def testLoadSaveCompare(self):
        src = StringIO(log)
        record = load_record(src)
        dst = StringIO()
        record.save_as_xml(dst)
        dst.seek(0)
        out = ''.join(dst.readlines())
        
        if out!=log:
            print log
            print out
            diff = difflib.Differ().compare(log.splitlines(1), out.splitlines(1))
            print ''.join(diff)
            
        self.assertEqual(log, out)


if __name__ == "__main__":
    #import sys;sys.argv = ['', 'Test.testName']
    unittest.main()