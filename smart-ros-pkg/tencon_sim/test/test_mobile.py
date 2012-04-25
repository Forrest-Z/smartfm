#!/usr/bin/env python


import unittest
from mobile import Mobile


class TestMobile(unittest.TestCase):
    def test_inf_accelerate(self):
        m = Mobile(0, 1)
        m.throttle(1)
        m.update()
        self.assertEqual(m.v, 1)
        self.assertEqual(m.x, 0.1)

    def test_accelerate(self):
        dt = 0.1
        m = Mobile(0, 1, 1, sim_time_step=dt)
        m.throttle(1)
        while round(m.v-1, 4)!=0:
            m.update()
        self.assertAlmostEqual(m.v, 1, 5)
        self.assertAlmostEqual(m.x, 0.5, 5)

    def test_time_to_stop(self):
        dt = 0.1
        m = Mobile(0, 1, 1, v0=1, sim_time_step=dt)
        ts = m.time_to_stop()
        t = 0
        m.throttle(0)
        while round(m.v,5)!=0:
            t = t + dt
            m.update()
        self.assertAlmostEqual(t, ts, 5)

    def test_dist_to_stop(self):
        dt = 0.1
        m = Mobile(0, 1, 1, v0=1, sim_time_step=dt)
        ds = m.dist_to_stop()
        m.throttle(0)
        while round(m.v,5)!=0:
            m.update()
        self.assertAlmostEqual(m.x, ds, 5)

    def test_time_to_pos(self):
        dt = 0.1
        m = Mobile(0, 1, 1, v0=1, sim_time_step=dt)
        ts = m.time_to_pos(1)
        t = 0
        while round(m.x-1, 2)!=0:
            m.update()
            t = t + dt
        self.assertAlmostEqual(t, ts, 5)

    def test_must_decelerate_to_stop_at_pos(self):
        dt = 0.1
        m = Mobile(0, 1, 1, v0=1, sim_time_step=dt)
        self.assertTrue( m.must_decelerate_to_stop_at_pos(0.1) )
        self.assertTrue( m.must_decelerate_to_stop_at_pos(0.49) )
        self.assertFalse( m.must_decelerate_to_stop_at_pos(0.6) )



if __name__=='__main__':
    unittest.main()
