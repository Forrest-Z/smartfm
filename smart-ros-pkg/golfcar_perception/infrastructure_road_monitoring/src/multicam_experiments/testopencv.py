#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv
import sys

camid = 0
if len(sys.argv) > 1:
    camid = int(sys.argv[1])

capture = cv.CaptureFromCAM(camid)
wname = "camera %d" % camid

if capture:
    cv.NamedWindow(wname, 1)

    while True:
        img = cv.QueryFrame(capture)
        if img is None:
            print 'Problem capturing'
            break
        cv.ShowImage(wname, img)
        if cv.WaitKey(10) == 27: #ESC
            break

    del(capture)
    cv.DestroyWindow(wname)

raw_input("Press ENTER to exit")
