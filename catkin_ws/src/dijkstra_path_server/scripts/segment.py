#!/usr/bin/python
# -*- coding: utf-8 -*-
#
# Active Intelligent Systems Laboratory
# Toyohashi University of Technology
#
# Yusuke Miake
#
from point import Point
from PIL import Image, ImageDraw
import numpy as np
import math


class Segment:
    def __init__(self, start_point=Point(), end_point=Point()):
        self._start_point = start_point
        self._end_point = end_point
        self._middle_point = Point(
            (start_point.x + end_point.x) / 2, (start_point.y + end_point.y) / 2)
        self._length = math.sqrt((start_point.x - end_point.x)**2 +
                                 (start_point.y - end_point.y)**2)

    def print_data(self):
        print("start_point\tx: " + str(self._start_point.x) +
              "\ty: " + str(self._start_point.y))
        print("end_point\tx: " + str(self._end_point.x) +
              "\ty: " + str(self._end_point.y))
        print("middle_point\tx: " + str(self._middle_point.x) +
              "\ty: " + str(self._middle_point.y))
        print("length:\t" + str(self._length))

    def draw(self, img=Image.new("RGB", (512, 512), (255, 255, 255)), fill=(0, 0, 0)):
        draw = ImageDraw.Draw(img)
        draw.line((self._start_point.x, self._start_point.y,
                   self._end_point.x, self._end_point.y), fill, width=8)
        return img

    def get_distance_to_point(self, input_point):
        x0 = input_point.x
        y0 = input_point.y
        x1 = self._start_point.x
        y1 = self._start_point.y
        x2 = self._end_point.x
        y2 = self._end_point.y
        a = x2 - x1
        b = y2 - y1
        a2 = a ** 2
        b2 = b ** 2
        r2 = a2 + b2
        tt = -(a * (x1 - x0) + b * (y1 - y0))
        if tt < 0:
            return math.sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0))
        elif tt > r2:
            return math.sqrt((x2 - x0) * (x2 - x0) + (y2 - y0) * (y2 - y0))
        else:
            f1 = a * (y1 - y0) - b * (x1 - x0)
            if r2 == 0:
                return math.sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0))
            return math.sqrt((f1 * f1) / r2)

    def get_middle_point(self):
        return self._middle_point

    def get_length(self):
        return self._length


if __name__ == "__main__":
    seg = Segment(start_point=Point(x=0, y=0), end_point=Point(x=100, y=100))
    seg.draw().show()
    seg.print_data()
    print(str(seg.get_distance_to_point(Point(x=0, y=100))))
