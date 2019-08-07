#!/usr/bin/python
# -*- coding: utf-8 -*-
#
# Active Intelligent Systems Laboratory
# Toyohashi University of Technology
#
# Yusuke Miake
#
from PIL import Image, ImageDraw
import math


class Point:
    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y

    def equal(self, input_point):
        if self.x == input_point.x and self.y == input_point.y:
            return True
        return False

    def get_distance(self, point):
        return math.sqrt((self.x - point.x) * (self.x - point.x)
                         + (self.y - point.y) * (self.y - point.y))

    def print_data(self):
        print("x: " + str(self.x) + "\ty:" + str(self.y))

    def draw(self, img=Image.new("RGB", (512, 512), (255, 255, 255)), fill=(0, 0, 0)):
        draw = ImageDraw.Draw(img)
        draw.point((self.x, self.y), fill)
        return img


if __name__ == "__main__":
    point = Point(x=100, y=100)
    point2 = Point(x=200, y=200)
    point.draw().show()
    point.print_data()
    print("distance: " + str(point.get_distance(point2)))
