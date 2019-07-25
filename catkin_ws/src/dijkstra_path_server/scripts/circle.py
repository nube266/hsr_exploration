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


class Circle:

    def __init__(self, center=Point(), radius=0):
        self._center = center
        self._radius = radius

    def print_data(self):
        print("center:\tx: " + str(self._center.x) +
              "\ty:" + str(self._center.y))
        print("radius:\t" + str(self._radius))

    def draw(self, img=Image.new("RGB", (512, 512), (255, 255, 255)), fill=(0, 0, 0)):
        draw = ImageDraw.Draw(img)
        draw.ellipse((self._center.x - self._radius, self._center.y - self._radius,
                      self._center.x + self._radius, self._center.y + self._radius), fill)
        return img


if __name__ == "__main__":
    circle = Circle(center=Point(x=100, y=100), radius=10)
    circle.draw().show()
    circle.print_data()
