#!/usr/bin/python

"""
Author: Malisa Smith    Date: 11-25-2016

This program can by run via the following command:
python spinningChair.py rotX rotY rotZ
"""

from graphics import *
import math
import os
import sys
import time

class PointXYZ:
    def __init__(self, x, y, z):
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)

    def __repr__(self):
        return "PointXYZ({}, {}, {})".format(self.x, self.y, self.z)

    def _move(self, dx, dy, dz):
        self.x = self.x + dx
        self.y = self.y + dy
        self.z = self.z + dz

    def clone(self):
        other = PointXYZ(self.x,self.y,self.z)
        return other

    # rotate the point around each axis by the given degrees, clockwise
    # e.g. if rotX is 90, the point will rotate along the YZ plane (X staying the same)
    def rotate(self, rotX, rotY, rotZ, pointXYZofRotation):
        # returns the radius and degree for the point
        def toPolar(axis1, axis2):
            axis1neg = axis1 < 0
            axis2neg = axis2 < 0
            radius = math.sqrt(axis1**2 + axis2**2)
            theta = 90
            if axis1 != 0:
                theta = math.degrees(math.atan(abs(axis2) * 1.0 / abs(axis1)))
            if axis1neg and axis2neg:
                theta = 180 + theta # mod 360 ??
            elif axis1neg:
                theta = 180 - theta
            elif axis2neg:
                theta = 360 - theta
            return radius, theta
        # returns the cartesian coordinates along axis 1 and 2 for the point
        def toCartesian(radius, theta):
            axis1 = radius * math.cos(math.radians(theta))
            axis2 = radius * math.sin(math.radians(theta))
            return axis1, axis2
        # given the location on each axis, rotate the point by the
        # given rotation amount, in degrees
        def newCoordinates(axis1, axis2, rot):
            radius, oldTheta = toPolar(axis1, axis2)
            newTheta = oldTheta + rot
            newaxis1, newaxis2 = toCartesian(radius, newTheta)
            return toCartesian(radius, newTheta)
        if rotX != 0:
            self.y, self.z = newCoordinates(self.y - pointXYZofRotation.getY(),
                                            self.z - pointXYZofRotation.getZ(), rotX)
            self.y = self.y + pointXYZofRotation.getY()
            self.z = self.z + pointXYZofRotation.getZ()
        if rotY != 0:
            self.x, self.z = newCoordinates(self.x - pointXYZofRotation.getX(),
                                            self.z - pointXYZofRotation.getZ(), rotY)
            self.x = self.x + pointXYZofRotation.getX()
            self.z = self.z + pointXYZofRotation.getZ()
        if rotZ != 0:
            self.x, self.y = newCoordinates(self.x - pointXYZofRotation.getX(),
                                            self.y - pointXYZofRotation.getY(), rotZ)
            self.x = self.x + pointXYZofRotation.getX()
            self.y = self.y + pointXYZofRotation.getY()

    def getX(self): return self.x
    def getY(self): return self.y
    def getZ(self): return self.z

class PolygonFlexible(Polygon):
    def moveEachPt(self, ptDeltas):
        for i in range(len(ptDeltas)):
            self.points[i].move(ptDeltas[i][0], ptDeltas[i][1])

class RectangleXYZ:
    # uses the four PointXYZ parameters to create the rectangle
    # (though note that only 3 are necessary)
    def __init__(self, pointxyz1, pointxyz2, pointxyz3, pointxyz4):
        self.points = [pointxyz1, pointxyz2, pointxyz3, pointxyz4]
        polygonPtsXY = [Point(pt.getX(), pt.getY()) for pt in self.points]
        self.polygon = PolygonFlexible(polygonPtsXY)
        self.polygon.setFill("gray")

    def draw(self, win):
        self.polygon.draw(win)

    def undraw(self):
        self.polygon.undraw()

    # rotate the rectangle around each axis by the given degrees
    # pointXYZofRotation sets the optional location of the axes of rotation
    def rotate(self, rotX, rotY, rotZ, pointXYZofRotation):
        ptDeltas = []
        for pt in self.points:
            ptOldX = pt.getX()
            ptOldY = pt.getY()
            pt.rotate(rotX, rotY, rotZ, pointXYZofRotation)
            ptDeltas.append([pt.getX() - ptOldX, pt.getY() - ptOldY])
        self.polygon.moveEachPt(ptDeltas)

    def minX(self):
        return min([pt.getX() for pt in self.points])

    def maxX(self):
        return max([pt.getX() for pt in self.points])

    def minY(self):
        return min([pt.getY() for pt in self.points])

    def maxY(self):
        return max([pt.getY() for pt in self.points])

    def minZ(self):
        return min([pt.getZ() for pt in self.points])

    def maxZ(self):
        return max([pt.getZ() for pt in self.points])

    def __repr__(self):
        return "RectangleXYZ({}, {}, {}, {})".format(
            self.points[0].__repr__(),
            self.points[1].__repr__(),
            self.points[2].__repr__(),
            self.points[3].__repr__())

class Cuboid:
    # uses the two PointXYZ parameters to create the cuboid facing straight on
    def __init__(self, pointxyz1, pointxyz2):
        # min might be unnecessary here
        self.x1 = min(pointxyz1.getX(), pointxyz2.getX())
        self.y1 = min(pointxyz1.getY(), pointxyz2.getY())
        self.z1 = min(pointxyz1.getZ(), pointxyz2.getZ())
        self.x2 = max(pointxyz1.getX(), pointxyz2.getX())
        self.y2 = max(pointxyz1.getY(), pointxyz2.getY())
        self.z2 = max(pointxyz1.getZ(), pointxyz2.getZ())
        self.rectanglexyzs = []
        self.defineRectangles()

    def defineRectangles(self):
        self.rectanglexyzs = [
            RectangleXYZ(PointXYZ(self.x1, self.y1, self.z1),
                         PointXYZ(self.x1, self.y1, self.z2),
                         PointXYZ(self.x1, self.y2, self.z2),
                         PointXYZ(self.x1, self.y2, self.z1)),
            RectangleXYZ(PointXYZ(self.x2, self.y1, self.z1),
                         PointXYZ(self.x2, self.y1, self.z2),
                         PointXYZ(self.x2, self.y2, self.z2),
                         PointXYZ(self.x2, self.y2, self.z1)),
            RectangleXYZ(PointXYZ(self.x1, self.y1, self.z1),
                         PointXYZ(self.x1, self.y1, self.z2),
                         PointXYZ(self.x2, self.y1, self.z2),
                         PointXYZ(self.x2, self.y1, self.z1)),
            RectangleXYZ(PointXYZ(self.x1, self.y2, self.z1),
                         PointXYZ(self.x1, self.y2, self.z2),
                         PointXYZ(self.x2, self.y2, self.z2),
                         PointXYZ(self.x2, self.y2, self.z1)),
            RectangleXYZ(PointXYZ(self.x1, self.y1, self.z1),
                         PointXYZ(self.x1, self.y2, self.z1),
                         PointXYZ(self.x2, self.y2, self.z1),
                         PointXYZ(self.x2, self.y1, self.z1)),
            RectangleXYZ(PointXYZ(self.x1, self.y1, self.z2),
                         PointXYZ(self.x1, self.y2, self.z2),
                         PointXYZ(self.x2, self.y2, self.z2),
                         PointXYZ(self.x2, self.y1, self.z2))]

    def draw(self, win):
        # sort rectangles by Z-value so that objects in front are drawn last
        self.rectanglexyzs.sort(key=lambda x: x.maxZ())
        for rectanglexyz in self.rectanglexyzs:
            rectanglexyz.draw(win)

    def undraw(self):
        for rectanglexyz in self.rectanglexyzs:
            rectanglexyz.undraw()

    def shift(self, dx, dy, dz):
        self.x1 += dx
        self.x2 += dx
        self.y1 += dy
        self.y2 += dy
        self.z1 += dz
        self.z2 += dz
        self.defineRectangles()

    # rotate the cuboid around each axis by the given degrees
    # pointXYZofRotation sets the optional location of the axes of rotation
    def rotate(self, rotX, rotY, rotZ, pointXYZofRotation):
        for rectangle in self.rectanglexyzs:
            rectangle.rotate(rotX, rotY, rotZ, pointXYZofRotation)

    def minX(self):
        return min([rect.minX() for rect in self.rectanglexyzs])

    def maxX(self):
        return max([rect.maxX() for rect in self.rectanglexyzs])

    def minY(self):
        return min([rect.minY() for rect in self.rectanglexyzs])

    def maxY(self):
        return max([rect.maxY() for rect in self.rectanglexyzs])

    def minZ(self):
        return min([rect.minZ() for rect in self.rectanglexyzs])

    def maxZ(self):
        return max([rect.maxZ() for rect in self.rectanglexyzs])

    def __repr__(self):
        return "Cuboid({}, {}, {}, {}, {}, {})".format(
            self.rectanglexyzs[0].__repr__(),
            self.rectanglexyzs[1].__repr__(),
            self.rectanglexyzs[2].__repr__(),
            self.rectanglexyzs[3].__repr__(),
            self.rectanglexyzs[4].__repr__(),
            self.rectanglexyzs[5].__repr__())

class CuboidCollection:
    # uses the two PointXYZ parameters to create the cuboid facing straight on
    def __init__(self, cuboids):
        self.cuboids = cuboids

    def draw(self, win):
        # sort cuboids by Z-value so that objects in front are drawn last
        self.cuboids.sort(key=lambda x: x.maxZ())
        for cuboid in self.cuboids:
            cuboid.draw(win)

    def undraw(self):
        for cuboid in self.cuboids:
            cuboid.undraw()

    # rotate the cuboid around each axis by the given degrees
    # pointXYZofRotation sets the optional location of the axes of rotation
    def rotate(self, rotX, rotY, rotZ, pointXYZofRotation):
        for cuboid in self.cuboids:
            cuboid.rotate(rotX, rotY, rotZ, pointXYZofRotation)

    def shift(self, dx, dy, dz):
        for cuboid in self.cuboids:
            cuboid.shift(dx, dy, dz)

    def minX(self):
        return min([cuboid.minX() for cuboid in self.cuboids])

    def maxX(self):
        return max([cuboid.maxX() for cuboid in self.cuboids])

    def minY(self):
        return min([cuboid.minY() for cuboid in self.cuboids])

    def maxY(self):
        return max([cuboid.maxY() for cuboid in self.cuboids])

    def minZ(self):
        return min([cuboid.minZ() for cuboid in self.cuboids])

    def maxZ(self):
        return max([cuboid.maxZ() for cuboid in self.cuboids])

    def getCenter(self):
        return PointXYZ((self.minX() + self.maxX())/2,
                        (self.minY() + self.maxY())/2,
                        (self.minZ() + self.maxZ())/2)

def spin(thing, win, rotX=1, rotY=2, rotZ=0):
    center = thing.getCenter()
    thing.draw(win)
    while win.checkMouse() == None:
        if win.isClosed():
            break
        else:
            thing.undraw()
            thing.draw(win)
            thing.rotate(rotX, rotY, rotZ, center)
            win.flush()
            time.sleep(0.07)

def main(argv=None):
    if argv is None:
        argv = sys.argv

    winX, winY = (900, 800)
    win = GraphWin("Spinning Chair", winX, winY)
    win.autoflush = False

    # define chair cuboid dimensions:
    # chair is based on: http://www.geocities.ws/sketchup_lessons/sketchup-typicalchairdimensions.jpg
    scaler = (min(winX, winY)/3)/15
    shifter = PointXYZ(winX / 2 - 7 * scaler, winY / 2 - 17 * scaler, 0)
    back = Cuboid(PointXYZ(0, 0, 0), PointXYZ(14 * scaler, 16 * scaler, 2 * scaler))
    seat = Cuboid(PointXYZ(0, 16 * scaler, 0), PointXYZ(14 * scaler, 18 * scaler, 16 * scaler))
    frontRightLeg = Cuboid(PointXYZ(0, 18 * scaler, 14 * scaler),
                           PointXYZ(2 * scaler, 34 * scaler, 16 * scaler))
    frontLeftLeg = Cuboid(PointXYZ(12 * scaler, 18 * scaler, 14 * scaler),
                          PointXYZ(14 * scaler, 34 * scaler, 16 * scaler))
    backRightLeg = Cuboid(PointXYZ(0, 18 * scaler, 0),
                          PointXYZ(2 * scaler, 34 * scaler, 2 * scaler))
    backLeftLeg = Cuboid(PointXYZ(12 * scaler, 18 * scaler, 0),
                         PointXYZ(14 * scaler, 34 * scaler, 2 * scaler))

    chair = CuboidCollection([back, seat, frontRightLeg, frontLeftLeg, backRightLeg, backLeftLeg])
    chair.shift(shifter.getX(), shifter.getY(), shifter.getZ())
    if len(argv) >= 4:
        spin(chair, win, int(argv[1]), int(argv[2]), int(argv[3]))
    else:
        spin(chair, win)

if __name__ == '__main__':
    sys.exit(main())
