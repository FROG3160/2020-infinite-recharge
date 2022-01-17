# Main file for runnig grip pipelines using Cscore
# Created by Aidan Jarrett for FROG TEAM 3160
# FRC 2020 Infinite Recharge

import cv2
import numpy as np
import logging
from cscore import CameraServer, UsbCamera
from networktables import NetworkTables
from powerport_tracker import PortGripPipeline
from powercell_tracker import CellGripPipeline

# from portal import Portal, Line

ROBORIO_IP = '10.31.60.2'
TARGETING_TABLE = "targets"

POWERPORT_TABLE = TARGETING_TABLE + '/PowerPort'
POWERCELL_TABLE = TARGETING_TABLE + '/PowerCell'
CONTROL_TABLE = TARGETING_TABLE + '/control'

INVALID_VALUE = -999

H_RES = 320
V_RES = 240

H_CENTER = H_RES / 2
V_CENTER = V_RES / 2

# portal = Portal()


class ContourTarget:
    def __init__(self, x, y, w, h):
        # X and Y are coordinates of the top-left corner of the bounding box
        self.x = x
        self.y = y
        self.w = w
        self.h = h
        self.area = self.getArea()
        self.center_x = self.getCenterX()
        self.center_y = self.getCenterY()

    def getCenterX(self):
        return self.x + (self.w / 2)

    def getCenterY(self):
        return self.y + (self.h / 2)

    def getArea(self):
        return self.w * self.h

    def getDistanceFromVCenter(self):
        return self.center_y - V_CENTER

    def getDistanceFromHCenter(self):
        return self.center_x - H_CENTER

    def __repr__(self):
        return 'ContourTarget - x:{}, y:{}, size:{}'.format(
            self.center_x, self.center_y, self.area
        )


def processTargetsFromContours(pipeline):
    """
    Creates target objects from contours found by pipeline.
    :return: list of targets
    """
    # print('Using pipeline: {}'.format(pipeline))
    targets = []
    contours = pipeline.filter_contours_output

    if contours:
        for contour in pipeline.filter_contours_output:
            # print("Contour: ", cv2.boundingRect(contour))
            # appends a tuple of (x, y, w, h)
            target = ContourTarget(*cv2.boundingRect(contour))
            targets.append(target)

    return targets


def getTargetNearCenter(pipeline, frame):
    # get an array of tuples with (x, y, w, h)
    pipeline.process(frame)
    targets = processTargetsFromContours(pipeline)

    if len(targets) > 0:
        # sort the targets by the distance from center
        targets = sorted(targets, key=lambda t: abs(t.getDistanceFromHCenter()))
        # return the coordinates of the closest one
        t1 = targets[0]

        return (t1.getCenterX(), t1.getCenterY(), t1.area)
    else:
        return (None, None, None)


def getTargetClosest(pipeline, frame):
    # get an array of tuples with (x, y, w, h)
    pipeline.process(frame)
    targets = processTargetsFromContours(pipeline)

    if len(targets) > 0:
        # sort the targets by the distance from center
        targets = sorted(targets, key=lambda t: t.center_y, reverse=True)
        # return the coordinates of the closest one
        t1 = targets[0]

        return (t1.getCenterX(), t1.getCenterY(), t1.area)
    else:
        return (None, None, None)


def getTargetLargest(pipeline, frame):
    pipeline.process(frame)
    targets = processTargetsFromContours(pipeline)

    if len(targets) > 0:
        # sort the targets by the distance from center
        targets = sorted(targets, key=lambda t: t.area, reverse=True)
        # return the coordinates of the closest one
        t1 = targets[0]

        return (t1.getCenterX(), t1.getCenterY(), t1.area)
    else:
        return (None, None, None)


def main():
    print('Initializing NetworkTables')

    logging.basicConfig(level=logging.DEBUG)

    NetworkTables.initialize(server=ROBORIO_IP)

    powerport_table = NetworkTables.getTable(POWERPORT_TABLE)
    powercell_table = NetworkTables.getTable(POWERCELL_TABLE)
    targeting_control = NetworkTables.getTable(CONTROL_TABLE)

    print('Creating video capture')
    cs = CameraServer.getInstance()
    cs.enableLogging()

    usb1 = cs.startAutomaticCapture(dev=0, name="PowerPort_Cam")
    usb2 = cs.startAutomaticCapture(dev=1, name="PowerCell_Cam")

    cvSink_PowerPort = cs.getVideo(name="PowerPort_Cam")
    cvSink_PowerCell = cs.getVideo(name="PowerCell_Cam")

    outputStream_PowerPort = cs.putVideo("PowerPort", H_RES, V_RES)
    outputStream_PowerCell = cs.putVideo("PowerCell", H_RES, V_RES)
    img_PowerPort = np.zeros(shape=(H_RES, V_RES, 3), dtype=np.uint8)
    img_PowerCell = np.zeros(shape=(H_RES, V_RES, 3), dtype=np.uint8)

    print('Creating pipelines')

    powerport_pipeline = PortGripPipeline()
    powercell_pipeline = CellGripPipeline()

    print('Setting Camera Exposure')
    # Changes camera settings for the object we're tracking
    usb2.setExposureManual(36)  # range is 0-100
    usb2.setBrightness(0)  # range is 0-100
    usb2.setWhiteBalanceManual(2800)  # range is 1700-15,000
    usb2.getProperty("saturation").set(99)
    usb2.getProperty("contrast").set(100)
    usb2.getProperty("sharpness").set(0)

    print('Doing it agian')
    # Changes camera settings for the object we're tracking
    usb1.setExposureManual(23)  # range is 0-100
    usb1.setBrightness(41)  # range is 0-100
    usb1.setWhiteBalanceManual(10000)  # range is 1700-15,000
    usb1.getProperty("saturation").set(100)
    usb1.getProperty("contrast").set(100)
    usb1.getProperty("sharpness").set(0)

    print('Waiting for networktables variable...')
    while True:
        # Was an unused variable
        # target_object = target_table.getNumber('object', 1)

        have_frame_PowerPort, frame_PowerPort = cvSink_PowerPort.grabFrame(
            img_PowerPort
        )
        have_frame_PowerCell, frame_PowerCell = cvSink_PowerCell.grabFrame(
            img_PowerCell
        )

        # if have_frame
        if have_frame_PowerPort:
            print('Found Targeting Port')
            PowerPort_x, PowerPort_y, PowerPort_area = getTargetNearCenter(
                powerport_pipeline, frame_PowerPort
            )
        else:
            PowerPort_x = None
            PowerPort_y = None

        if have_frame_PowerCell:
            # print('Targeting Cell')
            PowerCell_x, PowerCell_y, PowerCell_area = getTargetClosest(
                powercell_pipeline, frame_PowerCell
            )
        else:
            PowerCell_x = None
            PowerCell_y = None

        # Tell the CvSink to grab a frame from the camera and put it
        # in the source image.  If there is an error notify the output.
        time_PowerPort, img_PowerPort = cvSink_PowerPort.grabFrame(img_PowerPort)
        time_PowerCell, img_PowerCell = cvSink_PowerCell.grabFrame(img_PowerCell)

        if time_PowerPort == 0:
            # Send the output the error.
            outputStream_PowerPort.notifyError(cvSink_PowerPort.getError())
            # skip the rest of the current iteration
            continue

        if time_PowerCell == 0:
            # Send the output the error.
            outputStream_PowerCell.notifyError(cvSink_PowerCell.getError())
            # skip the rest of the current iteration
            continue

        if PowerPort_x and PowerPort_y:
            # print('Coordinates: {}'.format((xP, yP)))
            powerport_table.putNumber('center_x', PowerPort_x)
            powerport_table.putNumber('center_y', PowerPort_y)
            powerport_table.putNumber('area', PowerPort_area)

            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(
                img_PowerPort,
                '+',
                (int(PowerPort_x / 2 - 5), (int(PowerPort_y / 2))),
                font,
                0.4,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )
        else:
            powerport_table.putNumber('center_x', INVALID_VALUE)
            powerport_table.putNumber('center_y', INVALID_VALUE)
            powerport_table.putNumber('area', INVALID_VALUE)

        if PowerCell_x and PowerCell_y:
            # print('Coordinates: {}'.format((xC, yC)))
            powercell_table.putNumber('center_x', PowerCell_x)
            powercell_table.putNumber('center_y', PowerCell_y)
            powercell_table.putNumber('area', PowerCell_area)
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(
                img_PowerCell,
                '+',
                (int(PowerCell_x / 2 - 5), (int(PowerCell_y / 2))),
                font,
                0.4,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )
        else:

            powercell_table.putNumber('center_x', INVALID_VALUE)
            powercell_table.putNumber('center_y', INVALID_VALUE)
            powercell_table.putNumber('area', INVALID_VALUE)

        outputStream_PowerPort.putFrame(img_PowerPort)
        outputStream_PowerCell.putFrame(img_PowerCell)

    print('Capture closed')


if __name__ == '__main__':
    main()
