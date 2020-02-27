# Main file for runnig grip pipelines using Cscore
# Created by Aidan Jarrett for FROG TEAM 3160
# FRC 2020 Infinite Recharge

import cv2
import numpy as np
import logging
from cscore import CameraServer, UsbCamera
from networktables import NetworkTables
from port_tracker_lines import PortGripPipeline
from cell_tracker import CellGripPipeline
from portal import Portal, Line

ROBORIO_IP = '10.31.60.2'
TARGETING_TABLE = "target"

HATCH_TARGET = 0
CELL_TARGET = 1

H_RES = 320
V_RES = 240

H_CENTER = H_RES / 2
V_CENTER = V_RES / 2

portal = Portal()


class Target:
    def __init__(self, x, y, w, h):
        # X and Y are coordinates of the top-left corner of the bounding box
        self.x = x
        self.y = y
        self.w = w
        self.h = h
        print('TARGET: {} {} {} {}'.format(x, y, w, h))

    def getXCenter(self):
        return self.x + self.w / 2

    def getYCenter(self):
        return self.y + self.h / 2

    def getArea(self):
        return self.w * self.h

    def __lt__(self, other):
        return abs(self.x - H_CENTER) < abs(other.x - H_CENTER)


def getPortal(pipeline):
    lines = pipeline.filter_lines_output
    output_lines = []

    if lines:  # and (pipeline = cell_pipeline):
        for line in lines:
            output_lines.append(Line(line.x1, line.y1, line.x2, line.y2))
    portal.importLines(output_lines)
    portal_center = portal.getCenter()
    if portal_center:
        print('Portal Center: {},{}'.format(portal_center[0], portal_center[1]))
        print(
            'Inner Y, left: {}, right: {}'.format(
                portal.left_inner_y, portal.right_inner_y
            )
        )
        print('Y DIFF: {}'.format(portal.left_inner_y - portal.right_inner_y))

    # print('Found Portal--->')
    # for line in lines:
    # if abs(line.y1 - line.y2) > 10:
    # print("Line: ",
    # ((line.x1, line.y1),
    # (line.x2, line.y2),
    # 'Height: {}'.format(line.y1-line.y2),
    # 'Width: {}'.format(line.x1-line.x2),
    # line.length(), line.angle()
    # )
    # )
    # output_lines.append([line.x1, line.x2, line.y1, line.y2, line.length(), line.angle()])

    return portal_center


def getPortalBottom(pipeline):

    lines = pipeline.filter_lines_output
    found_lines = []
    if lines:
        for line in lines:
            if line.x1 > line.x2:
                leftX = line.x2
                leftY = line.y2
                rightX = line.x1
                rightY = line.y1
            else:
                leftX = line.x1
                leftY = line.y1
                rightX = line.x2
                rightY = line.y2
            xVector = rightX - leftX
            found_lines.append((xVector, line.angle(), line.length(), rightY - leftY))
    print(found_lines)
    return sorted(found_lines)[-1]


def getTargets(pipeline):
    """
    Performs extra processing on the pipeline's outputs and publishes data to NetworkTables.
    :param pipeline: the pipeline that just processed an image
    :return: None
    """
    # print('Using pipeline: {}'.format(pipeline))
    targets = []
    contours = pipeline.filter_contours_output

    if contours:
        for contour in pipeline.filter_contours_output:
            # print("Contour: ", cv2.boundingRect(contour))
            # appends a tuple of (x, y, w, h)
            target = Target(*cv2.boundingRect(contour))
            targets.append(target)

    return targets


def getTarget(pipeline, frame):
    # get an array of tuples with (x, y, w, h)
    pipeline.process(frame)
    targets = getTargets(pipeline)

    if len(targets) > 0:
        # sort the targets by the distance from center
        # targets = sorted([(abs(t.getXCenter()-H_CENTER), t) for t in targets])
        sorted(targets)
        # now get the center coordinats from the two closest
        t1 = targets[0]

        return (t1.getXCenter(), t1.getYCenter())
    else:
        return (None, None)


def main():
    print('Initializing NetworkTables')

    logging.basicConfig(level=logging.DEBUG)

    NetworkTables.initialize(server=ROBORIO_IP)

    target_table = NetworkTables.getTable(TARGETING_TABLE)

    print('Creating video capture')
    cs = CameraServer.getInstance()
    cs.enableLogging()

    usb2 = cs.startAutomaticCapture(dev=1, name="Port_Cam")
    usb1 = cs.startAutomaticCapture(dev=0, name="Cell_Cam")

    cvSink_P = cs.getVideo(name="Port_Cam")
    cvSink_C = cs.getVideo(name="Cell_Cam")

    outputStream_P = cs.putVideo("Port", H_RES, V_RES)
    outputStream_C = cs.putVideo("Power_Cell", H_RES, V_RES)
    img_P = np.zeros(shape=(H_RES, V_RES, 3), dtype=np.uint8)
    img_C = np.zeros(shape=(H_RES, V_RES, 3), dtype=np.uint8)

    print('Creating pipelines')

    port_pipeline = PortGripPipeline()
    cell_pipeline = CellGripPipeline()

    # Changes camera settings for the object we're tracking
    usb1.setExposureManual(36)  # range is 0-100
    usb1.setBrightness(0)  # range is 0-100
    usb1.setWhiteBalanceManual(2800)  # range is 1700-15,000
    usb1.getProperty("saturation").set(99)
    usb1.getProperty("contrast").set(100)
    usb1.getProperty("sharpness").set(0)

    # Changes camera settings for the object we're tracking
    usb2.setExposureManual(13)  # range is 0-100
    usb2.setBrightness(17)  # range is 0-100
    usb2.setWhiteBalanceManual(7674)  # range is 1700-15,000
    usb2.getProperty("saturation").set(100)
    usb2.getProperty("contrast").set(100)
    usb2.getProperty("sharpness").set(0)

    print('Waiting for networktables variable...')
    while True:
        # Was an unused variable
        # target_object = target_table.getNumber('object', 1)

        have_frame_P, frame_P = cvSink_P.grabFrame(img_P)
        have_frame_C, frame_C = cvSink_C.grabFrame(img_C)

        # if have_frame
        if have_frame_P:
            # print('Targeting Port')
            xP, yP = getTarget(port_pipeline, frame_P)
            # portal_center = getPortal(port_pipeline)
            linedata = getPortalBottom(port_pipeline)

        else:

            xP = None
            yP = None
            # portal_center = None
            linedata = None

        if have_frame_C:
            # print('Targeting Cell')
            xC, yC = getTarget(cell_pipeline, frame_C)
        else:
            xC = None
            yC = None

        # Tell the CvSink to grab a frame from the camera and put it
        # in the source image.  If there is an error notify the output.
        time_P, img_P = cvSink_P.grabFrame(img_P)
        time_C, img_C = cvSink_C.grabFrame(img_C)

        if time_P == 0:
            # Send the output the error.
            outputStream_P.notifyError(cvSink_P.getError())
            # skip the rest of the current iteration
            continue

        if time_C == 0:
            # Send the output the error.
            outputStream_C.notifyError(cvSink_C.getError())
            # skip the rest of the current iteration
            continue

        if linedata:
            print(
                'xVector: {}, Angle: {}, Length: {}, yDiff: {}'.format(
                    linedata[0], linedata[1], linedata[2], linedata[3]
                )
            )
        # if portal_center:
        # font = cv2.FONT_HERSHEY_SIMPLEX
        # cv2.putText(img_P, 'V', (int(portal_center[0] / 2), (int(portal_center[1] / 2))), font, .4, (255, 255, 255), 1, cv2.LINE_AA)
        if xP and yP:
            # print('Coordinates: {}'.format((xP, yP)))
            target_table.putNumber('Port_x', xP)
            target_table.putNumber('Port_y', yP)

            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(
                img_P,
                '+',
                (int(xP / 2 - 5), (int(yP / 2))),
                font,
                0.4,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )
        else:
            target_table.putNumber('Port_x', -999)
            target_table.putNumber('Port_y', -999)

        if xC and yC:
            # print('Coordinates: {}'.format((xC, yC)))
            target_table.putNumber('Cell_x', xC)
            target_table.putNumber('Cell_y', yC)
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(
                img_C,
                '+',
                (int(xC / 2 - 5), (int(yC / 2))),
                font,
                0.4,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )
        else:

            target_table.putNumber('Cell_x', -999)
            target_table.putNumber('Cell_y', -999)

        # if lines_targeted:
        #    target_table.putNumberArray('Lines', lines_targeted)

        # Give the output stream a new image to display
        outputStream_P.putFrame(img_P)
        outputStream_C.putFrame(img_C)

    print('Capture closed')


if __name__ == '__main__':
    main()
