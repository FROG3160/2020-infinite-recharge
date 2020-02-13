from collections import deque
from math import sqrt

XRANGE = 3  # pixels from center to
QUEUELEN = 50


class Line:

    def __init__(self, x1, y1, x2, y2):
        # set p1 as the left point,
        # p2 as the right point.
        if x1 > x2:
            self.p1X = x2
            self.p1Y = y2
            self.p2X = x1
            self.p2Y = y1
        else:
            self.p1X = x1
            self.p1Y = y1
            self.p2X = x2
            self.p2Y = y2
        self.setSide()

    def getYVector(self):
        return self.p2Y - self.p1Y

    def getXVector(self):
        return self.p2X - self.p1X

    def len(self):
        return sqrt(abs((self.p2X - self.p1X))**2 + (abs(self.p2Y - self.p1Y))**2)

    def setSide(self):
        # if p1 is below p2 then this line is on the
        # right side
        if self.p1Y > self.p2Y:
            self.side = 'right'
            self.outer_x = self.p2X
            self.upper_y = self.p2Y
            self.inner_x = self.p1X
            self.inner_y = self.p1Y
        else:
            self.side = 'left'
            self.outer_x = self.p1X
            self.outer_y = self.p1Y
            self.inner_x = self.p2X
            self.inner_y = self.p2X

    def __repr__(self):
        return('<Line - side: {}, ({},{}), ({}, {})>'.format(
            self.side,
            self.p1X,
            self.p1Y,
            self.p2X,
            self.p2Y
        ))

    def __lt__(self, other):
        if self.p1X < other.p1X:
            return self.p1X < other.p1X


class Portal:

    def __init__(self):
        self.left_lines = deque(maxlen=QUEUELEN)
        self.right_lines = deque(maxlen=QUEUELEN)

    def importLines(self, lines):
        left_lines = []
        right_lines = []
        for line in lines:
            if line.side == 'left':
                left_lines.append(line)
            else:
                right_lines.append(line)
        left_lines = sorted(left_lines)
        right_lines = sorted(right_lines)
        if len(left_lines) > 0:
            self.left_lines.append(left_lines[0])
        else:
            self.left_lines.append(None)
        if len(right_lines) > 0:
            self.right_lines.append(right_lines[-1])
        else:
            self.right_lines.append(None)
        self.calculate()

    def calculate(self):
        self.left_line_num = len([x for x in self.left_lines if x])
        self.right_line_num = len([x for x in self.right_lines if x])
        if self.left_line_num > QUEUELEN * 0.8 and self.right_line_num > QUEUELEN * 0.8:
            self.left_avg_len = sum(x.len() for x in self.left_lines if x) / self.left_line_num
            self.right_avg_len = sum(x.len() for x in self.right_lines if x) / self.right_line_num
            self.left_outer_x = sum(x.outer_x for x in self.left_lines if x) / self.left_line_num
            self.right_outer_x = sum(x.outer_x for x in self.right_lines if x) / self.right_line_num
            self.left_outer_y = sum(x.outer_y for x in self.left_lines if x) / self.left_line_num
            self.right_outer_y = sum(x.outer_y for x in self.right_lines if x) / self.right_line_num
            self.upper_width = self.right_outer_x - self.left_outer_x
        else:
            self.left_avg_len = -1
            self.right_avg_len = -1
            self.left_outer_x = -1
            self.right_outer_x = -1
            self.left_outer_y = -1
            self.right_outer_y = -1
            self.upper_width = -1

    def getCenter(self):
        if all(x > -1 for x in [self.left_outer_x, self.right_outer_x, self.left_outer_y, self.right_outer_y]):
            return ((self.right_outer_x + self.left_outer_x) / 2, (self.right_outer_y + self.left_outer_y) / 2)

    def getTargetX(self):
        remap(-self.left.getYVector() - self.right.getYVector(),
              -XRANGE, XRANGE, self.left.p2X, self.right.p1X)


if __name__ == "__main__":

    lines1 = [
        Line(124.42219, 197.61778, 134.39294, 223.11798),
        Line(174.64157, 203.29532, 161.37394, 223.48975),
        Line(163.31168, 227.47672, 177.03806, 204.96233),
        Line(124.42219, 197.61778, 134.39294, 223.11798),
        Line(174.64157, 203.29532, 161.37394, 223.48975),
        Line(163.31168, 227.47672, 177.03806, 204.96233),
        Line(124.42219, 197.61778, 134.39294, 223.11798),
        Line(174.64157, 203.29532, 161.37394, 223.48975),
        Line(163.31168, 227.47672, 177.03806, 204.96233),
    ]
    lines2 = [
        Line(124.42219, 197.61778, 134.39294, 223.11798),
        Line(174.64157, 203.29532, 161.37394, 223.48975),
        Line(163.31168, 227.47672, 177.03806, 204.96233),
        Line(124.42219, 197.61778, 134.39294, 223.11798),
        Line(174.64157, 203.29532, 161.37394, 223.48975),
        Line(163.31168, 227.47672, 177.03806, 204.96233),
        Line(124.42219, 197.61778, 134.39294, 223.11798),
        Line(174.64157, 203.29532, 161.37394, 223.48975),
        Line(163.31168, 227.47672, 177.03806, 204.96233),
    ]
    lines3 = [
        Line(125.42219, 197.61778, 134.39294, 223.11798),
        Line(174.64157, 202.29532, 161.37394, 223.48975),
        Line(163.31168, 227.47672, 178.03806, 204.96233),
        Line(125.42219, 197.61778, 134.39294, 223.11798),
        Line(174.64157, 204.29532, 161.37394, 223.48975),
        Line(163.31168, 227.47672, 178.03806, 204.96233),
        Line(125.42219, 197.61778, 134.39294, 223.11798),
        Line(174.64157, 204.29532, 161.37394, 223.48975),
        Line(163.31168, 227.47672, 179.03806, 204.96233),
    ]
    lines_set = list(set(lines2))
    p = Portal()
    p.importLines(lines1)
    p.importLines(lines2)
    p.importLines(lines3)
    p.calculate()
    blah = Line(124.42219, 197.61778, 134.39294, 223.11798),
    print(lines2)

    # maybe try ration from width to height or length to determine
    # amount of deviation from center
    # will need average height from line (yVector) and probably width
    # of bottom and/or across top using averages of those points
