import cv2
import numpy
import math
from enum import Enum


class PortGripPipeline:
    """
    An OpenCV pipeline generated by GRIP.
    """

    def __init__(self):
        """initializes all values to presets or None if need to be set
        """

        self.__resize_image_width = 320.0
        self.__resize_image_height = 240.0
        self.__resize_image_interpolation = cv2.INTER_CUBIC

        self.resize_image_output = None

        self.__blur_input = self.resize_image_output
        self.__blur_type = BlurType.Box_Blur
        self.__blur_radius = 0.9433962264150947

        self.blur_output = None

        self.__hsl_threshold_input = self.blur_output
        self.__hsl_threshold_hue = [55.932203389830505, 68.29787234042553]
        self.__hsl_threshold_saturation = [225.70621468926555, 255.0]
        self.__hsl_threshold_luminance = [31.214689265536723, 135.18617021276597]

        self.hsl_threshold_output = None

        self.__cv_erode_src = self.hsl_threshold_output
        self.__cv_erode_kernel = None
        self.__cv_erode_anchor = (-1, -1)
        self.__cv_erode_iterations = 0.5
        self.__cv_erode_bordertype = cv2.BORDER_CONSTANT
        self.__cv_erode_bordervalue = -1

        self.cv_erode_output = None

        self.__cv_dilate_src = self.cv_erode_output
        self.__cv_dilate_kernel = None
        self.__cv_dilate_anchor = (-1, -1)
        self.__cv_dilate_iterations = 7.0
        self.__cv_dilate_bordertype = cv2.BORDER_CONSTANT
        self.__cv_dilate_bordervalue = -1

        self.cv_dilate_output = None

        self.__find_contours_input = self.cv_dilate_output
        self.__find_contours_external_only = False

        self.find_contours_output = None

        self.__filter_contours_contours = self.find_contours_output
        self.__filter_contours_min_area = 400.0
        self.__filter_contours_min_perimeter = 0.0
        self.__filter_contours_min_width = 0.0
        self.__filter_contours_max_width = 1000.0
        self.__filter_contours_min_height = 0.0
        self.__filter_contours_max_height = 1000.0
        self.__filter_contours_solidity = [38.60640301318267, 100.0]
        self.__filter_contours_max_vertices = 1000000.0
        self.__filter_contours_min_vertices = 0.0
        self.__filter_contours_min_ratio = 0.0
        self.__filter_contours_max_ratio = 1000.0

        self.filter_contours_output = None

        self.__find_lines_input = self.cv_dilate_output

        self.find_lines_output = None

        self.__filter_lines_lines = self.find_lines_output
        self.__filter_lines_min_length = 5.0
        self.__filter_lines_angle = [138.9830508474576, 227.13532513181022]

        self.filter_lines_output = None

    def process(self, source0):
        """
        Runs the pipeline and sets all outputs to new values.
        """
        # Step Resize_Image0:
        self.__resize_image_input = source0
        (self.resize_image_output) = self.__resize_image(
            self.__resize_image_input,
            self.__resize_image_width,
            self.__resize_image_height,
            self.__resize_image_interpolation,
        )

        # Step Blur0:
        self.__blur_input = self.resize_image_output
        (self.blur_output) = self.__blur(
            self.__blur_input, self.__blur_type, self.__blur_radius
        )

        # Step HSL_Threshold0:
        self.__hsl_threshold_input = self.blur_output
        (self.hsl_threshold_output) = self.__hsl_threshold(
            self.__hsl_threshold_input,
            self.__hsl_threshold_hue,
            self.__hsl_threshold_saturation,
            self.__hsl_threshold_luminance,
        )

        # Step CV_erode0:
        self.__cv_erode_src = self.hsl_threshold_output
        (self.cv_erode_output) = self.__cv_erode(
            self.__cv_erode_src,
            self.__cv_erode_kernel,
            self.__cv_erode_anchor,
            self.__cv_erode_iterations,
            self.__cv_erode_bordertype,
            self.__cv_erode_bordervalue,
        )

        # Step CV_dilate0:
        self.__cv_dilate_src = self.cv_erode_output
        (self.cv_dilate_output) = self.__cv_dilate(
            self.__cv_dilate_src,
            self.__cv_dilate_kernel,
            self.__cv_dilate_anchor,
            self.__cv_dilate_iterations,
            self.__cv_dilate_bordertype,
            self.__cv_dilate_bordervalue,
        )

        # Step Find_Contours0:
        self.__find_contours_input = self.cv_dilate_output
        (self.find_contours_output) = self.__find_contours(
            self.__find_contours_input, self.__find_contours_external_only
        )

        # Step Filter_Contours0:
        self.__filter_contours_contours = self.find_contours_output
        (self.filter_contours_output) = self.__filter_contours(
            self.__filter_contours_contours,
            self.__filter_contours_min_area,
            self.__filter_contours_min_perimeter,
            self.__filter_contours_min_width,
            self.__filter_contours_max_width,
            self.__filter_contours_min_height,
            self.__filter_contours_max_height,
            self.__filter_contours_solidity,
            self.__filter_contours_max_vertices,
            self.__filter_contours_min_vertices,
            self.__filter_contours_min_ratio,
            self.__filter_contours_max_ratio,
        )

        # Step Find_Lines0:
        self.__find_lines_input = self.cv_dilate_output
        (self.find_lines_output) = self.__find_lines(self.__find_lines_input)

        # Step Filter_Lines0:
        self.__filter_lines_lines = self.find_lines_output
        (self.filter_lines_output) = self.__filter_lines(
            self.__filter_lines_lines,
            self.__filter_lines_min_length,
            self.__filter_lines_angle,
        )

    @staticmethod
    def __resize_image(input, width, height, interpolation):
        """Scales and image to an exact size.
        Args:
            input: A numpy.ndarray.
            Width: The desired width in pixels.
            Height: The desired height in pixels.
            interpolation: Opencv enum for the type fo interpolation.
        Returns:
            A numpy.ndarray of the new size.
        """
        return cv2.resize(input, ((int)(width), (int)(height)), 0, 0, interpolation)

    @staticmethod
    def __blur(src, type, radius):
        """Softens an image using one of several filters.
        Args:
            src: The source mat (numpy.ndarray).
            type: The blurType to perform represented as an int.
            radius: The radius for the blur as a float.
        Returns:
            A numpy.ndarray that has been blurred.
        """
        if type is BlurType.Box_Blur:
            ksize = int(2 * round(radius) + 1)
            return cv2.blur(src, (ksize, ksize))
        elif type is BlurType.Gaussian_Blur:
            ksize = int(6 * round(radius) + 1)
            return cv2.GaussianBlur(src, (ksize, ksize), round(radius))
        elif type is BlurType.Median_Filter:
            ksize = int(2 * round(radius) + 1)
            return cv2.medianBlur(src, ksize)
        else:
            return cv2.bilateralFilter(src, -1, round(radius), round(radius))

    @staticmethod
    def __hsl_threshold(input, hue, sat, lum):
        """Segment an image based on hue, saturation, and luminance ranges.
        Args:
            input: A BGR numpy.ndarray.
            hue: A list of two numbers the are the min and max hue.
            sat: A list of two numbers the are the min and max saturation.
            lum: A list of two numbers the are the min and max luminance.
        Returns:
            A black and white numpy.ndarray.
        """
        out = cv2.cvtColor(input, cv2.COLOR_BGR2HLS)
        return cv2.inRange(out, (hue[0], lum[0], sat[0]), (hue[1], lum[1], sat[1]))

    @staticmethod
    def __cv_erode(src, kernel, anchor, iterations, border_type, border_value):
        """Expands area of lower value in an image.
        Args:
           src: A numpy.ndarray.
           kernel: The kernel for erosion. A numpy.ndarray.
           iterations: the number of times to erode.
           border_type: Opencv enum that represents a border type.
           border_value: value to be used for a constant border.
        Returns:
            A numpy.ndarray after erosion.
        """
        return cv2.erode(
            src,
            kernel,
            anchor,
            iterations=(int)(iterations + 0.5),
            borderType=border_type,
            borderValue=border_value,
        )

    @staticmethod
    def __cv_dilate(src, kernel, anchor, iterations, border_type, border_value):
        """Expands area of higher value in an image.
        Args:
           src: A numpy.ndarray.
           kernel: The kernel for dilation. A numpy.ndarray.
           iterations: the number of times to dilate.
           border_type: Opencv enum that represents a border type.
           border_value: value to be used for a constant border.
        Returns:
            A numpy.ndarray after dilation.
        """
        return cv2.dilate(
            src,
            kernel,
            anchor,
            iterations=(int)(iterations + 0.5),
            borderType=border_type,
            borderValue=border_value,
        )

    @staticmethod
    def __find_contours(input, external_only):
        """Sets the values of pixels in a binary image to their distance to the nearest black pixel.
        Args:
            input: A numpy.ndarray.
            external_only: A boolean. If true only external contours are found.
        Return:
            A list of numpy.ndarray where each one represents a contour.
        """
        if external_only:
            mode = cv2.RETR_EXTERNAL
        else:
            mode = cv2.RETR_LIST
        method = cv2.CHAIN_APPROX_SIMPLE
        im2, contours, hierarchy = cv2.findContours(input, mode=mode, method=method)
        return contours

    @staticmethod
    def __filter_contours(
        input_contours,
        min_area,
        min_perimeter,
        min_width,
        max_width,
        min_height,
        max_height,
        solidity,
        max_vertex_count,
        min_vertex_count,
        min_ratio,
        max_ratio,
    ):
        """Filters out contours that do not meet certain criteria.
        Args:
            input_contours: Contours as a list of numpy.ndarray.
            min_area: The minimum area of a contour that will be kept.
            min_perimeter: The minimum perimeter of a contour that will be kept.
            min_width: Minimum width of a contour.
            max_width: MaxWidth maximum width.
            min_height: Minimum height.
            max_height: Maximimum height.
            solidity: The minimum and maximum solidity of a contour.
            min_vertex_count: Minimum vertex Count of the contours.
            max_vertex_count: Maximum vertex Count.
            min_ratio: Minimum ratio of width to height.
            max_ratio: Maximum ratio of width to height.
        Returns:
            Contours as a list of numpy.ndarray.
        """
        output = []
        for contour in input_contours:
            x, y, w, h = cv2.boundingRect(contour)
            if w < min_width or w > max_width:
                continue
            if h < min_height or h > max_height:
                continue
            area = cv2.contourArea(contour)
            if area < min_area:
                continue
            if cv2.arcLength(contour, True) < min_perimeter:
                continue
            hull = cv2.convexHull(contour)
            solid = 100 * area / cv2.contourArea(hull)
            if solid < solidity[0] or solid > solidity[1]:
                continue
            if len(contour) < min_vertex_count or len(contour) > max_vertex_count:
                continue
            ratio = (float)(w) / h
            if ratio < min_ratio or ratio > max_ratio:
                continue
            output.append(contour)
        return output

    class Line:
        def __init__(self, x1, y1, x2, y2):
            self.x1 = x1
            self.y1 = y1
            self.x2 = x2
            self.y2 = y2

        def length(self):
            return numpy.sqrt(pow(self.x2 - self.x1, 2) + pow(self.y2 - self.y1, 2))

        def angle(self):
            return math.degrees(math.atan2(self.y2 - self.y1, self.x2 - self.x1))

    @staticmethod
    def __find_lines(input):
        """Finds all line segments in an image.
        Args:
            input: A numpy.ndarray.
        Returns:
            A filtered list of Lines.
        """
        detector = cv2.createLineSegmentDetector()
        if len(input.shape) == 2 or input.shape[2] == 1:
            lines = detector.detect(input)
        else:
            tmp = cv2.cvtColor(input, cv2.COLOR_BGR2GRAY)
            lines = detector.detect(tmp)
        output = []
        if len(lines) != 0:
            for i in range(1, len(lines[0])):
                tmp = PortGripPipeline.Line(
                    lines[0][i, 0][0],
                    lines[0][i, 0][1],
                    lines[0][i, 0][2],
                    lines[0][i, 0][3],
                )
                output.append(tmp)
        return output

    @staticmethod
    def __filter_lines(inputs, min_length, angle):
        """Filters out lines that do not meet certain criteria.
        Args:
            inputs: A list of Lines.
            min_Lenght: The minimum lenght that will be kept.
            angle: The minimum and maximum angles in degrees as a list of two numbers.
        Returns:
            A filtered list of Lines.
        """
        outputs = []
        for line in inputs:
            if line.length() > min_length:
                if (line.angle() >= angle[0] and line.angle() <= angle[1]) or (
                    line.angle() + 180.0 >= angle[0]
                    and line.angle() + 180.0 <= angle[1]
                ):
                    outputs.append(line)
        return outputs


BlurType = Enum('BlurType', 'Box_Blur Gaussian_Blur Median_Filter Bilateral_Filter')
