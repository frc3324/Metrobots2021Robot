package frc.team3324.robot.util

import org.opencv.core.Core
import org.opencv.core.CvType
import org.opencv.core.Mat
import org.opencv.core.MatOfPoint
import org.opencv.core.Scalar
import org.opencv.imgcodecs.Imgcodecs
import org.opencv.imgproc.Imgproc
import org.opencv.imgproc.Moments

class RearCamera {
    fun RearCamera() {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME)

        val capture = VideoCapture(0)
        capture.set(3, 720)
        capture.set(4, 1280)

        val img = Mat()
        capture.read(img)
        //val imageCodecs = Imgcodecs()
        //val img = imageCodecs.imread("C:\\Users\\walden\\Documents\\GitHub\\Robotics2021Vision\\Red-A.jpg")

        val hsvImg = Mat()
        Imgproc.cvtColor(img,hsvImg,COLOR_BGR2HSV)

        val lowerYellow = Scalar(29, 4, 192)
        val upperYellow = Scalar(51, 62, 255)
        val mask = Mat()
        Core.inRange(hsvImg, lowerYellow, upperYellow, mask)
        val kernel: Mat = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, Size(7, 7))
        Imgproc.morphologyEx(mask, opening, Imgproc.MORPH_OPEN, kernel)
        val contourList: List<MatOfPoint> = ArrayList<MatOfPoint>()
        val hierarchy: Mat = Element.getNewMat()
        Imgproc.findContours(opening, contourList, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE)
        Imgproc.drawContours(img, contourList, -1, Scalar(255,0,0), 2, Imgproc.LINE_8, hierarchy, 2, Point())

        val height = capture.get(3)
        val width = capture.get(4)
        val centerY = height / 2
        val centerX = width / 2

        var contourCenter = mutableListOf<Int>()
        var checkContour = mutableListOf<Int>()

        var direction: Boolean

        for (contour in contourList) {
            var moment = Imgproc.moments(contour)
            contourCenter.add(moments.get_m10() / moments.get_m00();)
        }

        for (contour in contourCenter) {
            if (contour > centerX) {
                checkContour.add(true)
            } else {
                checkContour.add(false)
            }
        }

        if (checkContour.count(true) > checkContour.count(false)) {
            direction = true
        } else {
            direction = false
        }
        return direction
    }
}