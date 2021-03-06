package frc.team3324.robot.util

import org.opencv.core.*
import org.opencv.imgcodecs.Imgcodecs
import org.opencv.imgproc.Imgproc
import org.opencv.imgproc.Imgproc.COLOR_BGR2HSV
import org.opencv.videoio.VideoCapture
import org.opencv.core.Mat

class RearCamera {
    fun rearCamera(): Boolean {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME)

        val capture = VideoCapture(0)
        capture.set(3, 720.0)
        capture.set(4, 1280.0)

        val img = Mat()
        capture.read(img)
        //val img = Imgcodecs.imread("C:\\Users\\walden\\Documents\\GitHub\\Robotics2021Vision\\Red-A.jpg")

        val hsvImg = Mat()
        Imgproc.cvtColor(img, hsvImg, COLOR_BGR2HSV)

        val lowerYellow = Scalar(29.0, 4.0, 192.0)
        val upperYellow = Scalar(51.0, 62.0, 255.0)
        val mask = Mat()
        Core.inRange(hsvImg, lowerYellow, upperYellow, mask)
        val kernel: Mat = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, Size(7.0, 7.0))
        val opening = Mat()
        Imgproc.morphologyEx(mask, opening, Imgproc.MORPH_OPEN, kernel)
        val contourList: List<MatOfPoint> = ArrayList<MatOfPoint>()
        val hierarchy = Mat()
        Imgproc.findContours(opening, contourList, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE)
        Imgproc.drawContours(img, contourList, -1, Scalar(255.0, 0.0, 0.0), 2, Imgproc.LINE_8, hierarchy, 2, Point())

        val height = capture.get(3)
        val width = capture.get(4)
        val centerY = height / 2
        val centerX = width / 2

        var contourCenter = mutableListOf<Int>()
        var checkContour = mutableListOf<Boolean>()

        for (contour in contourList) {
            var moment = Imgproc.moments(contour)
            contourCenter.add((moment._m10 / moment._m00).toInt());
        }

        for (contour in contourCenter) {
            if (contour > centerX) {
                checkContour.add(true)
            } else {
                checkContour.add(false)
            }
        }

        println(contourList.count())

        return checkContour.count { true } > checkContour.count { false }
    }
}