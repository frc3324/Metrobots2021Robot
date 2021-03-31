package frc.team3324.robot.util

import edu.wpi.cscore.CvSink
import edu.wpi.first.cameraserver.CameraServer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.opencv.core.*
import org.opencv.imgcodecs.Imgcodecs
import org.opencv.imgproc.Imgproc
import org.opencv.videoio.VideoCapture

class RearCamera {
    var rearCamera = CameraServer.getInstance().startAutomaticCapture(0)
    val cvSink: CvSink = CameraServer.getInstance().getVideo(rearCamera.name)
    // val cvSource = CameraServer.getInstance().putVideo("Power Cells", Consts.Vision.WIDTH, Consts.Vision.HEIGHT)


    var contours = ArrayList<MatOfPoint>()

    var img = Mat()
    var mask = Mat()
    val kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, Size(3.0, 3.0))

    enum class Path {
        RED,
        BLUE
    }

    private var lowerHSV = Scalar(19.125, 127.5, 57.37499)
    private var upperHSV = Scalar(81.875, 255.0, 255.0)

    var erodeIterations = 1
    var dilateIterations = 2

    init {
        rearCamera.setResolution(Consts.Vision.WIDTH, Consts.Vision.HEIGHT)
        rearCamera.setExposureManual(35)
    }

    fun getContours() {
        contours.clear()
        cvSink.grabFrame(img)

        if (img.empty()) {
            println("There is no input frame for rear camera")
        }

        try {
            Imgproc.cvtColor(img, img, Imgproc.COLOR_BGR2HSV)

            // image manipulation
            Core.inRange(img, lowerHSV, upperHSV, mask)

            for (i in 0..erodeIterations) {
                Imgproc.erode(mask, mask, kernel)
            }
            for (i in 0..dilateIterations) {
                Imgproc.dilate(mask, mask, kernel)
            }

            // cvSource.putFrame(mask)

            // contour detection
            Imgproc.findContours(mask, contours, Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE)
        } catch(e: Exception) {
            println("Exception: ${e.message}")
        }
    }

    fun getRedOrBlue(): Path {
        getContours()

        var maxVal = 0.0
        for (contour in contours) {
            val contourArea = Imgproc.contourArea(contour)
            if (maxVal < contourArea) {
                maxVal = contourArea
            }
        }

        SmartDashboard.putNumber("Largest Contour Area", maxVal)

        if (maxVal > 500) {
            // Red Paths had an average max contour area of 938.76
            SmartDashboard.putString("Path", "RED")
            return Path.RED
        }
        // Blue Paths had an average max contour area of 199.5
        SmartDashboard.putString("Path", "BLUE")
        return Path.BLUE
    }
}