package frc.team3324.robot.util

import org.opencv.imgproc.Imgproc
import edu.wpi.cscore.CvSink
import edu.wpi.first.cameraserver.CameraServer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.opencv.core.*
import kotlin.math.abs
class FrontCamera {
    var frontCamera = CameraServer.getInstance().startAutomaticCapture(1)
    val cvSink: CvSink = CameraServer.getInstance().getVideo(frontCamera.name)
    val cvSource = CameraServer.getInstance().putVideo("Upper Target", Consts.Vision.WIDTH, Consts.Vision.HEIGHT)

    var contours = ArrayList<MatOfPoint>()

    var imgMat = Mat()
    var mask = Mat()

    var targetCenterX = 0.0
    var targetCenterY = 0.0

    var angle = 0.0

    val kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, Size(3.0, 3.0))


    var lowerHSV = Scalar(58.13, 64.75,73.13)
    var upperHSV = Scalar(106.87,255.0, 255.0)

    var erodeIterations = 0
    var dilateIterations = 1

    var targetRect = Rect()

    var pixelError = 0.0

    init {
        frontCamera.setResolution(Consts.Vision.WIDTH, Consts.Vision.HEIGHT)
        frontCamera.setExposureManual(20)
    }

    fun putShuffleboardHSV() {
        SmartDashboard.putNumber("LH", 0.0)
        SmartDashboard.putNumber("LS", 0.0)
        SmartDashboard.putNumber("LV", 0.0)
        SmartDashboard.putNumber("UH", 0.0)
        SmartDashboard.putNumber("US", 0.0)
        SmartDashboard.putNumber("UV", 0.0)

        SmartDashboard.putNumber("Erode", 0.0)
        SmartDashboard.putNumber("Dilate", 0.0)
    }

    fun getShuffleboardHSV():Array<Any> {
        lowerHSV = Scalar(SmartDashboard.getNumber("LH", 0.0), SmartDashboard.getNumber("LS", 0.0), SmartDashboard.getNumber("LV", 0.0))
        upperHSV = Scalar(SmartDashboard.getNumber("UH", 0.0), SmartDashboard.getNumber("US", 0.0), SmartDashboard.getNumber("UV", 0.0))

        erodeIterations = SmartDashboard.getNumber("erode", 0.0).toInt()
        dilateIterations = SmartDashboard.getNumber("dilate", 0.0).toInt()

        return arrayOf(lowerHSV, upperHSV, erodeIterations, dilateIterations)
    }

    fun getContours() {
        contours.clear()
        cvSink.grabFrame(imgMat)
        Core.flip(imgMat, imgMat, -1)

        if (imgMat.empty()) {
            println("There is no input frame for front camera")
        }

        try {
            Imgproc.cvtColor(imgMat, imgMat, Imgproc.COLOR_BGR2HSV)

            // image manipulation
            Core.inRange(imgMat, lowerHSV, upperHSV, mask)

            for (i in 0..erodeIterations) {
                Imgproc.erode(mask, mask, kernel)
            }
            for (i in 0..dilateIterations) {
                Imgproc.dilate(mask, mask, kernel)
            }

            cvSource.putFrame(mask)

            // contour detection
            Imgproc.findContours(mask, contours, Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE)
        } catch(e: Exception) {
            println("Exception: ${e.message}")
        }
    }

    fun lineUpAngle(): Double {
        try {
            getContours()

            targetRect = Imgproc.boundingRect(contours[0])

            targetCenterX = targetRect.x + (targetRect.width / 2.0)
            targetCenterY = targetRect.y + (targetRect.height / 2.0)

            SmartDashboard.putNumber("Target X",  targetCenterX)
            SmartDashboard.putNumber("Target Y", targetCenterY)

            // angle calculation
            pixelError = targetCenterX - (Consts.Vision.WIDTH / 2)
            angle = pixelError * Consts.Vision.HORIZONTAL_APP

        } catch (e: Exception) {
            angle = 0.0
            println("Exception: ${e.message}")
        }

        SmartDashboard.putNumber("Angle to Turn", angle)
        return angle
    }

    fun contourArea(): Double {
        getContours()

        var area = 0.0
        try {
            area = Imgproc.contourArea(contours[0])

            SmartDashboard.putNumber("Contour Area", area)
        } catch (e: Exception) {
            println("Exception: ${e.message}")
        }

        return area
    }
}