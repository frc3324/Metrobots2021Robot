package frc.team3324.robot.util

import org.opencv.imgproc.Imgproc
import edu.wpi.cscore.CvSink
import edu.wpi.first.cameraserver.CameraServer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.opencv.core.*
import org.opencv.imgproc.Imgproc.boundingRect
import kotlin.math.abs

class FrontCamera {
    var frontCamera = CameraServer.getInstance().startAutomaticCapture(1)
    val cvSink: CvSink = CameraServer.getInstance().getVideo(frontCamera.name)
    val cvSource = CameraServer.getInstance().putVideo("Mask", Consts.Vision.WIDTH, Consts.Vision.HEIGHT)

    var imgMat = Mat()
    var mask = Mat()
    var contours = ArrayList<MatOfPoint>()

    var targetCenterX = 0.0
    var targetCenterY = 0.0

    var angle = 0.0

    //val kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, Size(7.0, 7.0))
    val kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, Size(3.0, 3.0))


    //val lowerBound = Scalar(49.5,2.5,252.5) // alex's values
    //val upperBound = Scalar(50.5,3.5, 253.5)

    var lowerHSV = Scalar(58.13, 64.75,73.13) // andrew's values
    var upperHSV = Scalar(106.87,255.0, 255.0)

    var erodeIterations = 0
    var dilateIterations = 1

    var targetRect = Rect()

    init {
        frontCamera.setResolution(Consts.Vision.WIDTH, Consts.Vision.HEIGHT)
        frontCamera.setExposureManual(20)
    }

    fun lineUpAngle(): Double {

        val hsv = Mat()

        cvSink.grabFrame(imgMat)

        if (imgMat.empty()) {
            // complain
            System.out.println("There is no input frame for front camera")
        }

        //val capture = VideoCapture(1)
        //capture.read(imgMat)

        /*lowerHSV = Scalar(SmartDashboard.getNumber("LH", 0.0), SmartDashboard.getNumber("LS", 0.0), SmartDashboard.getNumber("LV", 0.0))
        upperHSV = Scalar(SmartDashboard.getNumber("UH", 0.0), SmartDashboard.getNumber("US", 0.0), SmartDashboard.getNumber("UV", 0.0))

        erodeIterations = SmartDashboard.getNumber("erode", 0.0).toInt()
        dilateIterations = SmartDashboard.getNumber("dilate", 0.0).toInt()*/


        try {
            Imgproc.cvtColor(imgMat, hsv, Imgproc.COLOR_BGR2HSV)


            // image manipulation
            Core.inRange(hsv, lowerHSV, upperHSV, mask)


            for (i in 0..erodeIterations) {
                Imgproc.erode(mask, mask, kernel)
            }
            for (i in 0..dilateIterations) {
                Imgproc.dilate(mask, mask, kernel)
            }

            cvSource.putFrame(mask)

            // contour detection and display
            Imgproc.findContours(mask, contours, Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE)
            Imgproc.drawContours(mask, contours, 0, Scalar(255.0, 255.0, 255.0))

            if (!contours.isEmpty()) {
                targetRect = boundingRect(contours[0])

                targetCenterX = targetRect.x + (targetRect.width / 2.0)
                targetCenterY = targetRect.y + (targetRect.height / 2.0)

                // will be positive when turning counterclockwise, just like gyro
                val pixelError = (Consts.Vision.WIDTH / 2) - targetCenterX

                angle = pixelError * Consts.Vision.HORIZONTAL_APP
            } else {
                angle = -2.0
            }

            println("Horizontal Angle to turn: $angle")

        } catch (e: Exception) {
            angle = -1.0
            println("Exception: ${e.message}")
        }

        return angle
    }
}