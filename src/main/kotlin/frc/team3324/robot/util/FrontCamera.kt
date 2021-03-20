package frc.team3324.robot.util

import org.opencv.imgproc.Imgproc
import edu.wpi.cscore.CvSink
import edu.wpi.first.cameraserver.CameraServer
import org.opencv.core.Core
import org.opencv.core.Mat
import org.opencv.core.MatOfPoint
import org.opencv.core.Size
import org.opencv.core.Scalar
import org.opencv.videoio.VideoCapture
import kotlin.math.abs
import kotlin.math.atan

class FrontCamera {
    val frontCamera = CameraServer.getInstance().startAutomaticCapture(1)
    val frontCameraName = frontCamera.name
    val cvSink: CvSink = CameraServer.getInstance().getVideo(frontCameraName)

    var imgMat = Mat()
    var mask = Mat()
    var contours = ArrayList<MatOfPoint>()
    var moment = Mat()
    var xCenter = 0.0
    var yCenter = 0.0
    var imageCenter = 0.0

    var angle = 0.0
    var distance = 0.0

    val verticalFOV = 34.3
    val horizontalFOV = 61.0
    val focalLength = 640 / (2 * kotlin.math.tan((horizontalFOV / 2.0)))

    val kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, Size(7.0, 7.0))

    val lowerBound = Scalar(49.5,2.5,252.5)
    val upperBound = Scalar(50.5,3.5, 253.5)

    init {
        frontCamera.setResolution(640, 480)
    }

    fun lineUpAngle(): Double {
        cvSink.grabFrame(imgMat)
        //val capture = VideoCapture(1)
        //capture.read(imgMat)
        //val cvtMat = Mat()
        Imgproc.cvtColor(imgMat, imgMat, Imgproc.COLOR_BGR2HSV)
        Core.inRange(imgMat, lowerBound, upperBound, mask)
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, kernel)
        Imgproc.findContours(mask, contours, Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE)
        Imgproc.drawContours(mask, contours, 0, Scalar(255.0))
        xCenter = Imgproc.moments(mask)._m10 / Imgproc.moments(mask)._m00
        yCenter = Imgproc.moments(mask)._m01 / Imgproc.moments(mask)._m00
        imageCenter = imgMat.size().width / 2
        distance = abs(imageCenter-xCenter)
        angle = atan(distance/(2*focalLength))

        println("Center of Image: " + imageCenter)
        println("Distance Between: "+ distance)
        println("Angle Between: " + angle)

        return angle

    }

}