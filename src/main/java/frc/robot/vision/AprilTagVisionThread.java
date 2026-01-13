package frc.robot.vision;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Arrays;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

public class AprilTagVisionThread implements Runnable {

    private Mat mat; 

    @Override
    public void run() {
        UsbCamera camera = CameraServer.startAutomaticCapture();
        camera.setResolution(640, 480);
        camera.setConfigJson("{\r\n" + //
                        "    \"fps\": 25,\r\n" + //
                        "    \"height\": 480,\r\n" + //
                        "    \"pixel format\": \"mjpeg\",\r\n" + //
                        "    \"properties\": [\r\n" + //
                        "        {\r\n" + //
                        "            \"name\": \"connect_verbose\",\r\n" + //
                        "            \"value\": 1\r\n" + //
                        "        },\r\n" + //
                        "        {\r\n" + //
                        "            \"name\": \"raw_brightness\",\r\n" + //
                        "            \"value\": 0\r\n" + //
                        "        },\r\n" + //
                        "        {\r\n" + //
                        "            \"name\": \"brightness\",\r\n" + //
                        "            \"value\": 50\r\n" + //
                        "        },\r\n" + //
                        "        {\r\n" + //
                        "            \"name\": \"raw_contrast\",\r\n" + //
                        "            \"value\": 32\r\n" + //
                        "        },\r\n" + //
                        "        {\r\n" + //
                        "            \"name\": \"contrast\",\r\n" + //
                        "            \"value\": 50\r\n" + //
                        "        },\r\n" + //
                        "        {\r\n" + //
                        "            \"name\": \"raw_saturation\",\r\n" + //
                        "            \"value\": 64\r\n" + //
                        "        },\r\n" + //
                        "        {\r\n" + //
                        "            \"name\": \"saturation\",\r\n" + //
                        "            \"value\": 50\r\n" + //
                        "        },\r\n" + //
                        "        {\r\n" + //
                        "            \"name\": \"raw_hue\",\r\n" + //
                        "            \"value\": 0\r\n" + //
                        "        },\r\n" + //
                        "        {\r\n" + //
                        "            \"name\": \"hue\",\r\n" + //
                        "            \"value\": 50\r\n" + //
                        "        },\r\n" + //
                        "        {\r\n" + //
                        "            \"name\": \"white_balance_temperature_auto\",\r\n" + //
                        "            \"value\": true\r\n" + //
                        "        },\r\n" + //
                        "        {\r\n" + //
                        "            \"name\": \"gamma\",\r\n" + //
                        "            \"value\": 100\r\n" + //
                        "        },\r\n" + //
                        "        {\r\n" + //
                        "            \"name\": \"raw_gain\",\r\n" + //
                        "            \"value\": 0\r\n" + //
                        "        },\r\n" + //
                        "        {\r\n" + //
                        "            \"name\": \"gain\",\r\n" + //
                        "            \"value\": 0\r\n" + //
                        "        },\r\n" + //
                        "        {\r\n" + //
                        "            \"name\": \"power_line_frequency\",\r\n" + //
                        "            \"value\": 2\r\n" + //
                        "        },\r\n" + //
                        "        {\r\n" + //
                        "            \"name\": \"white_balance_temperature\",\r\n" + //
                        "            \"value\": 4600\r\n" + //
                        "        },\r\n" + //
                        "        {\r\n" + //
                        "            \"name\": \"raw_sharpness\",\r\n" + //
                        "            \"value\": 3\r\n" + //
                        "        },\r\n" + //
                        "        {\r\n" + //
                        "            \"name\": \"sharpness\",\r\n" + //
                        "            \"value\": 50\r\n" + //
                        "        },\r\n" + //
                        "        {\r\n" + //
                        "            \"name\": \"backlight_compensation\",\r\n" + //
                        "            \"value\": 1\r\n" + //
                        "        },\r\n" + //
                        "        {\r\n" + //
                        "            \"name\": \"exposure_auto\",\r\n" + //
                        "            \"value\": 1\r\n" + //
                        "        },\r\n" + //
                        "        {\r\n" + //
                        "            \"name\": \"raw_exposure_absolute\",\r\n" + //
                        "            \"value\": 10\r\n" + //
                        "        },\r\n" + //
                        "        {\r\n" + //
                        "            \"name\": \"exposure_absolute\",\r\n" + //
                        "            \"value\": 10\r\n" + //
                        "        },\r\n" + //
                        "        {\r\n" + //
                        "            \"name\": \"exposure_auto_priority\",\r\n" + //
                        "            \"value\": false\r\n" + //
                        "        }\r\n" + //
                        "    ],\r\n" + //
                        "    \"width\": 640\r\n" + //
                        "}");

        CvSink cvSink = CameraServer.getVideo(camera);
        CvSource outputStream =
                CameraServer.putVideo("Rectangle", 640, 480);

        AprilTagDetector detector = new AprilTagDetector();
        detector.addFamily("tag36h11");

        AprilTagPoseEstimator.Config poseConfig =
            new AprilTagPoseEstimator.Config(
                Units.inchesToMeters(6.5), // Tag size (official FRC tag)
                546.6314297825006,         // fx
                547.4164926890426,         // fy
                333.04330891767796,        // cx
                243.95434359639907         // cy
            );

        AprilTagPoseEstimator poseEstimator =
            new AprilTagPoseEstimator(poseConfig);

        mat = new Mat();
        Mat gray = new Mat();

        Mat thresh = new Mat();

        while (!Thread.interrupted()) {
            if (cvSink.grabFrame(mat) == 0) {
                outputStream.notifyError(cvSink.getError());
                SmartDashboard.putString("Status", "Camera bad");
                continue;
            }

            SmartDashboard.putString("Status", "Camera good " + Math.random());

            // Convert to grayscale
            // mat = Imgcodecs.imdecode(mat, Imgcodecs.IMREAD_GRAYSCALE);
            Imgproc.cvtColor(mat, gray, Imgproc.COLOR_BGR2GRAY);

            AprilTagDetection[] detections = detector.detect(gray);

            SmartDashboard.putNumber("# of Detections", detections.length);

            for (AprilTagDetection detection : detections) {
                int id = detection.getId();

                // Estimate pose
                Transform3d pose = poseEstimator.estimate(detection);

                // Draw outline
                double[] cornersDoubleArr = detection.getCorners();
                Point[] corners = new Point[4];
                for (int i = 0; i < 4; i++) {
                    corners[i] = new Point(cornersDoubleArr[2 * i], cornersDoubleArr[2 * i + 1]);
                }
                for (int i = 0; i < 4; i++) {
                    Imgproc.line(
                        mat,
                        corners[i],
                        corners[(i + 1) % 4],
                        new Scalar(0, 255, 0),
                        2
                    );
                }

                // Draw ID
                Imgproc.putText(
                    mat,
                    "ID " + id,
                    corners[0],
                    Imgproc.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    new Scalar(0, 255, 0),
                    2
                );

                // Example: publish distance
                SmartDashboard.putNumber(
                    "Tag " + id + " Z (m)",
                    pose.getZ()
                );
            }

            outputStream.putFrame(mat);
        }
        detector.close();
    }

}
