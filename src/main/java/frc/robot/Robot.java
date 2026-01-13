// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.opencv.imgproc.Imgproc;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.vision.VisionThread;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.drivetrain.Pigeon2Gyro;
import frc.robot.vision.TestPipeline;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  // ROBO CONTAINER INSTANCE THING
  @Logged(name = "RobotContainer")
  private final RobotContainer m_robotContainer;
  
  private static final int IMG_WIDTH = 640;
  private static final int IMG_HEIGHT = 480;
  private VisionThread visionThread;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.  
   */
  public Robot() {
    m_robotContainer = new RobotContainer();

    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    // DataLogManager.start();

    // Shuffleboard.getTab("Pigeon").add(gyro);
    Epilogue.bind(this);
  }

  @Override
  public void robotInit() {
    UsbCamera camera = CameraServer.startAutomaticCapture();
    // camera.setResolution(IMG_WIDTH, IMG_HEIGHT);
    // camera.setResolution(640, 480);
    // camera.setConfigJson("{\r\n" + //
    //                 "    \"fps\": 25,\r\n" + //
    //                 "    \"height\": 480,\r\n" + //
    //                 "    \"pixel format\": \"mjpeg\",\r\n" + //
    //                 "    \"properties\": [\r\n" + //
    //                 "        {\r\n" + //
    //                 "            \"name\": \"connect_verbose\",\r\n" + //
    //                 "            \"value\": 1\r\n" + //
    //                 "        },\r\n" + //
    //                 "        {\r\n" + //
    //                 "            \"name\": \"raw_brightness\",\r\n" + //
    //                 "            \"value\": 0\r\n" + //
    //                 "        },\r\n" + //
    //                 "        {\r\n" + //
    //                 "            \"name\": \"brightness\",\r\n" + //
    //                 "            \"value\": 50\r\n" + //
    //                 "        },\r\n" + //
    //                 "        {\r\n" + //
    //                 "            \"name\": \"raw_contrast\",\r\n" + //
    //                 "            \"value\": 32\r\n" + //
    //                 "        },\r\n" + //
    //                 "        {\r\n" + //
    //                 "            \"name\": \"contrast\",\r\n" + //
    //                 "            \"value\": 50\r\n" + //
    //                 "        },\r\n" + //
    //                 "        {\r\n" + //
    //                 "            \"name\": \"raw_saturation\",\r\n" + //
    //                 "            \"value\": 64\r\n" + //
    //                 "        },\r\n" + //
    //                 "        {\r\n" + //
    //                 "            \"name\": \"saturation\",\r\n" + //
    //                 "            \"value\": 50\r\n" + //
    //                 "        },\r\n" + //
    //                 "        {\r\n" + //
    //                 "            \"name\": \"raw_hue\",\r\n" + //
    //                 "            \"value\": 0\r\n" + //
    //                 "        },\r\n" + //
    //                 "        {\r\n" + //
    //                 "            \"name\": \"hue\",\r\n" + //
    //                 "            \"value\": 50\r\n" + //
    //                 "        },\r\n" + //
    //                 "        {\r\n" + //
    //                 "            \"name\": \"white_balance_temperature_auto\",\r\n" + //
    //                 "            \"value\": true\r\n" + //
    //                 "        },\r\n" + //
    //                 "        {\r\n" + //
    //                 "            \"name\": \"gamma\",\r\n" + //
    //                 "            \"value\": 100\r\n" + //
    //                 "        },\r\n" + //
    //                 "        {\r\n" + //
    //                 "            \"name\": \"raw_gain\",\r\n" + //
    //                 "            \"value\": 0\r\n" + //
    //                 "        },\r\n" + //
    //                 "        {\r\n" + //
    //                 "            \"name\": \"gain\",\r\n" + //
    //                 "            \"value\": 0\r\n" + //
    //                 "        },\r\n" + //
    //                 "        {\r\n" + //
    //                 "            \"name\": \"power_line_frequency\",\r\n" + //
    //                 "            \"value\": 2\r\n" + //
    //                 "        },\r\n" + //
    //                 "        {\r\n" + //
    //                 "            \"name\": \"white_balance_temperature\",\r\n" + //
    //                 "            \"value\": 4600\r\n" + //
    //                 "        },\r\n" + //
    //                 "        {\r\n" + //
    //                 "            \"name\": \"raw_sharpness\",\r\n" + //
    //                 "            \"value\": 3\r\n" + //
    //                 "        },\r\n" + //
    //                 "        {\r\n" + //
    //                 "            \"name\": \"sharpness\",\r\n" + //
    //                 "            \"value\": 50\r\n" + //
    //                 "        },\r\n" + //
    //                 "        {\r\n" + //
    //                 "            \"name\": \"backlight_compensation\",\r\n" + //
    //                 "            \"value\": 1\r\n" + //
    //                 "        },\r\n" + //
    //                 "        {\r\n" + //
    //                 "            \"name\": \"exposure_auto\",\r\n" + //
    //                 "            \"value\": 1\r\n" + //
    //                 "        },\r\n" + //
    //                 "        {\r\n" + //
    //                 "            \"name\": \"raw_exposure_absolute\",\r\n" + //
    //                 "            \"value\": 10\r\n" + //
    //                 "        },\r\n" + //
    //                 "        {\r\n" + //
    //                 "            \"name\": \"exposure_absolute\",\r\n" + //
    //                 "            \"value\": 10\r\n" + //
    //                 "        },\r\n" + //
    //                 "        {\r\n" + //
    //                 "            \"name\": \"exposure_auto_priority\",\r\n" + //
    //                 "            \"value\": false\r\n" + //
    //                 "        }\r\n" + //
    //                 "    ],\r\n" + //
    //                 "    \"width\": 640\r\n" + //
    //                 "}");

    // CvSource outputStream =
    //     CameraServer.putVideo("Rectangle", 640, 480);

    visionThread = new VisionThread(camera, new TestPipeline(), pipeline -> {
      // if (!pipeline.filterContoursOutput().isEmpty()) {
      //   Rect r = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
      //   synchronized (imgLock) {
      //     centerX = r.x + (r.width / 2);
      //   }
      // }
    });
    visionThread.start();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  
    try {
    Pigeon2Gyro pigeon = m_robotContainer.getPigeon();
    if (pigeon != null) {
      double yawDeg = pigeon.getHeading().getDegrees();
      SmartDashboard.putNumber("PigeonYawDeg", yawDeg);
    } else {
      SmartDashboard.putString("PigeonYawDeg", "pigeon null");
    }
  } catch (Throwable t) {
    SmartDashboard.putString("PigeonYawDeg", "err: " + t.getMessage());
    t.printStackTrace();
  }
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
