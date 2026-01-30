package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Voltage;

@Logged
public class SwerveDriveConstants {

    // physical
    public static final double kTrackWidth = 0.4572; // meters

    // gamepad
    public static final double kJoyDriveSpeedFactor = 2.5;
    public static final double kJoyAngleSpeedFactor = 4.0;

    public static final double kDriveDeadband = 0.05;
    public static final double kAngleDeadband = 0.05;

    // imu
    public static final int kIMUID = 30;

    // encoders
    public static final int kFrontLeftEncoderID = 20;
    public static final int kFrontRightEncoderID = 21;
    public static final int kBackRightEncoderID = 22;
    public static final int kBackLeftEncoderID = 23;

    // motors
    public static final Voltage kNominalVoltage = Volts.of(12);

    public static final int kDriveCurrentLimit = 40;
    public static final int kAngleCurrentLimit = 20;
    
    // NOTE: this is set extremely low bc safety
    public static final double kDriveVoltsPerSpeed = 12.0 / 10.0; // units: V / (m/s)
    
    public static final double kDriveGearing = 1.0;
    public static final double kDrivePositionConversionFactor = 360 / kDriveGearing;
    public static final double kDriveVelocityConversionFactor = kDrivePositionConversionFactor / 60;
    
    public static final double kAngleGearing = 6.12 / 1.0;
    public static final double kAnglePositionConversionFactor = 360 / kAngleGearing;
    public static final double kAngleVelocityConversionFactor = kAnglePositionConversionFactor / 60;

    // drive motors
    public static final int kFrontLeftDriveID = 11;
    public static final int kFrontRightDriveID = 13;
    public static final int kBackRightDriveID = 15;
    public static final int kBackLeftDriveID = 17;

    public static final boolean kFrontLeftDriveInverted = true;
    public static final boolean kFrontRightDriveInverted = true;
    public static final boolean kBackRightDriveInverted = true;
    public static final boolean kBackLeftDriveInverted = true;

    // angle motors
    public static final int kFrontLeftAngleID = 12;
    public static final int kFrontRightAngleID = 14;
    public static final int kBackRightAngleID = 16;
    public static final int kBackLeftAngleID = 18;
    
    public static final boolean kFrontLeftAngleInverted = true;
    public static final boolean kFrontRightAngleInverted = true;
    public static final boolean kBackRightAngleInverted = true;
    public static final boolean kBackLeftAngleInverted = true;
    
    public static final double kFrontLeftAngleOffset = 0.311; // radians
    public static final double kFrontRightAngleOffset = -0.570;
    public static final double kBackRightAngleOffset = 2.095;
    public static final double kBackLeftAngleOffset = 0.970;
    
    
    
}
