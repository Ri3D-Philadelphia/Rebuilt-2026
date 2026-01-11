package frc.robot.subsystems.drivetrain;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;

public class SwerveDriveIOSpark implements SwerveDriveIO {

    public static final SwerveDriveConfig config = new SwerveDriveConfig(
        0, 
        0, 
        0
    );

    // Drive motors
    private SparkMax frontLeftDrive = new SparkMax(SwerveDriveConstants.kFrontLeftDriveID, MotorType.kBrushless);
    private SparkMax frontRightDrive = new SparkMax(SwerveDriveConstants.kFrontRightDriveID, MotorType.kBrushless);
    private SparkMax backRightDrive = new SparkMax(SwerveDriveConstants.kBackRightDriveID, MotorType.kBrushless);
    private SparkMax backLeftDrive = new SparkMax(SwerveDriveConstants.kBackLeftDriveID, MotorType.kBrushless);

    // Angle motors
    private SparkMax frontLeftAngle = new SparkMax(SwerveDriveConstants.kFrontLeftAngleID, MotorType.kBrushless);
    private SparkMax frontRightAngle = new SparkMax(SwerveDriveConstants.kFrontRightAngleID, MotorType.kBrushless);
    private SparkMax backRightAngle = new SparkMax(SwerveDriveConstants.kBackRightAngleID, MotorType.kBrushless);
    private SparkMax backLeftAngle = new SparkMax(SwerveDriveConstants.kBackLeftAngleID, MotorType.kBrushless);
    
    // Absolute encoders
    private CANcoder frontLeftEncoder = new CANcoder(SwerveDriveConstants.kFrontLeftEncoderID);
    private CANcoder frontRightEncoder = new CANcoder(SwerveDriveConstants.kFrontRightEncoderID);
    private CANcoder backRightEncoder = new CANcoder(SwerveDriveConstants.kBackRightEncoderID);
    private CANcoder backLeftEncoder = new CANcoder(SwerveDriveConstants.kBackLeftEncoderID);

    // Module states
    SwerveModuleState frontLeftState;
    SwerveModuleState frontRightState;
    SwerveModuleState backRightState;
    SwerveModuleState backLeftState;

    // wpilib kinematics
    SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
        new Translation2d(SwerveDriveConstants.kTrackWidth / 2, SwerveDriveConstants.kTrackWidth / 2), // FL
        new Translation2d(SwerveDriveConstants.kTrackWidth / 2, -SwerveDriveConstants.kTrackWidth / 2), // FR
        new Translation2d(-SwerveDriveConstants.kTrackWidth / 2, SwerveDriveConstants.kTrackWidth / 2), // BL
        new Translation2d(-SwerveDriveConstants.kTrackWidth / 2, -SwerveDriveConstants.kTrackWidth / 2) // BR
    );

    public SwerveDriveIOSpark() {
        configureMotors();
        frontLeftState = new SwerveModuleState(
            0, 
            new Rotation2d(getFrontLeftAngle())
        );
        frontRightState = new SwerveModuleState(
            0, 
            new Rotation2d(getFrontRightAngle())
        );
        backRightState = new SwerveModuleState(
            0, 
            new Rotation2d(getBackRightAngle())
        );
        backLeftState = new SwerveModuleState(
            0, 
            new Rotation2d(getBackLeftAngle())
        );
    }

    /**
     * Configures the SparkMaxes.
     */
    private void configureMotors() {
        // drive
        frontLeftDrive.configure(
            new SparkMaxConfig()
                .inverted(SwerveDriveConstants.kFrontLeftDriveInverted)
                .voltageCompensation(SwerveDriveConstants.kNominalVoltage.in(Volts))
                .smartCurrentLimit(SwerveDriveConstants.kDriveCurrentLimit)
                .apply(
                    new EncoderConfig()
                        .velocityConversionFactor(
                            SwerveDriveConstants.kDriveVelocityConversionFactor)
                        .positionConversionFactor(
                            SwerveDriveConstants.kDrivePositionConversionFactor)),
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
        frontRightDrive.configure(
            new SparkMaxConfig()
                .inverted(SwerveDriveConstants.kFrontRightDriveInverted)
                .voltageCompensation(SwerveDriveConstants.kNominalVoltage.in(Volts))
                .smartCurrentLimit(SwerveDriveConstants.kDriveCurrentLimit)
                .apply(
                    new EncoderConfig()
                        .velocityConversionFactor(
                            SwerveDriveConstants.kDriveVelocityConversionFactor)
                        .positionConversionFactor(
                            SwerveDriveConstants.kDrivePositionConversionFactor)),
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
        backRightDrive.configure(
            new SparkMaxConfig()
                .inverted(SwerveDriveConstants.kBackRightDriveInverted)
                .voltageCompensation(SwerveDriveConstants.kNominalVoltage.in(Volts))
                .smartCurrentLimit(SwerveDriveConstants.kDriveCurrentLimit)
                .apply(
                    new EncoderConfig()
                        .velocityConversionFactor(
                            SwerveDriveConstants.kDriveVelocityConversionFactor)
                        .positionConversionFactor(
                            SwerveDriveConstants.kDrivePositionConversionFactor)),
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
        backLeftDrive.configure(
            new SparkMaxConfig()
                .inverted(SwerveDriveConstants.kBackLeftDriveInverted)
                .voltageCompensation(SwerveDriveConstants.kNominalVoltage.in(Volts))
                .smartCurrentLimit(SwerveDriveConstants.kDriveCurrentLimit)
                .apply(
                    new EncoderConfig()
                        .velocityConversionFactor(
                            SwerveDriveConstants.kDriveVelocityConversionFactor)
                        .positionConversionFactor(
                            SwerveDriveConstants.kDrivePositionConversionFactor)),
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        // angle
        frontLeftAngle.configure(
            new SparkMaxConfig()
                .inverted(SwerveDriveConstants.kFrontLeftAngleInverted)
                .voltageCompensation(SwerveDriveConstants.kNominalVoltage.in(Volts))
                .smartCurrentLimit(SwerveDriveConstants.kAngleCurrentLimit)
                .apply(
                    new EncoderConfig()
                        .velocityConversionFactor(
                            SwerveDriveConstants.kAngleVelocityConversionFactor)
                        .positionConversionFactor(
                            SwerveDriveConstants.kAnglePositionConversionFactor)),
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
        frontRightAngle.configure(
            new SparkMaxConfig()
                .inverted(SwerveDriveConstants.kFrontRightAngleInverted)
                .voltageCompensation(SwerveDriveConstants.kNominalVoltage.in(Volts))
                .smartCurrentLimit(SwerveDriveConstants.kAngleCurrentLimit)
                .apply(
                    new EncoderConfig()
                        .velocityConversionFactor(
                            SwerveDriveConstants.kAngleVelocityConversionFactor)
                        .positionConversionFactor(
                            SwerveDriveConstants.kAnglePositionConversionFactor)),
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
        backRightAngle.configure(
            new SparkMaxConfig()
                .inverted(SwerveDriveConstants.kBackRightAngleInverted)
                .voltageCompensation(SwerveDriveConstants.kNominalVoltage.in(Volts))
                .smartCurrentLimit(SwerveDriveConstants.kAngleCurrentLimit)
                .apply(
                    new EncoderConfig()
                        .velocityConversionFactor(
                            SwerveDriveConstants.kAngleVelocityConversionFactor)
                        .positionConversionFactor(
                            SwerveDriveConstants.kAnglePositionConversionFactor)),
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
        backLeftAngle.configure(
            new SparkMaxConfig()
                .inverted(SwerveDriveConstants.kBackLeftAngleInverted)
                .voltageCompensation(SwerveDriveConstants.kNominalVoltage.in(Volts))
                .smartCurrentLimit(SwerveDriveConstants.kAngleCurrentLimit)
                .apply(
                    new EncoderConfig()
                        .velocityConversionFactor(
                            SwerveDriveConstants.kAngleVelocityConversionFactor)
                        .positionConversionFactor(
                            SwerveDriveConstants.kAnglePositionConversionFactor)),
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

    }

    /**
     * Set the motor speeds to match the given input velocities.
     */
    public void driveFieldCentric(double vxMetersPerSecond, double vyMetersPerSecond, double omegaRadiansPerSecond) {
        // update module states
        frontLeftState = new SwerveModuleState(
            0, 
            new Rotation2d(getFrontLeftAngle())
        );
        frontRightState = new SwerveModuleState(
            0, 
            new Rotation2d(getFrontRightAngle())
        );
        backRightState = new SwerveModuleState(
            0, 
            new Rotation2d(getBackRightAngle())
        );
        backLeftState = new SwerveModuleState(
            0, 
            new Rotation2d(getBackLeftAngle())
        );

        // TODO: dummy code
        ChassisSpeeds speeds = new ChassisSpeeds(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond);
        SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(speeds);
        
        SwerveModuleState frontLeftTarget = moduleStates[0];
        SwerveModuleState frontRightTarget = moduleStates[1];
        SwerveModuleState backLeftTarget = moduleStates[2];
        SwerveModuleState backRightTarget = moduleStates[3];        
        
        // just set drive voltages
        Voltage frontLeftDriveVolts = Volts.of(frontLeftTarget.speedMetersPerSecond * SwerveDriveConstants.kDriveVoltsPerSpeed);
        Voltage frontRightDriveVolts = Volts.of(frontRightTarget.speedMetersPerSecond * SwerveDriveConstants.kDriveVoltsPerSpeed);
        Voltage backRightDriveVolts = Volts.of(backRightTarget.speedMetersPerSecond * SwerveDriveConstants.kDriveVoltsPerSpeed);
        Voltage backLeftDriveVolts = Volts.of(backLeftTarget.speedMetersPerSecond * SwerveDriveConstants.kDriveVoltsPerSpeed);

        frontLeftDrive.setVoltage(frontLeftDriveVolts);
        frontRightDrive.setVoltage(frontRightDriveVolts);
        backRightDrive.setVoltage(backRightDriveVolts);
        backLeftDrive.setVoltage(backLeftDriveVolts);
    }

    /**
     * Return the front left module's angle.
     */
    public Angle getFrontLeftAngle() {
        return frontLeftEncoder.getAbsolutePosition().getValue();
    }
    
    /**
     * Return the front right module's angle.
     */
    public Angle getFrontRightAngle() {
        return frontRightEncoder.getAbsolutePosition().getValue();
    }
    
    /**
     * Return the back right module's angle.
     */
    public Angle getBackRightAngle() {
        return backRightEncoder.getAbsolutePosition().getValue();
    }
    
    /**
     * Return the back left module's angle.
     */
    public Angle getBackLeftAngle() {
        return backLeftEncoder.getAbsolutePosition().getValue();
    }
    
}
