package frc.robot.subsystems.drivetrain;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;

@Logged
public class SwerveDriveIOSpark implements SwerveDriveIO {

    public static final SwerveDriveConfig config = new SwerveDriveConfig(
        2, 
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
    private SwerveModuleState frontLeftState = new SwerveModuleState();
    private SwerveModuleState frontRightState = new SwerveModuleState();
    private SwerveModuleState backRightState = new SwerveModuleState();
    private SwerveModuleState backLeftState = new SwerveModuleState();

    private Voltage frontLeftDriveVolts = Volts.of(0);
    private Voltage frontRightDriveVolts = Volts.of(0);
    private Voltage backRightDriveVolts = Volts.of(0);
    private Voltage backLeftDriveVolts = Volts.of(0);

    // IMU
    private final Pigeon2Gyro pigeon = new Pigeon2Gyro(SwerveDriveConstants.kIMUID);

    // PID
    private PIDController frontLeftAngleController;
    private PIDController frontRightAngleController;
    private PIDController backRightAngleController;
    private PIDController backLeftAngleController;

    private Voltage frontLeftAngleVolts = Volts.of(0);
    private Voltage frontRightAngleVolts = Volts.of(0);
    private Voltage backRightAngleVolts = Volts.of(0);
    private Voltage backLeftAngleVolts = Volts.of(0);

    private SwerveModuleState frontLeftTarget;
    private SwerveModuleState frontRightTarget;
    private SwerveModuleState backRightTarget;
    private SwerveModuleState backLeftTarget;

    // Debug
    String test = "---";

    // wpilib kinematics
    SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
        new Translation2d(SwerveDriveConstants.kTrackWidth / 2, SwerveDriveConstants.kTrackWidth / 2), // FL
        new Translation2d(SwerveDriveConstants.kTrackWidth / 2, -SwerveDriveConstants.kTrackWidth / 2), // FR
        new Translation2d(-SwerveDriveConstants.kTrackWidth / 2, SwerveDriveConstants.kTrackWidth / 2), // BL
        new Translation2d(-SwerveDriveConstants.kTrackWidth / 2, -SwerveDriveConstants.kTrackWidth / 2) // BR
    );

    public SwerveDriveIOSpark() {
        configureMotors();
        configureControllers();
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

    private void configureControllers() {
        frontLeftAngleController = new PIDController(config.kP(), config.kI(), config.kD());
        frontRightAngleController = new PIDController(config.kP(), config.kI(), config.kD());
        backRightAngleController = new PIDController(config.kP(), config.kI(), config.kD());
        backLeftAngleController = new PIDController(config.kP(), config.kI(), config.kD());
    }

    public void powerFrontLeftDrive(Voltage volts) {
        test = volts + " volts";
        frontLeftDrive.setVoltage(volts);
    }

    /**
     * Normalize an angle in radians to between [-pi, pi).
     */
    public double normalizeAngle(double radians) {
        while (radians >= Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }
        return radians;
    }

    /**
     * Set the motor speeds to match the given input velocities.
     */
    public void driveFieldCentric(double vxMetersPerSecond, double vyMetersPerSecond, double omegaRadiansPerSecond) {
        
        test = vxMetersPerSecond + "," + vyMetersPerSecond + "," + omegaRadiansPerSecond + " drive!";

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

        // Get setpoints
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            vxMetersPerSecond, 
            vyMetersPerSecond, 
            omegaRadiansPerSecond, 
            pigeon.getHeading()
        );
        SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(speeds);
        
        frontLeftTarget = moduleStates[0];
        frontLeftTarget.optimize(frontLeftState.angle);

        frontRightTarget = moduleStates[1];
        frontRightTarget.optimize(frontRightState.angle);

        backLeftTarget = moduleStates[2];
        backLeftTarget.optimize(backLeftState.angle);

        backRightTarget = moduleStates[3];
        backRightTarget.optimize(backRightState.angle);
        
        // drive voltages
        frontLeftDriveVolts = Volts.of(frontLeftTarget.speedMetersPerSecond * SwerveDriveConstants.kDriveVoltsPerSpeed);
        frontRightDriveVolts = Volts.of(frontRightTarget.speedMetersPerSecond * SwerveDriveConstants.kDriveVoltsPerSpeed);
        backRightDriveVolts = Volts.of(backRightTarget.speedMetersPerSecond * SwerveDriveConstants.kDriveVoltsPerSpeed);
        backLeftDriveVolts = Volts.of(backLeftTarget.speedMetersPerSecond * SwerveDriveConstants.kDriveVoltsPerSpeed);

        frontLeftDrive.setVoltage(frontLeftDriveVolts);
        frontRightDrive.setVoltage(frontRightDriveVolts);
        backRightDrive.setVoltage(backRightDriveVolts);
        backLeftDrive.setVoltage(backLeftDriveVolts);

        // angle control
        frontLeftAngleVolts = Volts.of(config.kP() * normalizeAngle(frontLeftTarget.angle.getRadians() - frontLeftState.angle.getRadians()));
        frontRightAngleVolts = Volts.of(config.kP() * normalizeAngle(frontRightTarget.angle.getRadians() - frontRightState.angle.getRadians()));
        backRightAngleVolts = Volts.of(config.kP() * normalizeAngle(backRightTarget.angle.getRadians() - backRightState.angle.getRadians()));
        backLeftAngleVolts = Volts.of(config.kP() * normalizeAngle(backLeftTarget.angle.getRadians() - backLeftState.angle.getRadians()));

        // frontLeftAngleVolts = Volts.of(frontLeftAngleController.calculate(
        //     -normalizeAngle(frontLeftTarget.angle.getRadians() - frontLeftState.angle.getRadians()), 
        //     0
        // ));
        // frontRightAngleVolts = Volts.of(frontRightAngleController.calculate(
        //     -normalizeAngle(frontRightTarget.angle.getRadians() - frontRightState.angle.getRadians()), 
        //     0
        // ));
        // backRightAngleVolts = Volts.of(backRightAngleController.calculate(
        //     -normalizeAngle(backRightTarget.angle.getRadians() - backRightState.angle.getRadians()), 
        //     0
        // ));
        // backLeftAngleVolts = Volts.of(backLeftAngleController.calculate(
        //     -normalizeAngle(backLeftTarget.angle.getRadians() - backLeftState.angle.getRadians()), 
        //     0
        // ));
        
        frontLeftAngle.setVoltage(frontLeftAngleVolts);
        frontRightAngle.setVoltage(frontRightAngleVolts);
        backRightAngle.setVoltage(backRightAngleVolts);
        backLeftAngle.setVoltage(backLeftAngleVolts);
        
    }

    /**
     * Return the front left module's angle.
     */
    public Angle getFrontLeftAngle() {
        Angle rawAngle = frontLeftEncoder.getAbsolutePosition().getValue();
        return Radians.of(normalizeAngle(
            rawAngle.in(Radians) - SwerveDriveConstants.kFrontLeftAngleOffset
        ));
    }
    
    /**
     * Return the front right module's angle.
     */
    public Angle getFrontRightAngle() {
        Angle rawAngle = frontRightEncoder.getAbsolutePosition().getValue();
        return Radians.of(normalizeAngle(
            rawAngle.in(Radians) - SwerveDriveConstants.kFrontRightAngleOffset
        ));
    }
    
    /**
     * Return the back right module's angle.
     */
    public Angle getBackRightAngle() {
        Angle rawAngle = backRightEncoder.getAbsolutePosition().getValue();
        return Radians.of(normalizeAngle(
            rawAngle.in(Radians) - SwerveDriveConstants.kBackRightAngleOffset
        ));
    }
    
    /**
     * Return the back left module's angle.
     */
    public Angle getBackLeftAngle() {
        Angle rawAngle = backLeftEncoder.getAbsolutePosition().getValue();
        return Radians.of(normalizeAngle(
            rawAngle.in(Radians) - SwerveDriveConstants.kBackLeftAngleOffset
        ));
    }
    
}
