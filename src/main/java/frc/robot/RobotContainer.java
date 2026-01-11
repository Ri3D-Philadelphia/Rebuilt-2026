package frc.robot;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.SwerveDriveConstants;
import frc.robot.subsystems.fuelIntakePivot.FuelIntakePivot;
import frc.robot.subsystems.fuelIntakeRoller.FuelIntakeRoller;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooterFlywheel.ShooterFlywheel;
import frc.robot.subsystems.shooterHood.ShooterHood;

@Logged
public class RobotContainer {
    private SwerveDrive swerveDrive = new SwerveDrive();
    // private FuelIntakePivot fuelIntakePivot = new FuelIntakePivot();
    // private FuelIntakeRoller fuelIntakeRoller = new FuelIntakeRoller();
    // private Indexer indexer = new Indexer();
    // private ShooterHood shooterHood = new ShooterHood();
    // private ShooterFlywheel shooterFlywheel = new ShooterFlywheel();
    // private Climber climber = new Climber();

    // gamepads
    private CommandXboxController driver = new CommandXboxController(0);
    private DoubleSupplier driverForward =
        () -> MathUtil.applyDeadband(driver.getLeftY(), SwerveDriveConstants.kDriveDeadband)
         * SwerveDriveConstants.kJoyDriveSpeedFactor;

    private DoubleSupplier driverStrafe =
        () -> MathUtil.applyDeadband(driver.getLeftX(), SwerveDriveConstants.kDriveDeadband)
            * SwerveDriveConstants.kJoyDriveSpeedFactor;

    private DoubleSupplier driverTurn =
        () -> -MathUtil.applyDeadband(driver.getRightX(), SwerveDriveConstants.kAngleDeadband)
            * SwerveDriveConstants.kJoyAngleSpeedFactor;






    
    // Drive motors
    private SparkMax frontLeftDrive = new SparkMax(SwerveDriveConstants.kFrontLeftDriveID, MotorType.kBrushless);
    private SparkMax frontRightDrive = new SparkMax(SwerveDriveConstants.kFrontRightDriveID, MotorType.kBrushless);
    private SparkMax backRightDrive = new SparkMax(SwerveDriveConstants.kBackRightDriveID, MotorType.kBrushless);
    private SparkMax backLeftDrive = new SparkMax(SwerveDriveConstants.kBackLeftDriveID, MotorType.kBrushless);

    public RobotContainer() {

        // Command driveCommand = swerveDrive.driveFieldCentric(
        //     driverForward.getAsDouble(), 
        //     driverStrafe.getAsDouble(), 
        //     driverTurn.getAsDouble()
        // );
        
        Trigger xButton = driver.x();
        xButton.whileTrue(
            swerveDrive.GO(frontLeftDrive)
        );
        xButton.whileFalse(
            swerveDrive.STOP(frontLeftDrive)
        );
        // xButton.whileTrue(swerveDrive.driveFieldCentric(
        //     1, 0, 0
        // ));
        // xButton.whileFalse(swerveDrive.driveFieldCentric(
        //     0, 0, 0
        // ));




        
        // swerveDrive.setDefaultCommand();
    }
    
}
