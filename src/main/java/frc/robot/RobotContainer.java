package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
    private FuelIntakePivot fuelIntakePivot = new FuelIntakePivot();
    private FuelIntakeRoller fuelIntakeRoller = new FuelIntakeRoller();
    private Indexer indexer = new Indexer();
    private ShooterHood shooterHood = new ShooterHood();
    private ShooterFlywheel shooterFlywheel = new ShooterFlywheel();
    private Climber climber = new Climber();

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

    public RobotContainer() {
        
        swerveDrive.setDefaultCommand(swerveDrive.driveFieldCentric(
            driverForward.getAsDouble(), 
            driverStrafe.getAsDouble(), 
            driverTurn.getAsDouble()
        ));
    }
    
}
