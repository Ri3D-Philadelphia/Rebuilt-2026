package frc.robot;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.climber.Climb;
import frc.robot.subsystems.drivetrain.Pigeon2Gyro;


import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.SwerveDriveConstants;
import frc.robot.subsystems.fuelIntakePivot.FuelIntakePivot;
import frc.robot.subsystems.fuelIntakeRoller.FuelIntakeRoller;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooterFlywheel.ShooterFlywheel;
import frc.robot.subsystems.shooterHood.ShooterHood;



@Logged
public class RobotContainer {
    // private SwerveDrive swerveDrive = new SwerveDrive();
    private FuelIntakePivot fuelIntakePivot = new FuelIntakePivot();
    private FuelIntakeRoller fuelIntakeRoller = new FuelIntakeRoller(fuelIntakePivot);
    private Indexer indexer = new Indexer();
    // private ShooterHood shooterHood = new ShooterHood();
    // private final ShooterFlywheel m_shooterFlywheel = new ShooterFlywheel();
    // private Climb climb = new Climb();

  
    private final Pigeon2Gyro pigeon = new Pigeon2Gyro(30); // CAN ID 30

    
        public Pigeon2Gyro getPigeon() {
            return this.pigeon;
        }

    // gamepads
    private CommandXboxController driver = new CommandXboxController(0);
    private DoubleSupplier driverForward =
        () -> -MathUtil.applyDeadband(driver.getLeftY(), SwerveDriveConstants.kDriveDeadband)
         * SwerveDriveConstants.kJoyDriveSpeedFactor;

    private DoubleSupplier driverStrafe =
        () -> -MathUtil.applyDeadband(driver.getLeftX(), SwerveDriveConstants.kDriveDeadband)
            * SwerveDriveConstants.kJoyDriveSpeedFactor;

    private DoubleSupplier driverTurn =
        () -> -MathUtil.applyDeadband(driver.getRightX(), SwerveDriveConstants.kAngleDeadband)
            * SwerveDriveConstants.kJoyAngleSpeedFactor;

     public RobotContainer() {

        configureBindings1();
        configureBindings2();
        
        // fuelIntakePivot.home().schedule(); // TODO: Test this

        // m_shooterFlywheel.setDefaultCommand(m_shooterFlywheel.set(0));

     }

    private void configureBindings1() {
        // // Configure your button bindings here
        //  // Schedule `setVelocity` when the Xbox controller's B button is pressed,
        // // cancelling on release.
        // driver.a().whileTrue(m_shooterFlywheel.setSpeed(RotationsPerSecond.of(60)));
        // driver.b().whileTrue(m_shooterFlywheel.setSpeed(RotationsPerSecond.of(300)));
        // // Schedule `set` when the Xbox controller's B button is pressed,
        // // cancelling on release.
        // driver.x().whileTrue(m_shooterFlywheel.set(0.3));
        // driver.y().whileTrue(m_shooterFlywheel.set(-0.3));
      
      
        // call in a periodic or button action:

        // new RunCommand(() -> {
            
        // }, swerveDrive).schedule();
        
        // Trigger xButton = driver.x();
        // xButton.whileTrue(
        //     swerveDrive.GO()
        // );
        // xButton.whileFalse(
        //     swerveDrive.STOP()
        // );
        // xButton.whileTrue(swerveDrive.driveFieldCentric(
        //     1, 0, 0
        // ));
        // xButton.whileFalse(swerveDrive.driveFieldCentric(
        //     0, 0, 0
        // ));

        // Trigger lJoy = driver.leftStick();

        // xButton.whileTrue(swerveDrive.driveFieldCentric(
        //     driverForward, 
        //     driverStrafe, 
        //     driverTurn
        // ));
        // xButton.whileFalse(swerveDrive.driveFieldCentric(
        //     () -> 0, 
        //     () -> 0, 
        //     () -> 0
        // ));


        
        // swerveDrive.setDefaultCommand(driveCommand);

                // Using CommandXboxController driver (you have this)
        driver.y().onTrue(new InstantCommand(() -> {
            pigeon.zeroHeading();
            System.out.println("[PIGEON] zeroed");
        }));



        // new RunCommand(() -> {
        //     double yawDeg = pigeon.getHeading().getDegrees();
        //     SmartDashboard.putNumber("PigeonYawDeg", yawDeg);      // Shuffleboard / SmartDashboard
        //     System.out.println("[PIGEON] yaw deg = " + yawDeg);   // console log (driver station)
        // }).schedule();
   
      
    }

    private void configureBindings2() {
        // Hold position commands (subsystems continuously run control or set motor voltages)
        fuelIntakePivot.setDefaultCommand(
            fuelIntakePivot.holdPosition()
        );

        fuelIntakeRoller.setDefaultCommand(
            fuelIntakeRoller.holdRoller()
        );






        // // TODO: Testing intake homing
        // // TODO: TEMPORARY BINDING, REMOVE LATER
        // Trigger backButton = driver.back();
        // backButton.onTrue(fuelIntakePivot.home());






        // Intake commands
        Trigger leftBumper = driver.leftBumper();
        Trigger leftTrigger = driver.leftTrigger();

        leftTrigger.onTrue(
            Commands.runOnce(() -> {
                if (fuelIntakePivot.isExtended()
                    && fuelIntakeRoller.isRollingIn()) {

                    // Toggle OFF
                    fuelIntakeRoller.setStopped();
                    fuelIntakePivot.setRetracted();

                } else {
                    // Turn ON / Change direction
                    fuelIntakePivot.setExtended();
                    fuelIntakeRoller.setRollIn();
                }
            })
        );

        leftBumper.onTrue(
            Commands.runOnce(() -> {
                if (fuelIntakePivot.isExtended()
                    && fuelIntakeRoller.isRollingOut()) {

                    // Toggle OFF
                    fuelIntakeRoller.setStopped();
                    fuelIntakePivot.setRetracted();

                } else {
                    // Turn ON / Change direction
                    fuelIntakePivot.setExtended();
                    fuelIntakeRoller.setRollOut();
                }
            })
        );




        // TODO: Testing indexer 
        // TODO: TEMPORARY BINDING, REMOVE LATER

        Trigger rightBumper = driver.rightBumper();
        rightBumper.onTrue(indexer.runIndexerOnce().andThen(indexer.runSecondIndexerOnce()));





    }
    
}

