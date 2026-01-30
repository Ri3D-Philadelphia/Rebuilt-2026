package frc.robot;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.climber.Climb;
import frc.robot.subsystems.drivetrain.Pigeon2Gyro;


import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
import frc.robot.subsystems.shooterFlywheel.ShooterFlywheelConstants;
import frc.robot.subsystems.shooterHood.ShooterHoodConstants;



@Logged
public class RobotContainer {
    private SwerveDrive swerveDrive = new SwerveDrive();
    private FuelIntakePivot fuelIntakePivot = new FuelIntakePivot();
    private FuelIntakeRoller fuelIntakeRoller = new FuelIntakeRoller(fuelIntakePivot);
    private Indexer indexer = new Indexer();
    private final ShooterHood hood = new ShooterHood();
    private final ShooterFlywheel shooter = new ShooterFlywheel();
    private Climb climb = new Climb
    ();

  
    private final Pigeon2Gyro pigeon = new Pigeon2Gyro(30); // CAN ID 30

    
    //     public Pigeon2Gyro getPigeon() {
    //         return this.pigeon;
    //     }

    // gamepads
    private CommandXboxController driver = new CommandXboxController(0);
    private DoubleSupplier driverForward =
        () -> -MathUtil.applyDeadband(driver.getLeftY(), SwerveDriveConstants.kDriveDeadband)
         * SwerveDriveConstants.kJoyDriveSpeedFactor;

    private DoubleSupplier driverStrafe =
        () -> -MathUtil.applyDeadband(driver.getLeftX(), SwerveDriveConstants.kDriveDeadband)
            * SwerveDriveConstants.kJoyDriveSpeedFactor;

    private DoubleSupplier driverTurn =
        () -> MathUtil.applyDeadband(driver.getRightX(), SwerveDriveConstants.kAngleDeadband)
            * SwerveDriveConstants.kJoyAngleSpeedFactor;

     public RobotContainer() {

        configureBindings();
        
        // fuelIntakePivot.home().schedule(); // TODO: Test this

       //  m_shooterFlywheel.setDefaultCommand(m_shooterFlywheel.set(0));

     }

    private void configureBindings() {

        // ============= DRIVE CONTROLS =============
      
        swerveDrive.setDefaultCommand(swerveDrive.driveFieldCentric(
            driverForward, 
            driverStrafe, 
            driverTurn
        ));


        // ========= CLIMB CONTROLS =========

        driver.a().whileTrue(climb.setVoltage1());
        driver.b().whileTrue(climb.setVoltageminus1());
        driver.start().whileTrue(climb.resetEncoder());
        climb.setDefaultCommand(climb.stop());


         // ==================== SHOOTING ====================
        
        shooter.setDefaultCommand(shooter.stop());

        // driver.rightTrigger().onTrue(shooter.powerFlywheel()).onFalse(shooter.stop());
        driver.rightTrigger().onTrue(shooter.spinToDefault()).onFalse(shooter.stop());
        
        // Shoot via two indexers
        Trigger rightBumper = driver.rightBumper();
        rightBumper.onTrue(indexer.runIndexerOnce().andThen(indexer.runSecondIndexerOnce()));

        

        // ==================== HOOD ANGLE ADJUSTMENT ====================

        // DPad Up → Increase hood angle (shoot farther)
        driver.povUp().onTrue(hood.adjustAngleCommand(0.25));
        
        // DPad Down → Decrease hood angle (shoot closer)
        driver.povDown().onTrue(hood.adjustAngleCommand(-0.25));
        

        // ==================== GYRO RESET ====================
        
        // Left Stick & Right Stick (pressed together) → Reset gyro
        driver.leftStick().and(driver.rightStick()).onTrue(
            Commands.runOnce(() -> {
                swerveDrive.resetGyro(); 
                System.out.println("Gyro reset!");
            }).withName("ResetGyro")
        );

        // /// Fuel Intake
        // // Hold position commands (subsystems continuously run control or set motor voltages)
        // fuelIntakePivot.setDefaultCommand(
        //     fuelIntakePivot.holdPosition()
        // );

        // fuelIntakeRoller.setDefaultCommand(
        //     fuelIntakeRoller.holdRoller()
        // );


        // // Intake commands
        // Trigger leftBumper = driver.leftBumper();
        // Trigger leftTrigger = driver.leftTrigger();

        // leftTrigger.onTrue(
        //     Commands.runOnce(() -> {
        //         if (fuelIntakePivot.isExtended()
        //             && fuelIntakeRoller.isRollingIn()) {

        //             // Toggle OFF
        //             fuelIntakeRoller.setStopped();
        //             fuelIntakePivot.setRetracted();

        //         } else {
        //             // Turn ON / Change direction
        //             fuelIntakePivot.setExtended();
        //             fuelIntakeRoller.setRollIn();
        //         }
        //     })
        // );

        // leftBumper.onTrue(
        //     Commands.runOnce(() -> {
        //         if (fuelIntakePivot.isExtended()
        //             && fuelIntakeRoller.isRollingOut()) {

        //             // Toggle OFF
        //             fuelIntakeRoller.setStopped();
        //             fuelIntakePivot.setRetracted();

        //         } else {
        //             // Turn ON / Change direction
        //             fuelIntakePivot.setExtended();
        //             fuelIntakeRoller.setRollOut();
        //         }
        //     })
        // );



    }
    
}

