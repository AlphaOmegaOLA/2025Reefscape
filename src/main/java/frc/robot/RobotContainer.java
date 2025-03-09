package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.ControllerMap;
import frc.robot.States;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Elevator.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer 
{
    /* Controllers */
    private final XboxController driver = new XboxController(0);
    private final XboxController operator = new XboxController(1);

   /* Driver Controls */
	private final int translationAxis = 1;
	private final int strafeAxis = 0;
	private final int rotationAxis = 4;

    // Microsoft Life cam on arm
    private final UsbCamera usbcamera;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, ControllerMap.LOGO_RIGHT);
    private final POVButton climber_up = new POVButton(driver, 0);
    private final POVButton climber_down = new POVButton(driver, 180);

    /* Operator Buttons */
    // X = Algae Spool Out
    private final JoystickButton algaeSpool_out = new JoystickButton(operator, XboxController.Button.kX.value);
    // Y = Algae Intake
    private final JoystickButton algae_intake = new JoystickButton(operator, XboxController.Button.kY.value);
    // Right Bumper = Algae Outtake
    private final JoystickButton algae_outtake = new JoystickButton(operator, XboxController.Button.kRightBumper.value);
    // Left Bumper = Algae Spool In
    private final JoystickButton algaeSpool_in = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
    // A = Coral Intake Position and Angle
    private final JoystickButton coral_Intake = new JoystickButton(operator, XboxController.Button.kA.value);
    // B = Coral Start Position and Angle
    private final JoystickButton coral_Start = new JoystickButton(operator, XboxController.Button.kB.value);
    // Right Trigger = Coral1 Position and Angle
    //private final JoystickButton coral_coral1 = new JoystickButton(operator, ControllerMap.RB);
    private final Trigger coral_coral1 = new Trigger(() -> operator.getRightTriggerAxis() > 0.5);
    // Left Trigger = Coral2 Position and Angle
    //private final JoystickButton coral_coral2 = new JoystickButton(operator, ControllerMap.LB);
    private final Trigger coral_coral2 = new Trigger(() -> operator.getLeftTriggerAxis() > 0.5);


    /* Subsystems */
    private final PoseEstimator s_PoseEstimator = new PoseEstimator();
    private final Swerve s_Swerve = new Swerve(s_PoseEstimator);
    private final PIDElevator s_elevator = new PIDElevator();
    private final Climber s_Climber = new Climber();
    private final AlgaeSpool s_AlgaeSpool = new AlgaeSpool();
    private final AlgaeIntakeShooter s_AlgaeIntakeShooter = new AlgaeIntakeShooter();
    private final CoralIntakeArm s_CoralIntakeArm = new CoralIntakeArm();
    private final CoralIntakeShooter s_CoralIntakeShooter = new CoralIntakeShooter();
    private final RobotSkills coral = new RobotSkills(s_CoralIntakeArm, s_elevator);
    //private final Vision s_Vision = new Vision(s_PoseEstimator);

    /* Commands */
    private final Command c_coralIntake = coral.coralIntake();
    private final Command c_coralStart = coral.coralStart();
    private final Command c_coral1 = coral.coral1();
    private final Command c_coral2 = coral.coral2();

    /* AutoChooser */
   // private final SendableChooser<Command> autoChooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() 
    {
        s_Swerve.setDefaultCommand(
            new SwerveCommand(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> false,
                () -> false,
                () -> 0 // Dynamic heading placeholder
            )
        );

        // Manual Intake for Coral
        /* 
        s_CoralIntakeShooter.setDefaultCommand(
            Commands.run(() -> s_CoralIntakeShooter.manual(operator.getLeftY()), s_CoralIntakeShooter)
        );
        */

        // The defaults elevator PID angle
        s_elevator.setDefaultCommand(new PIDElevatorCommand(s_elevator));
        /* 
        s_elevator.setDefaultCommand(
            Commands.run(() -> s_elevator.runElevator(operator.getLeftY() * 0.2), s_elevator)
        );
        */

        // The default coral Arm PID angle
        s_CoralIntakeArm.setDefaultCommand(new CoralIntakeArmCommand(s_CoralIntakeArm));

         // Camera
         usbcamera = CameraServer.startAutomaticCapture();

        // Configure the button bindings
        configureButtonBindings();
    }

        /*//Pathplanner commands - templates
        NamedCommands.registerCommand("marker1", Commands.print("Passed marker 1"));
        NamedCommands.registerCommand("marker2", Commands.print("Passed marker 2"));
        NamedCommands.registerCommand("print hello", Commands.print("hello"));
    
        
        //Auto chooser
        autoChooser = AutoBuilder.buildAutoChooser("New Auto"); // Default auto will be `Commands.none()`
        SmartDashboard.putData("Auto Mode", autoChooser);
    }*/

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() 
    {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));

        climber_up.whileTrue(new InstantCommand(() -> s_Climber.manual(1.0)));
        climber_up.onFalse(new InstantCommand (() -> s_Climber.manual(0.0)));
        climber_down.whileTrue(new InstantCommand(() -> s_Climber.manual(-1.0)));
        climber_down.onFalse(new InstantCommand (() -> s_Climber.manual(0.0)));


        /* Operator Buttons */
        algaeSpool_out.whileTrue(new InstantCommand (() -> s_AlgaeSpool.intakeDown(-1.0)));
        algaeSpool_out.onFalse(new InstantCommand(() -> s_AlgaeSpool.intakeStop()));
        algae_intake.whileTrue(new InstantCommand(() -> s_AlgaeIntakeShooter.intake(1.0)));
        algae_intake.onFalse(new InstantCommand(() -> s_AlgaeIntakeShooter.manual(0.0)));
        algae_outtake.whileTrue(new InstantCommand(() -> s_AlgaeIntakeShooter.outtake(-1.0)));
        algae_outtake.onFalse(new InstantCommand(() -> s_AlgaeIntakeShooter.manual(0.0)));
        algaeSpool_in.whileTrue(new InstantCommand(() -> s_AlgaeSpool.intakeUp(1.0)));
        algaeSpool_in.onFalse(new InstantCommand(() -> s_AlgaeSpool.intakeStop()));
        coral_Intake.onTrue(c_coralIntake);
        //coral_Intake.whileTrue(new InstantCommand(() -> SmartDashboard.putString("buttonPressed", "CORAL INTAKE BUTTON")));
        coral_Start.onTrue(c_coralStart);
        //coral_Start.whileTrue(new InstantCommand(() -> SmartDashboard.putString("buttonPressed", "CORAL START BUTTON")));
        coral_coral1.onTrue(c_coral1);
        //coral_coral1.whileTrue(new InstantCommand(() -> SmartDashboard.putString("buttonPressed", "CORAL CORAL1 BUTTON")));
        coral_coral2.onTrue(c_coral2);
        //coral_coral2.whileTrue(new InstantCommand(() -> SmartDashboard.putString("buttonPressed", "CORAL C0RAL2 BUTTON")));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    /*public Command getAutonomousCommand() 
    {
        // An ExampleCommand will run in autonomous
        return autoChooser.getSelected();
    }*/
}