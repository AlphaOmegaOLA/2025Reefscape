package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.CoralIntakeArm;
import frc.robot.subsystems.CoralIntakeShooter;
import frc.robot.subsystems.Elevator.PIDElevator;
import frc.robot.RobotSkillsConstants;
import frc.robot.subsystems.Swerve;


import frc.robot.States;
import frc.robot.Constants.*;

public class RobotSkills
{
    private CoralIntakeArm coralIntakeArm;
    private PIDElevator elevator;
    private Swerve swerve;
    private RobotSkillsConstants constants;
    private CoralIntakeShooter coralShooter;

    public RobotSkills(CoralIntakeArm coralIntakeArm, PIDElevator elevator, CoralIntakeShooter shooter, Swerve swerve)
    {
        this.coralIntakeArm = coralIntakeArm;
        this.elevator = elevator;
        constants = new RobotSkillsConstants();
        this.coralShooter = shooter;
        this.swerve = swerve;
    } 

    public Command coralIntake()
    {
        return new ParallelCommandGroup(
            Commands.runOnce(() ->  States.coralIntakeArmState = States.CoralIntakeArmStates.intake),
            Commands.runOnce(() ->  States.elevatorState = States.ElevatorStates.coral0)
        );
    }

    public Command coralStart()
    {
        return new ParallelCommandGroup(
            Commands.runOnce(() ->  States.coralIntakeArmState = States.CoralIntakeArmStates.coral0),
            Commands.runOnce(() ->  States.elevatorState = States.ElevatorStates.coral0)
        );
    }

    public Command coral1()
    {
        return new ParallelCommandGroup(
            Commands.runOnce(() ->  States.coralIntakeArmState = States.CoralIntakeArmStates.coral1),
            Commands.runOnce(() ->  States.elevatorState = States.ElevatorStates.coral1)
        );
    }

    public Command coral2()
    {
        return new ParallelCommandGroup(
            Commands.runOnce(() ->  States.coralIntakeArmState = States.CoralIntakeArmStates.coral2),
            Commands.runOnce(() ->  States.elevatorState = States.ElevatorStates.coral2)
        );
    }

    public Command autoTroughCoral()
    {
        return new ParallelCommandGroup(
            Commands.runOnce(() ->  States.coralIntakeArmState = States.CoralIntakeArmStates.coral2),
            Commands.runOnce(() ->  States.elevatorState = States.ElevatorStates.coral2)
        );    
    }

    public Command shootFast()
    {
        return new InstantCommand(() -> this.coralShooter.fast());
    }

    public Command rollShortAndShoot()
    {
        return new SequentialCommandGroup
        (
            new AutoDriveCommand
            (
                this.swerve, "forward",
                constants.forwardRollInches,
                constants.backwardsRollSourceSeconds
            ),
            new ParallelCommandGroup(
                this.autoTroughCoral(),
                new SequentialCommandGroup(
                    new WaitCommand(2.0),
                    this.coralShooter.fast()
                )
            )
  
        );
    }
}
