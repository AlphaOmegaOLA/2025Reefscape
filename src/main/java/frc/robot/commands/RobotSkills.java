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
import frc.robot.subsystems.Elevator.PIDElevator;

import frc.robot.States;
import frc.robot.Constants.*;

public class RobotSkills
{
    private CoralIntakeArm coralIntakeArm;
    private PIDElevator elevator;

    public RobotSkills(CoralIntakeArm coralIntakeArm, PIDElevator elevator)
    {
        this.coralIntakeArm = coralIntakeArm;
        this.elevator = elevator;
    } 

    public Command coralIntake()
    {
        return new ParallelCommandGroup(
            new InstantCommand(() -> States.coralIntakeArmState = States.CoralIntakeArmStates.intake),
            new InstantCommand(() -> States.elevatorState = States.ElevatorStates.coral0)
        );
    }

    public Command coralStart()
    {
        return new ParallelCommandGroup(
            new InstantCommand(() -> States.coralIntakeArmState = States.CoralIntakeArmStates.coral0),
            new InstantCommand(() -> States.elevatorState = States.ElevatorStates.coral0)
        );
    }

    public Command coral1()
    {
        return new ParallelCommandGroup(
            new InstantCommand(() -> States.coralIntakeArmState = States.CoralIntakeArmStates.coral1),
            new InstantCommand(() -> States.elevatorState = States.ElevatorStates.coral1)
        );
    }

    public Command coral2()
    {
        return new ParallelCommandGroup(
            new InstantCommand(() -> States.coralIntakeArmState = States.CoralIntakeArmStates.coral2),
            new InstantCommand(() -> States.elevatorState = States.ElevatorStates.coral2)
        );
    }
}
