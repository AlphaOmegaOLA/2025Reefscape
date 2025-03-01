package frc.robot.commands;

import static edu.wpi.first.units.Units.Amp;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.States;

public class PIDClimberCommand extends Command 
{
/** Creates a new PIDClimberCommand. */
  private frc.robot.subsystems.Elevator.PIDElevator PIDElevator;

    private PIDController elevatorController;
    public PIDClimberCommand(frc.robot.subsystems.Elevator.PIDElevator PIDElevator) 
    {
        // Use addRequirements() here to declare subsystem dependencies.
        this.PIDElevator = PIDElevator;
        addRequirements(PIDElevator);

        elevatorController = new PIDController(Constants.ElevatorConstants.Elevator.ELEVATOR_P, Constants.ElevatorConstants.Elevator.ELEVATOR_I, Constants.ElevatorConstants.Elevator.ELEVATOR_D);
        elevatorController.setTolerance(Constants.ElevatorConstants.Elevator.ERROR_TOLERANCE);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() 
    {
        switch(States.elevatorState)
        {
            case hold:
                PIDElevator.setAngle(PIDElevator.getAngle());
                break;
            case coral0:
                PIDElevator.setAngle(Constants.ElevatorConstants.Elevator.ELEVATOR_START_ANGLE);
                break;
            case coral1:
                PIDElevator.setAngle(Constants.ElevatorConstants.Elevator.ELEVATOR_CORAL1_ANGLE);
                break;
            case coral2:
                PIDElevator.setAngle(Constants.ElevatorConstants.Elevator.ELEVATOR_CORAL2_ANGLE);
                break;
        }

        PIDElevator.runElevator(elevatorController.calculate(PIDElevator.getAngle(), PIDElevator.setpoint));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() 
    {
        return false;
    }
}
