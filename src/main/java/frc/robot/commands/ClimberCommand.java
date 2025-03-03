package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class ClimberCommand extends Command
{
    private Climber climber;
    public ClimberCommand(Climber climber)
    {
        this.climber = climber;
        addRequirements(climber);
    }

    public Command climbSlow()
    {
       return this.climber.slow();
    }

    public Command climbFast()
    {
        return this.climber.fast();
    }
}
