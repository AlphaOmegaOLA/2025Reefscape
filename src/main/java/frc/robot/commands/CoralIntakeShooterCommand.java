package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralIntakeShooter;

public class CoralIntakeShooterCommand extends Command
{
    private CoralIntakeShooter coralIntakeShooter;
    public CoralIntakeShooterCommand(CoralIntakeShooter coralIntakeShooter)
    {
        this.coralIntakeShooter = coralIntakeShooter;
        addRequirements(coralIntakeShooter);
    }

    public Command shootSlow()
    {
       return this.coralIntakeShooter.slow();
    }

    public Command shootFast()
    {
        return this.coralIntakeShooter.fast();
    }
}
