package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeIntakeShooter;

public class AlgaeIntakeShooterCommand extends Command
{
    private AlgaeIntakeShooter algaeIntakeShooter;
    public AlgaeIntakeShooterCommand(AlgaeIntakeShooter algaeIntakeShooter)
    {
        this.algaeIntakeShooter = algaeIntakeShooter;
        addRequirements(algaeIntakeShooter);
    }

    public Command shootSlow()
    {
       return this.algaeIntakeShooter.slow();
    }

    public Command shootFast()
    {
        return this.algaeIntakeShooter.fast();
    }
}
