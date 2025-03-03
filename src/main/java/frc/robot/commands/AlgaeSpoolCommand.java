package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeSpool;

public class AlgaeSpoolCommand extends Command
{
    private AlgaeSpool algaeSpool;
    public AlgaeSpoolCommand(AlgaeSpool algaeSpool)
    {
        this.algaeSpool = algaeSpool;
        addRequirements(algaeSpool);
    }

    public Command climbSlow()
    {
       return this.algaeSpool.slow();
    }

    public Command climbFast()
    {
        return this.algaeSpool.fast();
    }
}
