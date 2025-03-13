package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.HardwareConfigs;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

public class AutoDriveCommand extends Command
{
    /*private final Swerve swerve;
    private final Translation2d direction;
    private final HardwareConfigs hardwareConfigs;
    private final double speed;
    private final double time;
    private final Timer timer = new Timer();

    public AutoDriveCommand(Swerve swerve, String directionStr, double distanceInches, double timeSeconds)
    {
        this.swerve = swerve;
        this.time = timeSeconds;
        hardwareConfigs = new HardwareConfigs();

        //Convert distance from inches to meters and calculate speed
        double distanceMeters = distanceInches * 0.0254;
        (this.speed) = (distanceMeters / timeSeconds);

        // Determine direction
        switch (directionStr.toLowerCase())
        {
            case "forward":
                this.direction = new Translation2d(speed, 0);
                break;
            case "backward":
                this.direction = new Translation2d(-speed, 0);
                break;
            case "right":
                this.direction = new Translation2d(0, -speed);
                break;
            case "left":
            default:
                this.direction = new Translation2d(0,speed);
                break;
        }
        
        addRequirements(swerve);
    }

    @Override
    public void initialize()
    {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute()
    {
        // Adjusting the speed and rotation based on the RevSwerveConfig and current drive state
        Translation2d adjustedDirection = direction.times(Constants.Swerve.maxSpeed);
        swerve.drive(adjustedDirection, 0, false, true);
    }

    @Override
    public void end(boolean interrupted) 
    {
        swerve.drive(new Translation2d(0, 0), 0, false, true);
        timer.stop();
    }

    @Override
    public boolean isFinished() 
    {
        return timer.get() >= time;
    }
    */
}
