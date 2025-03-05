package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeSpoolConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/* The Note intake rollers */
public class AlgaeSpool extends SubsystemBase
{
    private Spark algaeSpoolMotor;

    public AlgaeSpool()
    {
        algaeSpoolMotor = new Spark(AlgaeSpoolConstants.AlgaeSpool.ALGAE_SPOOL_MOTOR_ID);
    }

    // Take a Note in
    public void intake(double speed)
    {
        // while (speed != 0)
        algaeSpoolMotor.set(speed * .2);
        // if (speed == 0)
        // { algaeSpoolMotor.set(speed);}
        // Use the while or if statement to make the motor stop when button is released
    }

    // Run intake at reduced speed
    public Command slow()
    {
        return this.startEnd(() -> this.algaeSpoolMotor.set(AlgaeSpoolConstants.AlgaeSpool.HALF_SPEED), 
            () -> this.algaeSpoolMotor.set(0));
    }

    // Run intake at full speed
    public Command fast()
    {
        return this.startEnd(() -> this.algaeSpoolMotor.set(AlgaeSpoolConstants.AlgaeSpool.FULL_SPEED), 
            () -> this.algaeSpoolMotor.set(0));
    }

    public void intakeStop()
    {
        algaeSpoolMotor.set(0);
    } 

    // Periodically check the status of the intake to see
    // if a note is detected and print the status to the dashboard.
    public void periodic()
    {

    }
    
}
