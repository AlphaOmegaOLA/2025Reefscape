package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeSpoolConstants;
import frc.robot.Constants.CoralAngleConstants;
import frc.robot.HardwareConfigs;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.AnalogInput;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/* The Note intake rollers */
public class AlgaeSpool extends SubsystemBase
{
    private SparkMax algaeSpoolMotor;
    private boolean algaeIsDown = false;
    private RelativeEncoder algaeSpoolEncoder;
    private HardwareConfigs hardwareConfigs;
    public SparkClosedLoopController closedLoopController;

    public AlgaeSpool()
    {
        algaeSpoolMotor = new SparkMax(AlgaeSpoolConstants.AlgaeSpool.ALGAE_SPOOL_MOTOR_ID, MotorType.kBrushless);
        algaeSpoolEncoder = algaeSpoolMotor.getEncoder();
        hardwareConfigs = new HardwareConfigs();
        algaeSpoolEncoder.setPosition(0);
        algaeSpoolMotor.configure(hardwareConfigs.algaeSpoolSparkConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        closedLoopController = algaeSpoolMotor.getClosedLoopController();
    }

    public void intakeDown(double speed)
    {
        algaeSpoolMotor.set(speed);
    }

    public void intakeUp(double speed)
    {
        algaeSpoolMotor.set(speed * .8);
    }

    public double getAngle()
    {
        return algaeSpoolEncoder.getPosition();
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
        SmartDashboard.putBoolean("ALGAE DOWN", algaeIsDown);
        SmartDashboard.putNumber("ALGAE SPOOL ANGLE", algaeSpoolEncoder.getPosition());
    }
    
}
