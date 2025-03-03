package frc.robot.commands;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CoralAngleConstants;
import frc.robot.States;
import frc.robot.subsystems.CoralIntakeArm;

public class CoralIntakeArmCommand extends Command
{
    private CoralIntakeArm coralIntakeArm;
    private double currentCoralIntakeArmTarget;
    public CoralIntakeArmCommand(CoralIntakeArm coralIntakeArm)
    {
        this.coralIntakeArm = coralIntakeArm;
        addRequirements(coralIntakeArm);
    }

    @Override
    public void execute()
    {
        switch (States.coralIntakeArmState) 
        {
          case coral0:
            currentCoralIntakeArmTarget = CoralAngleConstants.CoralAngle.CORAL_ANGLE_START_ANGLE;
            break;
          case coral1:
            currentCoralIntakeArmTarget = CoralAngleConstants.CoralAngle.CORAL_ANGLE_CORAL1_ANGLE;            
            break;
          case coral2:
            currentCoralIntakeArmTarget = CoralAngleConstants.CoralAngle.CORAL_ANGLE_CORAL2_ANGLE;            
            break;
          case intake:
            currentCoralIntakeArmTarget = CoralAngleConstants.CoralAngle.CORAL_ANGLE_INTAKE_ANGLE;            
            break;
          default:
            currentCoralIntakeArmTarget = CoralAngleConstants.CoralAngle.CORAL_ANGLE_START_ANGLE;            
            break;
        } 
        coralIntakeArm.setAngle(currentCoralIntakeArmTarget);
    }
}
