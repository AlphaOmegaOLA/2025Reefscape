// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralAngleConstants;
import frc.robot.HardwareConfigs;

public class CoralIntakeArm extends SubsystemBase 
{
  private SparkMax coralIntakeArmMotor;
  private RelativeEncoder coralIntakeArmEncoder;
  private HardwareConfigs hardwareConfigs;
  public SparkClosedLoopController closedLoopController;
  public double currentCoralIntakeArmTarget = 0.0;

  public CoralIntakeArm() 
  {
    coralIntakeArmMotor = new SparkMax(CoralAngleConstants.CoralAngle.CORAL_ANGLE_MOTOR_ID, MotorType.kBrushless);
    coralIntakeArmEncoder = coralIntakeArmMotor.getEncoder();
    coralIntakeArmEncoder.setPosition(0);
    coralIntakeArmMotor.configure(hardwareConfigs.coralAngleSparkConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    closedLoopController = coralIntakeArmMotor.getClosedLoopController();
  }
  
  public void setAngle(double angle)
  {
    closedLoopController.setReference(angle, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  public double getAngle() 
  {
    return coralIntakeArmEncoder.getPosition();
  }
 
 //Check if this is the correct method to get the angle
  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("CLIMBER ANGLE", coralIntakeArmEncoder.getPosition());
  }
}
