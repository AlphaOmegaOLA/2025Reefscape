// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.HardwareConfigs;

public class Climber extends SubsystemBase 
{
  /** Creates a new PIDClimber */
  private SparkMax climberMotor;
  private RelativeEncoder climberEncoder;
  private HardwareConfigs hardwareConfigs;

  public Climber() 
  {

        climberMotor = new SparkMax(ClimberConstants.Climber.CLIMBER_MOTOR_ID, MotorType.kBrushless);
        climberEncoder = climberMotor.getEncoder();
        hardwareConfigs = new HardwareConfigs();
        climberEncoder.setPosition(0);
        climberMotor.configure(hardwareConfigs.climberSparkConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }
  
  private void setMotor(double speed)
  {
    climberMotor.set(speed);
  }

  public Command slow()
  {
      return this.startEnd(() -> this.setMotor(-ClimberConstants.Climber.HALF_SPEED),
          () -> this.setMotor(0));
  }

  public Command fast()
  {
      return this.startEnd(() -> this.setMotor(-ClimberConstants.Climber.FULL_SPEED),
          () -> this.setMotor(0));
  }

  public void manual(double speed)
  {
      this.setMotor(speed);       
  }
 
 //Check if this is the correct method to get the angle
  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("CLIMBER ANGLE", climberEncoder.getPosition());
  }
}
