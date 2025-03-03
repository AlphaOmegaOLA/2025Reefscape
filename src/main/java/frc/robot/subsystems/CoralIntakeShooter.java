// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralIntakeConstants;

public class CoralIntakeShooter extends SubsystemBase 
{
  private SparkMax coralIntakeShooterMotor;

  public CoralIntakeShooter() 
  {
    coralIntakeShooterMotor = new SparkMax(CoralIntakeConstants.CoralIntake.CORAL_INTAKE_MOTOR_ID, MotorType.kBrushless);
  }
  
  private void setMotor(double speed)
  {
    coralIntakeShooterMotor.set(speed);
  }

  public Command slow()
  {
      return this.startEnd(() -> this.setMotor(-CoralIntakeConstants.CoralIntake.HALF_SPEED),
          () -> this.setMotor(0));
  }

  public Command fast()
  {
      return this.startEnd(() -> this.setMotor(-CoralIntakeConstants.CoralIntake.FULL_SPEED),
          () -> this.setMotor(0));
  }

  public void manual(double speed)
  {
      this.setMotor(speed);       
  }
}
