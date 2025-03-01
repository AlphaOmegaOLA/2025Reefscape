// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.PIDClimber;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class PIDClimber extends SubsystemBase 
{
  /** Creates a new PIDClimber */
  private SparkMax climberMotor;
  public double setpoint = 0;

  public PIDClimber() 
  {
        climberMotor = new SparkMax(ElevatorConstants.Elevator.CLIMBER_MOTOR_ID, MotorType.kBrushless);
  }

  public void setAngle(double angle)
  {
    setpoint = angle;
  }

 public void runElevator(double input)
 {
  climberMotor.set(input);
 }

 public double getAngle() 
 {
     return elevatorEncoder.getAngle().getDegrees();
 }
 

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("ARM ANGLE", elevatorEncoder.getAngle().getDegrees());
  }
}
