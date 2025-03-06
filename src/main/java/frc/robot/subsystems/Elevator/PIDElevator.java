// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class PIDElevator extends SubsystemBase 
{
  /** Creates a new PIDE;levator. */
  private SparkMax elevatorMotor;
  private RevThroughBoreEncoder elevatorEncoder;
  public double setpoint = 0;

  public PIDElevator() 
  {
        elevatorMotor = new SparkMax(ElevatorConstants.Elevator.ELEVATOR_MOTOR_ID, MotorType.kBrushless);
        // Need to set brake mode and inverted:true in new configuration scheme
        elevatorEncoder = new RevThroughBoreEncoder(ElevatorConstants.Elevator.ELEVATOR_ENCODER_ID);
        elevatorEncoder.setOffset(ElevatorConstants.Elevator.ELEVATOR_ENCODER_OFFSET);
  }

  public void setAngle(double angle)
  {
    setpoint = angle;
  }

 public void runElevator(double input)
 {
    elevatorMotor.set(input);
 }

 public double getAngle() 
 {
     return elevatorEncoder.getAngle().getDegrees();
 }
 

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("ELEVATOR ENCODER ANGLE", elevatorEncoder.getAngle().getDegrees());
    SmartDashboard.putNumber("ELEVATOR SET POINT", setpoint);
  }
}
