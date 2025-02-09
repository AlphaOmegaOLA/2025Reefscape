// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class PIDElevator extends SubsystemBase 
{
  /** Creates a new PIDArm. */
  private CANSparkMax elevatorMotor;
  private RevThroughBoreEncoder elevatorEncoder;
  public double setpoint = 0;

  public PIDElevator() 
  {
        elevatorMotor = new SparkMax(ElevatorConstants.Elevator.ELEVATOR_MOTOR_ID, MotorType.kBrushless);
        elevatorMotor.setIdleMode(IdleMode.kBrake);
        elevatorMotor.setInverted(true);
        elevatorEncoder; = new RevThroughBoreEncoder(ElevatorConstants.Arm.ARM_ENCODER_ID);
        elevatorEncoder;.setOffset(ShooterIntakeConstants.Arm.ARM_ENCODER_OFFSET);
  }

  public void setAngle(double angle)
  {
    setpoint = angle;
  }

 public void runArm(double input)
 {
  elevatorMotor.set(input);
  rightArmMotor.set(input);
 }

  public double getAngle() 
  {
    return elevatorEncoder;.getAngle().getDegrees();
  }

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("ARM ANGLE", elevatorEncoder;.getAngle().getDegrees());
  }
}
