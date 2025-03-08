// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.HardwareConfigs;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeSpoolConstants;
import frc.robot.Constants.ElevatorConstants;
import com.revrobotics.spark.SparkClosedLoopController;


public class PIDElevator extends SubsystemBase 
{
  /** Creates a new PIDE;levator. */
  private SparkMax elevatorMotor;
  private RevThroughBoreEncoder elevatorEncoder;
  private RelativeEncoder relativeEncoder;
  private SparkClosedLoopController closedLoopController;
  private HardwareConfigs hardwareConfigs;
  public double elevatorSetpoint = 0;

  public PIDElevator() 
  {
        elevatorMotor = new SparkMax(ElevatorConstants.Elevator.ELEVATOR_MOTOR_ID, MotorType.kBrushless);
        // Need to set brake mode and inverted:true in new configuration scheme
        elevatorEncoder = new RevThroughBoreEncoder(ElevatorConstants.Elevator.ELEVATOR_ENCODER_ID);
        relativeEncoder = elevatorMotor.getEncoder();
        hardwareConfigs = new HardwareConfigs();
        elevatorMotor.configure(hardwareConfigs.elevatorSparkConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        relativeEncoder.setPosition(0);
        //elevatorEncoder.setOffset(ElevatorConstants.Elevator.ELEVATOR_ENCODER_OFFSET);
        closedLoopController = elevatorMotor.getClosedLoopController();
  }

  public void setAngle(double angle)
  {
      closedLoopController.setReference(angle, ControlType.kPosition, ClosedLoopSlot.kSlot1);
  }

 public void runElevator(double input)
 {
    elevatorMotor.set(input);
 }

 public double getAngle() 
 {
     return relativeEncoder.getPosition();
 }
 

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("ELEVATOR ENCODER ANGLE", elevatorEncoder.getAngle().getDegrees());
    SmartDashboard.putNumber("ELEVATOR SET POINT", elevatorSetpoint);
    SmartDashboard.putNumber("ELEVATOR RELATIVE ENCODER", relativeEncoder.getPosition());
  }
}
