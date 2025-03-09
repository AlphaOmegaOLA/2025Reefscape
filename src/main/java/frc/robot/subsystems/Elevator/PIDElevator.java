// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Robot;

public class PIDElevator extends SubsystemBase {
  /** Single-motor elevator system **/

  // Initialize the motor (CHANGE ID TO MATCH YOUR MOTOR CONTROLLER ID)
  private SparkMax elevatorMotor = new SparkMax(ElevatorConstants.Elevator.ELEVATOR_MOTOR_ID, MotorType.kBrushless);
  
  // PID Controller (TUNE GAINS FOR YOUR SYSTEM)
  private PIDController elevatorController = new PIDController(
      ElevatorConstants.Elevator.ELEVATOR_P,
      ElevatorConstants.Elevator.ELEVATOR_I,
      ElevatorConstants.Elevator.ELEVATOR_D);
  
  // Encoder attached to the motor
  private RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder();
  
  // Setpoint tracking
  private double elevatorSetpoint;

  public PIDElevator() {
    // Configure the motor (UPDATE CONFIGURATION AS NEEDED)
    elevatorMotor.configure(Robot.hardwareConfigs.elevatorSparkConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    // Initialize position (SET TO ZERO OR INITIAL HEIGHT)
    elevatorEncoder.setPosition(0);
  }

  public void runToSetpoint(double setpoint) {
    // Compute PID output (ENSURE kF IS CORRECTLY SCALED)
    double input = elevatorController.calculate(elevatorEncoder.getPosition(), setpoint);
    setVoltage(input);
    elevatorSetpoint = setpoint;
  }

  public void setVoltage(double voltage) {
    elevatorMotor.setVoltage(voltage);
  }

  public double getPosition() {
    return elevatorEncoder.getPosition();
  }

  public void manual(double speed)
  {
    elevatorMotor.set(speed);
  }

  public double getCurrent() {
    return elevatorMotor.getOutputCurrent();
  }

  public double getError() {
    return elevatorSetpoint - getPosition();
  }

  public double getVoltage() {
    return elevatorMotor.getAppliedOutput() * 12;
  }

  public void setPosition(double rotations) {
    elevatorEncoder.setPosition(rotations);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Elevator Position", getPosition());
    SmartDashboard.putNumber("Elevator Voltage", getVoltage());
    SmartDashboard.putNumber("Elevator Error", getError());
    SmartDashboard.putNumber("Elevator Setpoint", elevatorSetpoint);
  }
}
