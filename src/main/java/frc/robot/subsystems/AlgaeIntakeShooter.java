// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeIntakeConstants;
import frc.robot.HardwareConfigs;

public class AlgaeIntakeShooter extends SubsystemBase 
{
  private SparkMax algaeIntakeShooterMotor;
  private HardwareConfigs hardwareConfigs;

  public AlgaeIntakeShooter() 
  {
    algaeIntakeShooterMotor = new SparkMax(AlgaeIntakeConstants.AlgaeIntake.ALGAE_INTAKE_MOTOR_ID, MotorType.kBrushless);
    hardwareConfigs = new HardwareConfigs();
    algaeIntakeShooterMotor.configure(hardwareConfigs.algaeIntakeSparkConfig, SparkMax.ResetMode.kNoResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
  }
  
  private void setMotor(double speed)
  {
    algaeIntakeShooterMotor.set(speed);
  }

  public Command slow()
  {
      return this.startEnd(() -> this.setMotor(-AlgaeIntakeConstants.AlgaeIntake.HALF_SPEED),
          () -> this.setMotor(0));
  }

  public Command fast()
  {
      return this.startEnd(() -> this.setMotor(-AlgaeIntakeConstants.AlgaeIntake.FULL_SPEED),
          () -> this.setMotor(0));
  }

  public void manual(double speed)
  {
    this.setMotor(speed * .35);
  }

  public void intake(double speed)
  {
    this.setMotor(speed * .35);
  }

  public void outtake(double speed)
  {
    this.setMotor(speed * .35);
  }
}
