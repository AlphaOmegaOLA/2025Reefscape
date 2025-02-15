package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;

import com.revrobotics.spark.config.SparkMaxConfig;

public final class HardwareConfigs 
{
  
    public CANcoderConfiguration swerveCANcoderConfig = new CANcoderConfiguration();
    public SparkMaxConfig elevatorSparkConfig =  new SparkMaxConfig();
    public SparkMaxConfig swerveDriveSparkConfig =  new SparkMaxConfig();

    public HardwareConfigs()
    {
        /** Swerve CANCoder Configuration */
        swerveCANcoderConfig.MagnetSensor.SensorDirection = Constants.Swerve.cancoderInvert;

        //Swerve angle motor config
        //Motor inverts and Neutral modes
        elevatorSparkConfig.inverted(Constants.Swerve.angleMotorInvert);
        elevatorSparkConfig.idleMode(Constants.Swerve.angleNeutralMode);

        //Gear ratio and wrapping config
        elevatorSparkConfig.encoder.positionConversionFactor(360/Constants.Swerve.angleGearRatio);
        elevatorSparkConfig.encoder.velocityConversionFactor(Constants.Swerve.angleGearRatio / 60);
        elevatorSparkConfig.closedLoop.positionWrappingEnabled(true);

        //current limiting
        elevatorSparkConfig.smartCurrentLimit(40);

        //PID config
        swerveDriveSparkConfig.closedLoop.p(Constants.Swerve.driveKP);
        swerveDriveSparkConfig.closedLoop.i(Constants.Swerve.driveKI);
        swerveDriveSparkConfig.closedLoop.d(Constants.Swerve.driveKD);

        //Swerve drive motor config
        //Motor inverts and Neutral modes
        swerveDriveSparkConfig.inverted(Constants.Swerve.driveMotorInvert);
        swerveDriveSparkConfig.idleMode(Constants.Swerve.driveNeutralMode);

        //Gear ratio and wrapping config
        swerveDriveSparkConfig.encoder.positionConversionFactor(Constants.Swerve.driveGearRatio);
        swerveDriveSparkConfig.closedLoop.positionWrappingEnabled(true);

        //current limiting
        swerveDriveSparkConfig.smartCurrentLimit(40);

        //PID config
        swerveDriveSparkConfig.closedLoop.p(Constants.Swerve.driveKP);
        swerveDriveSparkConfig.closedLoop.i(Constants.Swerve.driveKI);
        swerveDriveSparkConfig.closedLoop.d(Constants.Swerve.driveKD);

        elevatorSparkConfig.openLoopRampRate(Constants.Swerve.openLoopRamp);
        elevatorSparkConfig.closedLoopRampRate(Constants.Swerve.closedLoopRamp);

        /** Elevator Configuration */

        //Elevator motor config
        //Motor inverts and neutral modes
        elevatorSparkConfig.inverted(Constants.ElevatorConstants.Elevator.ELEVATOR_MOTOR_INVERTED);
        elevatorSparkConfig.idleMode(Constants.ElevatorConstants.Elevator.ELEVATOR_NEUTRAL_MODE);

        //Gear ratio and wrapping config
        elevatorSparkConfig.encoder.positionConversionFactor(360/Constants.Swerve.angleGearRatio);
        elevatorSparkConfig.encoder.velocityConversionFactor(Constants.ElevatorConstants.Elevator.ELEVATOR_GEAR_RATIO / 60);
        elevatorSparkConfig.closedLoop.positionWrappingEnabled(true);

        //current limiting
        elevatorSparkConfig.smartCurrentLimit(40);

        //PID config
        swerveDriveSparkConfig.closedLoop.p(Constants.Swerve.driveKP);
        swerveDriveSparkConfig.closedLoop.i(Constants.Swerve.driveKI);
        swerveDriveSparkConfig.closedLoop.d(Constants.Swerve.driveKD);

        //Swerve drive motor config
        //Motor inverts and Neutral modes
        swerveDriveSparkConfig.inverted(Constants.Swerve.driveMotorInvert);
        swerveDriveSparkConfig.idleMode(Constants.Swerve.driveNeutralMode);

        //Gear ratio and wrapping config
        swerveDriveSparkConfig.encoder.positionConversionFactor(Constants.Swerve.driveGearRatio);
        swerveDriveSparkConfig.closedLoop.positionWrappingEnabled(true);

        //current limiting
        swerveDriveSparkConfig.smartCurrentLimit(40);

        //PID config
        swerveDriveSparkConfig.closedLoop.p(Constants.Swerve.driveKP);
        swerveDriveSparkConfig.closedLoop.i(Constants.Swerve.driveKI);
        swerveDriveSparkConfig.closedLoop.d(Constants.Swerve.driveKD);

        elevatorSparkConfig.openLoopRampRate(Constants.Swerve.openLoopRamp);
        elevatorSparkConfig.closedLoopRampRate(Constants.Swerve.closedLoopRamp);
    }
}