package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;

import com.revrobotics.spark.config.SparkMaxConfig;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

public final class HardwareConfigs 
{
    public CANcoderConfiguration swerveCANcoderConfig = new CANcoderConfiguration();
    public SparkMaxConfig swerveAngleSparkConfig =  new SparkMaxConfig();
    public SparkMaxConfig swerveDriveSparkConfig =  new SparkMaxConfig();
    public SparkMaxConfig elevatorSparkConfig =  new SparkMaxConfig();
    public SparkMaxConfig coralAngleSparkConfig= new SparkMaxConfig();
    public SparkMaxConfig coralIntakeSparkConfig = new SparkMaxConfig();
    public SparkMaxConfig algaeIntakeSparkConfig = new SparkMaxConfig();
    public SparkMaxConfig climberSparkConfig = new SparkMaxConfig();
    public SparkMaxConfig algaeSpoolSparkConfig = new SparkMaxConfig();

    public HardwareConfigs()
    {
        /** Swerve CANCoder Configuration */
       swerveCANcoderConfig.MagnetSensor.SensorDirection = Constants.Swerve.cancoderInvert;

       //Swerve angle motor config
       //Motor inverts and nuetral modes
       swerveAngleSparkConfig.inverted(Constants.Swerve.angleMotorInvert);
       swerveAngleSparkConfig.idleMode(Constants.Swerve.angleNeutralMode);

       //Gear ratio and wrapping config
       swerveAngleSparkConfig.encoder.positionConversionFactor(360/Constants.Swerve.angleGearRatio);
       swerveAngleSparkConfig.encoder.velocityConversionFactor(Constants.Swerve.angleGearRatio / 60);
       swerveAngleSparkConfig.closedLoop.positionWrappingEnabled(true);

       //current limiting
       swerveAngleSparkConfig.smartCurrentLimit(40);

       //PID config
       swerveDriveSparkConfig.closedLoop.p(Constants.Swerve.driveKP);
       swerveDriveSparkConfig.closedLoop.i(Constants.Swerve.driveKI);
       swerveDriveSparkConfig.closedLoop.d(Constants.Swerve.driveKD);

       //Swerve drive motor config
       //Motor inverts and nuetral modes
       swerveDriveSparkConfig.inverted(Constants.Swerve.driveMotorInvert);
       swerveDriveSparkConfig.idleMode(Constants.Swerve.driveNeutralMode);

       //Gear ratio and wrapping config
       swerveDriveSparkConfig.encoder.positionConversionFactor(Constants.Swerve.wheelCircumference / Constants.Swerve.driveGearRatio);
       swerveDriveSparkConfig.closedLoop.positionWrappingEnabled(true);

       //current limiting
       swerveDriveSparkConfig.smartCurrentLimit(40);

       //PID config
       swerveDriveSparkConfig.closedLoop.p(Constants.Swerve.driveKP);
       swerveDriveSparkConfig.closedLoop.i(Constants.Swerve.driveKI);
       swerveDriveSparkConfig.closedLoop.d(Constants.Swerve.driveKD);

       swerveAngleSparkConfig.openLoopRampRate(Constants.Swerve.openLoopRamp);
       swerveAngleSparkConfig.closedLoopRampRate(Constants.Swerve.closedLoopRamp);

        /** Elevator Configuration */
        //Elevator motor config
        //Motor inverts and neutral modes
        elevatorSparkConfig.inverted(Constants.ElevatorConstants.Elevator.ELEVATOR_MOTOR_INVERTED);
        elevatorSparkConfig.idleMode(Constants.ElevatorConstants.Elevator.ELEVATOR_NEUTRAL_MODE);

        //Gear ratio and wrapping config
        elevatorSparkConfig.encoder.positionConversionFactor(360/Constants.Swerve.angleGearRatio);
        elevatorSparkConfig.encoder.velocityConversionFactor(Constants.ElevatorConstants.Elevator.ELEVATOR_GEAR_RATIO / 20);
        elevatorSparkConfig.closedLoop.positionWrappingEnabled(true);

        //current limiting
        elevatorSparkConfig.smartCurrentLimit(40);

        //PID config
        elevatorSparkConfig.closedLoop.p(Constants.ElevatorConstants.Elevator.ELEVATOR_P);
        elevatorSparkConfig.closedLoop.i(Constants.ElevatorConstants.Elevator.ELEVATOR_I);
        elevatorSparkConfig.closedLoop.d(Constants.ElevatorConstants.Elevator.ELEVATOR_D);

         /** Climber Configuration */

        //Climber motor config
        //Motor inverts and neutral modes
        climberSparkConfig.inverted(Constants.ClimberConstants.Climber.CLIMBER_MOTOR_INVERTED);
        climberSparkConfig.idleMode(Constants.ClimberConstants.Climber.CLIMBER_NEUTRAL_MODE);

        //Gear ratio and wrapping config
        climberSparkConfig.encoder.positionConversionFactor(360/Constants.Swerve.angleGearRatio);
        climberSparkConfig.encoder.velocityConversionFactor(Constants.ClimberConstants.Climber.CLIMBER_GEAR_RATIO / 100);
        climberSparkConfig.closedLoop.positionWrappingEnabled(false);

        //current limiting
        climberSparkConfig.smartCurrentLimit(40);


        /** Coral Angle Configuration */
        //Coral angle motor config
        //Motor inverts and neutral modes
        coralAngleSparkConfig.inverted(Constants.CoralAngleConstants.CoralAngle.CORAL_ANGLE_MOTOR_INVERTED);
        coralAngleSparkConfig.idleMode(Constants.CoralAngleConstants.CoralAngle.CORAL_ANGLE_NEUTRAL_MODE);

        //Gear ratio and wrapping config
        coralAngleSparkConfig.encoder.positionConversionFactor(360/Constants.Swerve.angleGearRatio);
        coralAngleSparkConfig.encoder.velocityConversionFactor(Constants.CoralAngleConstants.CoralAngle.CORAL_ANGLE_GEAR_RATIO / 20);
        coralAngleSparkConfig.closedLoop.positionWrappingEnabled(false);

        //current limiting
        coralAngleSparkConfig.smartCurrentLimit(40);
        
        //PID config
        coralAngleSparkConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        coralAngleSparkConfig.closedLoop.p(Constants.CoralAngleConstants.CoralAngle.CORAL_ANGLE_P);
        coralAngleSparkConfig.closedLoop.i(Constants.CoralAngleConstants.CoralAngle.CORAL_ANGLE_I);
        coralAngleSparkConfig.closedLoop.d(Constants.CoralAngleConstants.CoralAngle.CORAL_ANGLE_D);

        /** Coral Intake Configuration */
        //Coral intake motor config
        //Motor inverts and neutral modes
        coralIntakeSparkConfig.inverted(Constants.CoralIntakeConstants.CoralIntake.CORAL_INTAKE_MOTOR_INVERTED);
        coralIntakeSparkConfig.idleMode(Constants.CoralIntakeConstants.CoralIntake.CORAL_INTAKE_NEUTRAL_MODE);

        //Gear ratio and wrapping config
        coralIntakeSparkConfig.encoder.positionConversionFactor(360/Constants.Swerve.angleGearRatio);
        coralIntakeSparkConfig.encoder.velocityConversionFactor(Constants.CoralIntakeConstants.CoralIntake.CORAL_INTAKE_GEAR_RATIO / 5);
        coralIntakeSparkConfig.closedLoop.positionWrappingEnabled(false);

        //current limiting
        coralIntakeSparkConfig.smartCurrentLimit(40);

        /** Algae Intake Configuration */
        //Algae intake motor config
        //Motor inverts and neutral modes
        algaeIntakeSparkConfig.inverted(Constants.AlgaeIntakeConstants.AlgaeIntake.ALGAE_INTAKE_MOTOR_INVERTED);
        algaeIntakeSparkConfig.idleMode(Constants.AlgaeIntakeConstants.AlgaeIntake.ALGAE_INTAKE_NEUTRAL_MODE);

        //Gear ratio and wrapping config
        algaeIntakeSparkConfig.encoder.positionConversionFactor(360/Constants.Swerve.angleGearRatio);
        algaeIntakeSparkConfig.encoder.velocityConversionFactor(Constants.AlgaeIntakeConstants.AlgaeIntake.ALGAE_INTAKE_GEAR_RATIO / 1);
        algaeIntakeSparkConfig.closedLoop.positionWrappingEnabled(false);

        //current limiting
        algaeIntakeSparkConfig.smartCurrentLimit(40);

        /** Algae Spool Configuration */
        // Algae Spool motor config
        // Motor inverts and neutral modes
        algaeSpoolSparkConfig.inverted(Constants.AlgaeSpoolConstants.AlgaeSpool.ALGAE_SPOOL_MOTOR_INVERTED);
        algaeSpoolSparkConfig.idleMode(Constants.AlgaeSpoolConstants.AlgaeSpool.ALGAE_SPOOL_NEUTRAL_MODE);

        //Gear ratio and wrapping config
        algaeSpoolSparkConfig.encoder.positionConversionFactor(360/Constants.Swerve.angleGearRatio);
        algaeSpoolSparkConfig.encoder.velocityConversionFactor(Constants.AlgaeSpoolConstants.AlgaeSpool.ALGAE_SPOOL_GEAR_RATIO / 64);
        algaeSpoolSparkConfig.closedLoop.positionWrappingEnabled(false);

        //current limiting
        algaeSpoolSparkConfig.smartCurrentLimit(40);
    }
}