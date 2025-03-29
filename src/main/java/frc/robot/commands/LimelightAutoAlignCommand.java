package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class LimelightAutoAlignCommand extends Command {
  private Swerve swerve;
  private Vision vision;

  private final double kPTranslation = 1.2;
  private final double kPRotation = 2.0;

  private final double maxSpeed = 1.5;
  private final double maxRotSpeed = 2.0;

  private final double translationTolerance = 0.1; // meters
  private final double rotationTolerance = Math.toRadians(3.0); // radians

  public LimelightAutoAlignCommand(Swerve swerve, Vision vision) {
    this.swerve = swerve;
    this.vision = vision;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    // Ensure LEDs are on and vision mode is active, assumed handled in Vision init
  }

  @Override
  public void execute() {
    if (!vision.hasTarget()) {
      swerve.drive(new Translation2d(0, 0), 0.0, true, false);
      return;
    }

    Pose2d errorPose = vision.limelight.getRedBotPose2d(); // or getBlueBotPose2d if on blue alliance
    Pose2d robotPose = swerve.getPose();

    double dx = errorPose.getX() - robotPose.getX();
    double dy = errorPose.getY() - robotPose.getY();
    double dtheta = errorPose.getRotation().minus(robotPose.getRotation()).getRadians();

    double forwardSpeed = clamp(kPTranslation * dx, -maxSpeed, maxSpeed);
    double strafeSpeed = clamp(kPTranslation * dy, -maxSpeed, maxSpeed);
    double rotSpeed = clamp(kPRotation * dtheta, -maxRotSpeed, maxRotSpeed);

    swerve.drive(new Translation2d(forwardSpeed, strafeSpeed), rotSpeed, true, false);
  }

  @Override
  public void end(boolean interrupted) {
    swerve.drive(new Translation2d(0, 0), 0.0, true, false);
  }

  @Override
  public boolean isFinished() {
    if (!vision.hasTarget()) return false;

    Pose2d errorPose = vision.limelight.getRedBotPose2d(); // or getBlueBotPose2d
    Pose2d robotPose = swerve.getPose();

    double dx = Math.abs(errorPose.getX() - robotPose.getX());
    double dy = Math.abs(errorPose.getY() - robotPose.getY());
    double dtheta = Math.abs(errorPose.getRotation().minus(robotPose.getRotation()).getRadians());

    return dx < translationTolerance && dy < translationTolerance && dtheta < rotationTolerance;
  }

  private double clamp(double value, double min, double max) {
    return Math.max(min, Math.min(max, value));
  }
}
