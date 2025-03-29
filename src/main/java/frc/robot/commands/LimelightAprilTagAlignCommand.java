package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class LimelightAprilTagAlignCommand extends Command {
  private final Swerve swerve;
  private final Vision vision;

  private final double kPTranslation = 1.2;
  private final double kPRotation = 2.0;

  private final double maxSpeed = 1.5;
  private final double maxRotSpeed = 2.0;

  public LimelightAprilTagAlignCommand(Swerve swerve, Vision vision) {
    this.swerve = swerve;
    this.vision = vision;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    // Vision settings like LED/camMode assumed handled elsewhere
  }

  @Override
  public void execute() {
    if (!vision.hasTarget()) {
      swerve.drive(new Translation2d(0, 0), 0.0, true, false);
      return;
    }

    Pose2d targetPose = vision.limelight.getRedBotPose2d(); // or getBlueBotPose2d()
    Pose2d currentPose = swerve.getPose();

    double dx = targetPose.getX() - currentPose.getX();
    double dy = targetPose.getY() - currentPose.getY();
    double dtheta = targetPose.getRotation().minus(currentPose.getRotation()).getRadians();

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
    return false; // Runs continuously during teleop while held
  }

  private double clamp(double value, double min, double max) {
    return Math.max(min, Math.min(max, value));
  }
}
