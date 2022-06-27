package frc.robot.commands;

import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.DoubleSupplier;

public class DriveCommand extends CommandBase {
    private final DrivetrainSubsystem drivetrain;
    private final DoubleSupplier translationXSupplier;
    private final DoubleSupplier translationYSupplier;
    private final DoubleSupplier rotationSupplier;
    private boolean driveMode;

    /**
   * Creates a Drive Command for the curent vector goal of the robot
   * @param drivetrain A DrivetrainSubsystem to drive.
   * @param translationXSupplier A DoubleSupplier providing the X translasion power on a scale from -1 to 1. 
   * @param translationYSupplier A DoubleSupplier providing the Y translasion power on a scale from -1 to 1. 
   * @param translationXSupplier A DoubleSupplier providing the rotation power on a scale from  -1 to 1.
   * @param driveMode a boolean true for robot oriented false for field oriented.
   */
    public DriveCommand(
            DrivetrainSubsystem drivetrain,
            DoubleSupplier translationXSupplier,
            DoubleSupplier translationYSupplier,
            DoubleSupplier rotationSupplier,
            boolean driveMode
    ) {
        this.drivetrain = drivetrain;
        this.translationXSupplier = translationXSupplier;
        this.translationYSupplier = translationYSupplier;
        this.rotationSupplier = rotationSupplier;
        this.driveMode = driveMode;

        addRequirements(drivetrain);
    }
    public void ToggleDriveMode(){
        driveMode = !driveMode;
    }

    @Override
    public void execute() {
        double translationXPercent = translationXSupplier.getAsDouble();
        double translationYPercent = translationYSupplier.getAsDouble();
        double rotationPercent = rotationSupplier.getAsDouble();
        ChassisSpeeds chassisSpeed;

        if(driveMode == true){
        //For robot oriented
        chassisSpeed = new ChassisSpeeds(
            translationXPercent * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            translationYPercent * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            rotationPercent * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);
        }else{
        //For field oriented
        chassisSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(
            translationXPercent * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            translationYPercent * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            rotationPercent * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
            drivetrain.getRotation());
        }

        drivetrain.drive(chassisSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the drivetrain
        drivetrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
