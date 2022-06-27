package frc.robot;

import com.cpr3663.Library;

import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Button;

public class RobotContainer {
    private final DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();

    private final XboxController controller = new XboxController(Constants.DRIVE_CONTROLLER_PORT);

    private final DriveCommand driveCommand;

    public RobotContainer() {
        drivetrain.register();

        driveCommand = new DriveCommand(
            drivetrain,
            () -> -modifyAxis(controller.getLeftY()), // Axes are flipped here on purpose
            () -> -modifyAxis(controller.getLeftX()),
            () -> -modifyAxis(controller.getRightX()),
            false);

        drivetrain.setDefaultCommand(driveCommand);

        //Zero Gryo
        new Button(controller::getBackButtonPressed)
                .whenPressed(drivetrain::zeroGyroscope);
        
        //Change Drive Mode
        new Button(controller::getStartButtonPressed)
                .whenPressed(driveCommand::ToggleDriveMode);
    }

    public DrivetrainSubsystem getDrivetrain() {
        return drivetrain;
    }

    private static double deadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
            if (value > 0.0) {
                return (value - deadband) / (1.0 - deadband);
            } else {
                return (value + deadband) / (1.0 - deadband);
            }
        } else {
            return 0.0;
        }
    }

    private static double modifyAxis(double value) {
        // Deadband
        value = deadband(value, 0.05);

        // Square the axis
        value = Math.copySign(value * value, value);

        return value;
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return null;
    }
}
