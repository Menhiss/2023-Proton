// RobotBuilder Version: 5.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: Command.

package frc.robot.commands;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivingSubsystem;



/**
 *
 // Bob's notes:
 // TODO: I don't think we need the PIDController, as we are using the internal Talon controller.
 // - based 2019 bot code for the racks
 */
public class DriveForwardCommand extends CommandBase {
    private final DrivingSubsystem m_DrivingSubsystem;
    private final PIDController m_PIDController;
    
    public DriveForwardCommand(DrivingSubsystem drivingSubsystem, double distance) {
        m_DrivingSubsystem = drivingSubsystem;
        m_PIDController = new PIDController(3, 0, 0.8);
        m_PIDController.setSetpoint(distance);
        addRequirements(m_DrivingSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_PIDController.reset();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
     //todo   double speed = m_PIDController.calculate(m_DrivingSubsystem.getEncoderValue());
    //todo  m_DrivingSubsystem.drive(speed, speed, 1.0);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_DrivingSubsystem.drive(0, 0, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public boolean runsWhenDisabled() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
        return false;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
    }
}
