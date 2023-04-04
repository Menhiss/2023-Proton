package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.*;

import frc.robot.subsystems.FlapSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.Constants;

public class FlapLowerCommand extends CommandBase {
    private boolean m_isFinished = false;
    private final FlapSubsystem m_flapSubsystem;
    public FlapLowerCommand (FlapSubsystem flapSubsystem) {
        m_flapSubsystem = flapSubsystem;
        addRequirements(m_flapSubsystem);
    }

    @Override
     public void initialize() {
     }
 
     // Called every time the scheduler runs while the command is scheduled.
     @Override
     public void execute() {
        m_flapSubsystem.lowerFlaps();
        m_isFinished = true;
     }
 
     // Called once the command ends or is interrupted.
     @Override
     public void end(boolean interrupted) {
     }
 
     // Returns true when the command should end.
     @Override
     public boolean isFinished() {
         return m_isFinished;
     }
 
     @Override
     public boolean runsWhenDisabled() {
         return false;
     }
 }
 
