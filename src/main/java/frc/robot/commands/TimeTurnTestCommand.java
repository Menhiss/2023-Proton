package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.DrivingSubsystem;
import frc.robot.subsystems.GripperSubsystem;

public class TimeTurnTestCommand extends CommandBase {
    private final DrivingSubsystem m_drivingSubsystem;
    private final GripperSubsystem m_kickSubsystem;

    // Does the power calculation for a given ramp angle
    private final Timer m_delay = new Timer();
    boolean m_timerStarted = false;
    private boolean m_kickCube = false;    
    
    public TimeTurnTestCommand(GripperSubsystem kickSubsystem, boolean kickCube, DrivingSubsystem drivingSubsystem) {
        m_kickSubsystem = kickSubsystem;
        m_drivingSubsystem = drivingSubsystem;
        m_kickCube = kickCube;
        addRequirements(m_drivingSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_delay.reset();
        m_delay.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
            m_drivingSubsystem.drive(.75, -.75, 1);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            m_drivingSubsystem.drive(0, 0, 1);
            System.out.println("End!!!");
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        boolean finished = (m_delay.get() > 1.0);
        return finished;
    }


    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
}