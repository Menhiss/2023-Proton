package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivingSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.FlapSubsystem;
import frc.robot.Constants;

public class CubeNCubeAutoCommand extends CommandBase {
    private final FlapSubsystem  m_flapSubsystem;
    private final DrivingSubsystem m_drivingSubsystem;
    private final ArmSubsystem m_armSubsystem;
    private final GripperSubsystem m_gripperSubsystem;
    boolean m_switch = true;
    

    // Does the power calculation for a given ramp angle
    private final Timer m_timer = new Timer();  
    
    public CubeNCubeAutoCommand(FlapSubsystem flapSubsystem, ArmSubsystem armSubsystem, GripperSubsystem gripperSubsystem, DrivingSubsystem drivingSubsystem) {
        m_flapSubsystem = flapSubsystem;
        m_armSubsystem = armSubsystem;
        m_gripperSubsystem = gripperSubsystem;
        m_drivingSubsystem = drivingSubsystem;
        addRequirements(m_drivingSubsystem);
        addRequirements(m_armSubsystem);
        addRequirements(m_gripperSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_timer.reset();
        m_timer.start();
        return;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() { 
        if (m_switch){
            m_armSubsystem.goToClosedLoopPosition(Constants.levelThreeTarget);
            m_switch = false;
        }
        else if (m_timer.get() > 0.5 && m_timer.get() < 1.0)
        {
            m_gripperSubsystem.extendArm();
            m_drivingSubsystem.drive(-1.0, -1.0, 1.0);
        }
        else if (m_timer.get() > 1.1 && m_timer.get() <1.2){
            m_drivingSubsystem.drive(0,0, 1.0);
            m_gripperSubsystem.openGripper();
        }
        else if (m_timer.get() > 1.2 && m_timer.get() < 5.2){
            m_drivingSubsystem.drive(1.0, 1.0, 1.0);
            if (m_timer.get() > 1.4){
                m_armSubsystem.goToClosedLoopPosition(Constants.levelZeroTarget);
                m_gripperSubsystem.closeGripper();
            }
        }
        else if (m_timer.get() > 5.2 && m_timer.get() < 5.3){
            m_drivingSubsystem.drive(0, 0, 1.0);
        }
        else {

            //pseudo code for rest, not possible until I know how to tank turn

            //if tank turn isn't done
                //tank turn 180

            //else
                //drop flaps
                //retract arm
                //open gripper
                //stop timer
                //reset timer
                //start timer

                //if timer is less than 2 seconds
                    //drive forward

                //else if timer is greater than 2 seconds
                    //extend arm
                    //close gripper
                    //retract arm
                    //raise flaps

                    //if tank turn 2 isn't done
                        //tank turn 180
                    
                    //else
                        //stop timer
                        //reset timer
                        //start timer
                        //if timer is less than 5 seconds
                            //drive forward

                            //see what happens

                            //if timer is greater than 4.3 seconds
                                //raise arm

                        //else 
                            //open gripper
                    


        }

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            System.out.println("End!!!");
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        boolean finished = (m_timer.get() > 10);
        return finished;
    }


    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
}