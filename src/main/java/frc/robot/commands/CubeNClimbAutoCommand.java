package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivingSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.FlapSubsystem;
import frc.robot.subsystems.FootSubsystem;
import frc.robot.SwitchController;
import frc.robot.Constants;

public class CubeNClimbAutoCommand extends CommandBase {
    private final FlapSubsystem  m_flapSubsystem;
    private final DrivingSubsystem m_drivingSubsystem;
    private final ArmSubsystem m_armSubsystem;
    private final GripperSubsystem m_gripperSubsystem;
    private final FootSubsystem m_footSubsystem;
    boolean m_switch = true;
    private final SwitchController m_switchController = new SwitchController();
    private final double m_rampAngle = 14.0;
    private final double m_flatSpeed = 0.85;
    private final double m_powerScale = 1.02;  // scale output up/down quickly
    private boolean m_isCancelled = false;
    private double m_angle = 0.0;
    private boolean m_onRamp = false;  // are we on the ramp yet?
    private final Timer m_delay = new Timer();
    boolean m_timerStarted = false;
    private final double m_stopTime = 2;
    private final double targetAngle = 180.0;

    // Does the power calculation for a given ramp angle
    private final Timer m_timer = new Timer();  
    
    public CubeNClimbAutoCommand(DrivingSubsystem drivingSubsystem, FlapSubsystem flapSubsystem, ArmSubsystem armSubsystem, GripperSubsystem gripperSubsystem, FootSubsystem footSubsystem) {
        m_drivingSubsystem = drivingSubsystem;
        m_flapSubsystem = flapSubsystem;
        m_armSubsystem = armSubsystem;
        m_gripperSubsystem = gripperSubsystem;
        m_footSubsystem = footSubsystem;
        addRequirements(m_drivingSubsystem);
        addRequirements(m_armSubsystem);
        addRequirements(m_gripperSubsystem);
        addRequirements(m_footSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_timer.reset();
        m_timer.start();
        System.out.println("Init gyro");
        m_drivingSubsystem.gyroscope.setYawAxis(IMUAxis.kZ);
        m_drivingSubsystem.gyroscope.reset();
        return;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (m_switch){
            m_armSubsystem.goToClosedLoopPosition(Constants.levelThreeTarget);
            m_switch = false;
        }
        else if (m_timer.get() > 0.5 && m_timer.get() < 1.1)
        {
            //dwell
            m_drivingSubsystem.drive(-.75, -.75, 1.0);
        }
        else if (m_timer.get() > 1.1 && m_timer.get() <1.3){
            m_drivingSubsystem.drive(0,0, 1.0);
            m_gripperSubsystem.openGripper();
        }
        else if (m_timer.get() > 1.3 && m_timer.get() < 2.3){
            m_drivingSubsystem.drive(.75, .75, 1.0);
            if (m_timer.get() > 1.5){
                m_armSubsystem.goToClosedLoopPosition(Constants.levelZeroTarget);
                m_gripperSubsystem.closeGripper();
            }
        }
        else if (m_timer.get() > 2.3 && m_timer.get() < 2.7){
            m_drivingSubsystem.drive(0.75, -0.75, 1.0);
        }
        else if (m_timer.get() > 2.7 && m_timer.get() < 2.8){
            m_drivingSubsystem.drive(0.75, -0.75, 1.0);
        }
        else;
            m_angle = m_drivingSubsystem.getAngle();
            if (m_onRamp)
                {
                // Controller for balancing on the charger
                ExecuteRampControl();
                }
            else
                {
                // Approach the ramp
                ExecuteFlatControl();
                }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            m_drivingSubsystem.SetBrakeMode(true);
            m_drivingSubsystem.drive(0.0, 0.0, 1.0);
            m_footSubsystem.extendFoot();
            System.out.println("End!!!");
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (m_isCancelled) {
            return true;
        }

        if (!m_onRamp){
            return false;
        }

        boolean motorsOff = m_drivingSubsystem.IsStopped();
        boolean finished = false;

        if (motorsOff){
            if (m_timerStarted){
                finished = (m_delay.get() > m_stopTime);
            }   
            else{
                m_delay.get();
                m_timerStarted = true;
            } 
        }
        else{
            m_timer.reset();
        }
        return finished;
    }


    @Override
    public boolean runsWhenDisabled() {
        return false;
    }

    private void ExecuteRampControl() {
        // Sanity checks
        if (!m_onRamp)
        {
            return;
        }

        double power = m_switchController.GetPower(Math.abs(m_angle));

        if (m_angle < 0) {
            power *= -1;
        }

        power *= m_powerScale;
        System.out.println("On the ramp: Angle = " + m_angle + "Power = " + power);
        m_drivingSubsystem.drive(-power, -power, 1.0);
    }

    private void ExecuteFlatControl() {
        // see if we are on the ramp, at which point we stop running this controller
        if (Math.abs(m_angle) > m_rampAngle)
        {
            m_onRamp = true;
            return;
        }

        System.out.println("On the flat" + m_flatSpeed);
        m_drivingSubsystem.drive(-m_flatSpeed, -m_flatSpeed, 1.0);
    }
}