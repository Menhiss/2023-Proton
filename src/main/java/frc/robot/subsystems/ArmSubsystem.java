// RobotBuilder Version: 5.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: Subsystem.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.ErrorCode;
import frc.robot.PowerCurve;


public class ArmSubsystem extends SubsystemBase {
    //constants
    private final double kp = 0.015;
    private final int kTimeoutMS = 20;
    private WPI_TalonFX armMotor;
    private double powerExp = 2.5;
    private double m_targetPos = 0.0;
   
    public ArmSubsystem() {
        armMotor = new WPI_TalonFX(2);
        //armMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 1, 0);
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        armMotor.configAllSettings(config);
        armMotor.setSensorPhase(true);
        armMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);
        armMotor.configClosedloopRamp(0.2, kTimeoutMS);

        // todo: set F,K,P,I,D!
        armMotor.config_kF(0,0.0, kTimeoutMS);
        armMotor.config_kP(0,kp, kTimeoutMS);
        armMotor.config_kI(0,0.0, kTimeoutMS);
        armMotor.config_kD(0,0.0, kTimeoutMS);  

        ErrorCode err = armMotor.setSelectedSensorPosition(0.0);
        System.out.println("%%%%%%% robot init Position Zero Error = " + err);
            addChild("ArmMotor", armMotor);
    }

    @Override
    public void periodic() {
        armMotor.set(ControlMode.Position, m_targetPos, DemandType.AuxPID, m_targetPos);
        double encoderValue = armMotor.getSelectedSensorPosition(1);
 //       System.out.println("Encoder Value: " + encoderValue);
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void drive(double motorSpeed, double speedFactor){
        armMotor.set(PowerCurve.getPoint(motorSpeed, powerExp) * 0.5);
    }

    public void goToClosedLoopPosition(double targetPos){
        m_targetPos = targetPos;
    }
}

