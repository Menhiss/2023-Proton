// RobotBuilder Version: 5.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: RobotContainer.

package frc.robot;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.cameraserver.*;
import edu.wpi.first.cscore.UsbCamera;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.ArmSubsystem;


/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private static RobotContainer m_robotContainer = new RobotContainer();

  // The robot's subsystems
  public final CameraSubsystem m_cameraSubsystem = new CameraSubsystem();
  public final GripperSubsystem m_gripperSubsystem = new GripperSubsystem();
  public final ArmSubsystem m_armSubsystem = new ArmSubsystem();
  public final DrivingSubsystem m_drivingSubsystem = new DrivingSubsystem();
  public final FootSubsystem m_footSubsystem = new FootSubsystem();
  public final FlapSubsystem m_flapSubsystem = new FlapSubsystem();

  // public final UsbCamera m_usbCamera;

  // Joysticks
  private final XboxController m_xboxController1 = new XboxController(0);
  private final XboxController m_xboxController2 = new XboxController(1);
  
  // A chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
  * The container for the robot.  Contains subsystems, OI devices, and commands.
  */
  private RobotContainer() {

    // m_usbCamera = CameraServer.startAutomaticCapture();
    // Configure the button bindings
    configureButtonBindings();

    m_drivingSubsystem.setDefaultCommand(new TeleopDriveCommand (m_flapSubsystem, 
      m_drivingSubsystem, m_xboxController1));
    m_gripperSubsystem.setDefaultCommand(new GripperOpenCommand(m_gripperSubsystem));
    m_armSubsystem.setDefaultCommand(new MoveArmCommand(m_flapSubsystem,
      m_armSubsystem, m_xboxController2));
    // m_armSubsystem.setDefaultCommand(new ArmExtendCommand(m_gripperSubsystem));
    // m_footSubsystem.setDefaultCommand(new FootToggleCommand(m_footSubsystem));

    // SmartDashboard Buttons
   // SmartDashboard.putData("Autonomous Command", new TankTurnCommand(m_drivingSubsystem));

   // m_chooser.setDefaultOption("Autonomous Command", new TankTurnCommand(m_drivingSubsystem));

    SmartDashboard.putData("Auto Mode", m_chooser);
  }

  public static RobotContainer getInstance() {
    return m_robotContainer;
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Controller 2
    Trigger armRetractButton = new JoystickButton(m_xboxController2, XboxController.Button.kA.value);
    Trigger armExtendButton = new JoystickButton(m_xboxController2, XboxController.Button.kB.value);
    Trigger gripperCloseButton = new JoystickButton(m_xboxController2, XboxController.Button.kX.value);
    Trigger gripperOpenButton = new JoystickButton(m_xboxController2, XboxController.Button.kY.value);

    Trigger gripperToggleButton = new JoystickButton(m_xboxController2, XboxController.Button.kRightBumper.value);

    Trigger levelZeroButton = new JoystickButton(m_xboxController2, XboxController.Button.kA.value);
    Trigger levelOneButton = new JoystickButton(m_xboxController2, XboxController.Button.kX.value);
    Trigger levelTwoButton = new JoystickButton(m_xboxController2, XboxController.Button.kY.value);
    Trigger levelThreeButton = new JoystickButton(m_xboxController2, XboxController.Button.kB.value);

    // Controller 1
    Trigger toggleFootButton = new JoystickButton(m_xboxController1, XboxController.Button.kStart.value);
    Trigger balanceButton = new JoystickButton(m_xboxController1, XboxController.Button.kB.value);
    Trigger flapButton = new JoystickButton(m_xboxController1, XboxController.Button.kLeftBumper.value);

    // armRetractButton.onTrue(new ArmRetractCommand(m_gripperSubsystem));
    // armExtendButton.onTrue(new ArmExtendCommand(m_gripperSubsystem));
    // gripperCloseButton.onTrue(new GripperCloseCommand(m_gripperSubsystem));
    // gripperOpenButton.onTrue(new GripperOpenCommand(m_gripperSubsystem));

    gripperToggleButton.onTrue(new GripperDropCommand(m_gripperSubsystem));
    gripperToggleButton.onFalse(new GripperPickUpCommand(m_gripperSubsystem));

    flapButton.onTrue(new FlapCommand(m_flapSubsystem));

    toggleFootButton.onTrue(new FootToggleCommand(m_footSubsystem));
    balanceButton.onTrue(new BalanceCommand(m_drivingSubsystem, m_footSubsystem, m_gripperSubsystem,
     m_xboxController1, false));
    levelZeroButton.onTrue(new MoveArmToLevelCommand(m_flapSubsystem, m_armSubsystem, Constants.levelZeroTarget));
    levelOneButton.onTrue(new MoveArmToLevelCommand(m_flapSubsystem, m_armSubsystem, Constants.levelOneTarget));
    levelTwoButton.onTrue(new MoveArmToLevelCommand(m_flapSubsystem, m_armSubsystem, Constants.levelTwoTarget));
    levelThreeButton.onTrue(new MoveArmToLevelCommand(m_flapSubsystem, m_armSubsystem, Constants.levelThreeTarget));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
  */
  public Command getAutonomousCommand(String autoSelected) {      
    return new Level3AutoCommand(m_flapSubsystem, m_armSubsystem, m_gripperSubsystem, m_drivingSubsystem);
    // return new BalanceCommand(m_drivingSubsystem, m_footSubsystem, m_gripperSubsystem,null, true);
    // The selected command will be run in autonomous
    // switch (autoSelected){
    //   case Constants.kBalance:
    //     return new BalanceCommand(m_drivingSubsystem, m_footSubsystem, m_gripperSubsystem,
    //     null, true);

    //   case Constants.kCrossLine:
    //     return new LeaveCommunityCommand(m_gripperSubsystem, true, m_drivingSubsystem);

    //   case Constants.kConekick:
    //     return new ConeKickCommand(m_gripperSubsystem, true, m_drivingSubsystem);

    //   case Constants.Level3:
    //     return new Level3AutoCommand(m_flapSubsystem, m_armSubsystem, m_gripperSubsystem, m_drivingSubsystem);

    //   default:
    //     return null;
    // }
  }
  
  public void retractFoot(){
    m_footSubsystem.retractFoot();
  } 

  public void extendFoot(){
    //m_footSubsystem.retractFoot();
  }
}
