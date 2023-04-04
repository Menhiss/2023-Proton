package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.*;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivingSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.FootSubsystem;
import frc.robot.subsystems.FlapSubsystem;

import frc.robot.Constants;

public class CubeAndCubeAutoCmdGroup extends SequentialCommandGroup {

    public CubeAndCubeAutoCmdGroup(GripperSubsystem gripperSubsystem, 
        DrivingSubsystem drivingSubsystem, ArmSubsystem armSubsystem, FlapSubsystem flapSubsystem) {
            addCommands(new MoveArmToLevelCommand(armSubsystem, Constants.levelThreeTarget),
            new DriveStraightCommand(drivingSubsystem, 0.75, 0.5, false),
            new GripperDropCommand(gripperSubsystem),
            new DriveStraightCommand(drivingSubsystem, 0.75, 0.6, true),
            new ArmExtendCommand(gripperSubsystem),
            new GripperCloseCommand(gripperSubsystem),
            new ArmRetractCommand(gripperSubsystem),
            new MoveArmToLevelCommand(armSubsystem, Constants.levelZeroTarget),
            new DriveStraightCommand(drivingSubsystem, 1.0, 4, true),
            new TankTurnCommand(drivingSubsystem, 180),
            new FlapLowerCommand(flapSubsystem),
            new GripperOpenCommand(gripperSubsystem),
            new DriveStraightCommand(drivingSubsystem, 0.75, 1, false),
            new ArmExtendCommand(gripperSubsystem),
            new GripperCloseCommand(gripperSubsystem),
            new ArmRetractCommand(gripperSubsystem),
            new FlapRaiseCommand(flapSubsystem),
            new TankTurnCommand(drivingSubsystem, 180),
            new DriveStraightCommand(drivingSubsystem, 1.0, 5, false),
            new MoveArmToLevelCommand(armSubsystem, Constants.levelTwoTarget),
            new DriveStraightCommand(drivingSubsystem, 0.75, 0.5, false),
            new GripperOpenCommand(gripperSubsystem));
    }
}
