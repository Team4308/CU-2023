package frc.robot.commands.auto.groups;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.auto.DriveDistance;
import frc.robot.subsystems.DriveSystem;
import frc.robot.commands.ArmRotateCommand;
import frc.robot.commands.DockingCommand;
import frc.robot.commands.auto.ArmExtend;
import frc.robot.commands.auto.ArmRotate;
import frc.robot.commands.auto.ArmRotateHold;
import frc.robot.subsystems.ArmExtendSystem;
import frc.robot.subsystems.ArmRotateSystem;

import frc.robot.subsystems.ClawSystem;

public class PreloadMobHigh extends SequentialCommandGroup {

    public PreloadMobHigh(DriveSystem driveSystem, ArmExtendSystem armExtendSystem, ArmRotateSystem armRotateSystem, ClawSystem clawSystem) {
        addCommands(
            //places game piece on high node, skips docking, then passes mobility bonus line

            //game piece
            new SequentialCommandGroup(
                new InstantCommand(() -> clawSystem.solenoid1.set(Value.kReverse), clawSystem),
                new WaitCommand(0.25),
                new ArmRotate(29000, armRotateSystem),
                new ParallelRaceGroup(
                    new WaitCommand(3),
                    new ArmExtend(-600000, armExtendSystem),
                    new RepeatCommand(new ArmRotateHold(29000, armRotateSystem))
                )
            ),
            new ParallelRaceGroup(
                new SequentialCommandGroup(
                    //new WaitCommand(0.25),
                    new DriveDistance(0.75, driveSystem),
                    new WaitCommand(0.25),
                    new InstantCommand(() -> clawSystem.solenoid1.set(Value.kForward), clawSystem),
                    new WaitCommand(1),
                    new ParallelDeadlineGroup(
                        new WaitCommand(1.5),
                        new ArmExtend(-30000, armExtendSystem)
                    )
                ),
                new RepeatCommand(new ArmRotateHold(31000, armRotateSystem))
                ),
            
            new SequentialCommandGroup(
                new DriveDistance(-0.5, driveSystem),
                new ParallelDeadlineGroup(
                new ArmRotate(6000, armRotateSystem),
                new InstantCommand(() -> clawSystem.solenoid1.set(Value.kReverse), clawSystem)
            
                ),
            new DriveDistance(-4.0, driveSystem)
            )
            
            // new ParallelDeadlineGroup(new WaitCommand(4), new DriveDistance(5, driveSystem))
        );
    }
}