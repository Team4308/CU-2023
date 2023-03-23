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
import frc.robot.subsystems.ArmExtendSystem;
import frc.robot.subsystems.ArmRotateSystem;

import frc.robot.subsystems.ClawSystem;

public class PreloadMobHigh extends SequentialCommandGroup {

    public PreloadMobHigh(DriveSystem driveSystem, ArmExtendSystem armExtendSystem, ArmRotateSystem armRotateSystem, ClawSystem clawSystem) {
        addCommands(
            //places game piece, skips docking, then passes mobility bonus line

            //game piece
            new SequentialCommandGroup(
                new InstantCommand(() -> clawSystem.solenoid1.set(Value.kReverse), clawSystem),
                new WaitCommand(0.25),
                new ArmRotate(24000, armRotateSystem),
                new ParallelRaceGroup(
                    new WaitCommand(2.5),
                    new ArmExtend(-460000, armExtendSystem)
                )
            ),
            new ParallelRaceGroup(
                new SequentialCommandGroup(
                    new DriveDistance(0.5, driveSystem),
                    new WaitCommand(0.5),
                    new InstantCommand(() -> clawSystem.solenoid1.set(Value.kForward), clawSystem),
                    new ParallelDeadlineGroup(
                        new WaitCommand(2.5),
                        new ArmExtend(0, armExtendSystem)
                    )
                ),
                new RepeatCommand(new ArmRotate(32000, armRotateSystem))),
            new ParallelDeadlineGroup(
            new ArmRotate(0, armRotateSystem),
            new DriveDistance(-2.5, driveSystem))
            // new ParallelDeadlineGroup(new WaitCommand(4), new DriveDistance(5, driveSystem))
        );
    }
}