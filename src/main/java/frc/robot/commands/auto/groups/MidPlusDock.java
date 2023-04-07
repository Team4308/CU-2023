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
import frc.robot.commands.auto.DriveTillTipped;
import frc.robot.subsystems.DriveSystem;
import frc.robot.commands.ArmRotateCommand;
import frc.robot.commands.DockingCommand;
import frc.robot.commands.auto.ArmExtend;
import frc.robot.commands.auto.ArmRotate;
import frc.robot.commands.auto.ArmRotateHold;
import frc.robot.subsystems.ArmExtendSystem;
import frc.robot.subsystems.ArmRotateSystem;

import frc.robot.subsystems.ClawSystem;

public class MidPlusDock extends SequentialCommandGroup {

    public MidPlusDock(DriveSystem driveSystem, ArmExtendSystem armExtendSystem, ArmRotateSystem armRotateSystem, ClawSystem clawSystem) {
        // places cube on mid then docks
        // arm needs to be at 29000 to place cube on mid
        addCommands(
            // pickup cube and raise arm up about halfway
            new SequentialCommandGroup(
                new InstantCommand(() -> clawSystem.solenoid1.set(Value.kReverse), clawSystem), // 0.0
                new WaitCommand(0.25), // 0.25
                new ArmRotate(19000, armRotateSystem) // ???
            ),
            // continue raising arm while extending
            // this saves time
            new ParallelRaceGroup(
                new ParallelRaceGroup(
                    new ArmRotateHold(29000, armRotateSystem),
                    new ArmExtend(-250000, armExtendSystem)
                ),
                new WaitCommand(2)
            ),
            new ParallelRaceGroup(
                new SequentialCommandGroup(
                    // release cube
                    new SequentialCommandGroup(
                        new InstantCommand(() -> clawSystem.solenoid1.set(Value.kForward), clawSystem),
                        new WaitCommand(0.5)
                    ),
                    // retract arm slightly
                    new ParallelRaceGroup(
                        new ArmExtend(-20000, armExtendSystem),
                        new WaitCommand(1)
                    )
                ),
                new ArmRotateHold(29000, armRotateSystem)
            ), // 0.5
            // retract arm while driving toward docking station
            // this saves time
            new ParallelDeadlineGroup(
                new ParallelRaceGroup(
                    new DriveTillTipped(-12, -0.4, driveSystem),
                    new WaitCommand(3)
                ),
                new ArmExtend(-20000, armExtendSystem),
                new ArmRotate(19000, armRotateSystem)
            ),
            new ParallelDeadlineGroup(
                new ParallelRaceGroup(
                    new DriveTillTipped(12, -0.2, driveSystem),
                    new WaitCommand(3)
                ),
                new ArmExtend(-20000, armExtendSystem),
                new ArmRotate(2000, armRotateSystem)
            ),
            new DockingCommand(driveSystem)
            // new WaitCommand(0.4),
            // new DockingCommand(driveSystem),
            // new WaitCommand(0.4),
            // new DockingCommand(driveSystem),
            // new WaitCommand(0.4),
            // new DockingCommand(driveSystem)
        );
    }
}
