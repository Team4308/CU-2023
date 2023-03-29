package frc.robot.commands.auto.groups;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.DockingCommand;
import frc.robot.commands.auto.DriveDistance;
import frc.robot.commands.auto.TurnAngle;
import frc.robot.commands.auto.TurnDistance;
import frc.robot.subsystems.DriveSystem;

import frc.robot.commands.auto.ArmExtend;
import frc.robot.commands.auto.ArmRotate;
import frc.robot.commands.auto.ArmRotateHold;
import frc.robot.subsystems.ArmExtendSystem;
import frc.robot.subsystems.ArmRotateSystem;

import frc.robot.subsystems.ClawSystem;

public class PreloadDock extends SequentialCommandGroup {

    public PreloadDock(DriveSystem driveSystem, ArmExtendSystem armExtendSystem, ArmRotateSystem armRotateSystem, ClawSystem clawSystem) {
        driveSystem.resetAngle();
        addCommands(
            //Places game piece, turns around and docks

            //game piece (copied from PreloadMob)
            new SequentialCommandGroup(
                new InstantCommand(() -> clawSystem.solenoid1.set(Value.kReverse), clawSystem),
                new WaitCommand(0.5),
                new ArmRotate(29000, armRotateSystem)
            ),
            new ParallelRaceGroup(
                new SequentialCommandGroup(
                    new ParallelRaceGroup(
                        new WaitCommand(1.75),
                        new ArmExtend(-280000, armExtendSystem)
                    ),
                          
                    new InstantCommand(() -> clawSystem.solenoid1.set(Value.kForward), clawSystem),
                    new SequentialCommandGroup(
                        new WaitCommand(.25),
                        new ParallelDeadlineGroup(
                            new WaitCommand(1.25),
                            new ArmExtend(-30000, armExtendSystem)
                        )
                    )
                ),
                new RepeatCommand(new ArmRotateHold(29000, armRotateSystem))
            ),

            //Movement and docking (guessed values)
            new SequentialCommandGroup(
                
                new WaitCommand(0.5),
                new ParallelDeadlineGroup(
                    new ArmRotate(3000, armRotateSystem),
                    new DriveDistance(-0.5, driveSystem)),
                new TurnAngle(180, driveSystem),
                // new TurnDistance(1.2, -1.2, driveSystem),
                new DriveDistance(1.5, driveSystem),
                // new ParallelDeadlineGroup(
                //         new WaitCommand(4),
                        new DockingCommand(driveSystem)
                // )
                
            )
        );
    }
}