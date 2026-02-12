package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.FeederSubsys;
import frc.robot.subsystems.FeederSubsys.FeederSpeed;
import frc.robot.subsystems.HoodSubsys;
import frc.robot.subsystems.ShooterSubsys;
import frc.robot.subsystems.ShooterSubsys.FuelFeedSpeed;

public final class RobotCommands {
    private static ShooterSubsys shooterSubsys;
    private static FeederSubsys feederSubsys;
    private static HoodSubsys hoodSubsys;

    public static void init(ShooterSubsys shooter, FeederSubsys feeder, HoodSubsys hood) {
        RobotCommands.shooterSubsys = shooter;
        RobotCommands.feederSubsys = feeder;
        RobotCommands.hoodSubsys = hood;
    }

    public static Command windUp() {
        return new SequentialCommandGroup(
            shooterSubsys.setVelocityRPMCommand(3000),
            hoodSubsys.positionCommand(0.4)
        );
    }

    public static Command feed() {
        return new ParallelCommandGroup(
            feederSubsys.setSpeedCommand(FeederSpeed.FEED_FAST),
            shooterSubsys.setFeedSpeedCommand(FuelFeedSpeed.FEED_FAST)
        );
    }

    public static Command stopFeed() {
        return new ParallelCommandGroup(
            feederSubsys.setSpeedCommand(FeederSpeed.OFF),
            shooterSubsys.setFeedSpeedCommand(FuelFeedSpeed.OFF)
        );
    }

    public static Command shoot() {
        return new SequentialCommandGroup(
            windUp(),
            new WaitCommand(1.0),
            feed(),
            new WaitCommand(1.0),
            stopFeed(),
            shooterSubsys.stopCommand()
        );
    }
}
