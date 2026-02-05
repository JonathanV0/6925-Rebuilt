package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class ShooterSubsys extends SubsystemBase {

/* Shooter ID */
public final TalonFX shooterMotorTop0 = new TalonFX(0);
public final TalonFX shooterMotorTop1 = new TalonFX(0);
public final TalonFX shooterMotorTop2 = new TalonFX(0);
public final TalonFX shooterMotorBottom = new TalonFX(0);

private final double MAX_VALUE = 0.5;

public ShooterSubsys() {

  shooterMotorTop0.getConfigurator().apply(Robot.ctreConfigs.shooterFXConfig);
  shooterMotorTop1.getConfigurator().apply(Robot.ctreConfigs.shooterFXConfig);
  shooterMotorTop2.getConfigurator().apply(Robot.ctreConfigs.shooterFXConfig);
  shooterMotorBottom.getConfigurator().apply(Robot.ctreConfigs.shooterFXConfig);
  shooterMotorBottom.geInvertedValue(true);
}

public void setMotor(double top, double bottom) {
  shooterMotorTop.set(top);
  shooterMotorBottom.set(bottom);
}


public void outake() {
  shooterMotorTop0.set(-MAX_VALUE);
  shooterMotorTop1.set(-MAX_VALUE);
  shooterMotorTop2.set(-MAX_VALUE);
  shooterMotorBottom.set(-MAX_VALUE);
}

public void shooterOff() {
  shooterMotorTop.set(0);
shooterMotorTop1.set(0);
shooterMotorTop2.set(0);
shooterMotorBottom.set(0);
}

@Override
public void periodic() {}
}


