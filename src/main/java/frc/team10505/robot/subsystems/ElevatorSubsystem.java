package frc.team10505.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

public class elevatorSubsystem extends SubsystemBase {
    //Variables//
    private final TalonFX elevatorFxLeader = new TalonFX(0);
    private final TalonFx elevatorFxFollower = new TalonFx(1);
    private double elevatorEncoderValue = 0.0;
}
