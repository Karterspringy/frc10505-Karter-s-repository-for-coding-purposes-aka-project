package frc.team10505.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix6.Utils;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralSubsystem extends SubsystemBase {
    // vars
    public static final int kIntakeLeftMotorId = 43;
    public static final int kIntakeRightMotorId = 44;
    public static final int kIntakeLLimit = 0;
    public static final int kIntakeRLimit = 0;
    public double intakeLeftSpeed = 0;
    public double intakeRightSpeed = 0;
    // Motors
    public final SparkMax intakeLeft = new SparkMax(kIntakeLeftMotorId, MotorType.kBrushless);
    public final SparkMax intakeRight = new SparkMax(kIntakeRightMotorId, MotorType.kBrushless);
    public final SparkMaxConfig intLeft = new SparkMaxConfig();
    public final SparkMaxConfig intRight = new SparkMaxConfig();
    // Sim Left
    public final Mechanism2d IntMech = new Mechanism2d(12.0, 12.0);
    private final MechanismRoot2d lIntRoot = IntMech.getRoot("lIntake root", 2.5, 6.0);
    public final MechanismLigament2d lIntViz = lIntRoot
            .append(new MechanismLigament2d("Left int Ligament", 5.0, 0.0, 10.0, new Color8Bit(Color.kGreen)));
    public final MechanismLigament2d lIntViz2 = lIntRoot
            .append(new MechanismLigament2d("Left int Ligament2", 5.0, -180, 10.0, new Color8Bit(Color.kGreen)));
    private final FlywheelSim lIntSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 1.0, 4.0),
            DCMotor.getNEO(1));
    // sim right
    private final MechanismRoot2d rIntRoot = IntMech.getRoot("rIntake root", 7.5, 6.0);
    public final MechanismLigament2d rIntViz = rIntRoot
            .append(new MechanismLigament2d("Right int Ligament", 5.0, 0.0, 10.0, new Color8Bit(Color.kRed)));
    public final MechanismLigament2d rIntViz2 = rIntRoot
            .append(new MechanismLigament2d("Right int Ligament2", 5.0, -180, 10.0, new Color8Bit(Color.kRed)));
    private final FlywheelSim rIntSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 1.0, 4.0),
            DCMotor.getNEO(1));

    public CoralSubsystem() {
        SmartDashboard.putData("Int Viz", IntMech);
        intLeft.idleMode(IdleMode.kBrake);
        intRight.idleMode(IdleMode.kBrake);
        
    }

    public Command runInt(double speed) {
        if (Utils.isSimulation()) {
            return runEnd(() -> {
                intakeLeftSpeed = speed;
                intakeRightSpeed = -speed;
            }, () -> {
                intakeLeftSpeed = 0;
                intakeRightSpeed = 0;
            });
        } else {
            return runEnd(() -> {
                intakeLeft.set(speed);
                intakeRight.set(-speed);
            }, () -> {
                intakeLeft.set(0);
                intakeRight.set(0);
            });
        }
    }

    @Override
    public void periodic() {
        if (Utils.isSimulation()) {
            // Left
            lIntSim.update(0.01);
            lIntSim.setInput(intakeLeftSpeed);
            lIntViz.setAngle(lIntViz.getAngle() + lIntSim.getAngularVelocityRPM() * 0.05);
            lIntViz2.setAngle(lIntViz2.getAngle() + lIntSim.getAngularVelocityRPM() * 0.05);
            // Right
            rIntSim.update(0.01);
            rIntSim.setInput(intakeRightSpeed);
            rIntViz.setAngle(rIntViz.getAngle() + rIntSim.getAngularVelocityRPM() * 0.05);
            rIntViz2.setAngle(rIntViz2.getAngle() + rIntSim.getAngularVelocityRPM() * 0.05);
        } else {

        }
    }
}