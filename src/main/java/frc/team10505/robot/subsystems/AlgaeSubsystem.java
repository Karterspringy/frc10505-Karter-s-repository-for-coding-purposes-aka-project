package frc.team10505.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeSubsystem extends SubsystemBase {
    // varis
    public static final int kAlgaePivotMotorId = 8;
    public static final int kAlgaeIntakeMotorId = 7;
    public static final int kAlgaeMotorLimit = 15;
    public static final int kAlgaeIntakeLimit = 25;
    private double pivotEncoderScale = 360;
    private double pivotOffset = 180.0;
    private double simSpeed = 0;
    // Motors
    private final SparkMax algaePivot = new SparkMax(kAlgaePivotMotorId, MotorType.kBrushless);
    private final SparkMax algaeIntake;
    private SparkMaxConfig pivotMotorConfig = new SparkMaxConfig();
    // Encoders real and sim
    private final AbsoluteEncoder pivAbsoluteEncoder = algaePivot.getAbsoluteEncoder();
    private DutyCycleEncoder pivotEncoderValue = new DutyCycleEncoder(2);
    private double simEncoder;
    // controls
    private final PIDController pivotController; //= new PIDController(0.0, 0.0, 0.0);
    private final ArmFeedforward pivotFeedforward; //= new ArmFeedforward(0.0, 0.0, 0.0);
    // Operator interface
    private final SendableChooser<Double> pivotAngle = new SendableChooser<>();
    private double Angle = 0.0;
    // Sim vars
    public final Mechanism2d pivMech = new Mechanism2d(6.0, 12.0);
    private final MechanismRoot2d pivRoot = pivMech.getRoot("pivotRoot", 2.5, 6.0);
    public final MechanismLigament2d pivViz = pivRoot
            .append(new MechanismLigament2d("PivotLigament", 5.0, 0.0, 10.0, new Color8Bit(Color.kRed)));
    private final SingleJointedArmSim pivotSim = new SingleJointedArmSim(DCMotor.getKrakenX60(2), 80, SingleJointedArmSim.estimateMOI(0.2, 3.0), Units.inchesToMeters(1.5),
            -110, 110,
            true, Angle);
   
    public boolean usePID = true;

    public final Mechanism2d alIntMech = new Mechanism2d(6.0, 12.0);
    private final MechanismRoot2d alIntRoot = alIntMech.getRoot("ALgae intake root", 2.5, 6.0);
    public final MechanismLigament2d alIntViz = alIntRoot
            .append(new MechanismLigament2d("Algae int Ligament", 5.0, 0.0, 10.0, new Color8Bit(Color.kGreen)));
    public final MechanismLigament2d alIntViz2 = alIntRoot
            .append(new MechanismLigament2d("Algae int Ligament2", 5.0, -180, 10.0, new Color8Bit(Color.kGreen)));
    private final FlywheelSim algaeIntSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 0.000000000000000000000000000000001, 4.0), DCMotor.getNEO(1));
   
    public AlgaeSubsystem() {
        if (Utils.isSimulation()) {
            algaeIntake = new SparkMax(kAlgaeIntakeMotorId, MotorType.kBrushless);
            pivotController = new PIDController(1.0, 0.0, 0.0);
            pivotFeedforward = new ArmFeedforward(0.0, 0.1632552, 0.2, 0.2);
           
        } else {
            algaeIntake = new SparkMax(kAlgaeIntakeMotorId, MotorType.kBrushless);
            pivotController = new PIDController(0.0, 0.0, 0.0);
            pivotFeedforward = new ArmFeedforward(0.0, 0.0, 0.0, 0.0);

        }
      
        pivotMotorConfig.idleMode(IdleMode.kBrake);
        pivotMotorConfig.smartCurrentLimit(kAlgaeMotorLimit, kAlgaeMotorLimit);
        pivotMotorConfig.absoluteEncoder.positionConversionFactor(pivotEncoderScale);
        pivotMotorConfig.absoluteEncoder.zeroOffset(pivotOffset);
        algaePivot.configure(pivotMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        SmartDashboard.putData("pivot Viz", pivMech);
        SmartDashboard.putData("Algae intake viz", alIntMech);

     
    }

    public double simGetEffort() {
        return pivotFeedforward.calculate(Units.degreesToRadians(pivViz.getAngle()), 0.0)
         + pivotController.calculate(simEncoder, Angle);
    }

    public Command setAngle(double newAngle) {
        return runOnce(() -> {
            Angle = newAngle;
        });
    }

    public Command runAlgInt(double speed) {
        if (Utils.isSimulation()) {
            return runEnd(() -> {
                simSpeed = speed;
            }, () -> {
                simSpeed = 0;
            });
        }else{
            return runEnd(() -> {
                algaeIntake.set(speed);
            }, () -> {
                algaeIntake.set(0);
            }
            );
        }
        }
    

    public Command setVoltage(double voltage) {
        return runOnce(() -> {
            algaePivot.setVoltage(voltage);
        });
    }

    public double getPivotEncoder() {
        return (-pivAbsoluteEncoder.getPosition() + pivotOffset);
    }

    public double calculatevoltage() {
        if (Utils.isSimulation()) {
            return pivotFeedforward.calculate(Units.degreesToRadians(pivViz.getAngle()),0) + pivotController.calculate(pivViz.getAngle(), Angle);
        } else {
            return pivotFeedforward.calculate(Units.degreesToRadians(pivotEncoderValue.get()), 0)
                    + pivotController.calculate(pivotEncoderValue.get(), Angle);
        }
    }

    @Override
    public void periodic() {
        if (Utils.isSimulation()) {
            simEncoder = pivViz.getAngle();
            pivotSim.setInput(calculatevoltage());
            pivotSim.update(0.01);
            pivViz.setAngle(Units.radiansToDegrees(pivotSim.getAngleRads()));
            algaeIntSim.update(0.001);
            algaeIntSim.setInput(simSpeed);
            alIntViz.setAngle(alIntViz.getAngle() + algaeIntSim.getAngularVelocityRPM()* 0.05);
            alIntViz2.setAngle(alIntViz2.getAngle() + algaeIntSim.getAngularVelocityRPM()* 0.05);
            SmartDashboard.putNumber("angle", Angle);
            SmartDashboard.putNumber("pivot Encoder", simEncoder);
            SmartDashboard.putNumber("pivot angle", Angle);
            SmartDashboard.putNumber("sim pivot angle", pivotSim.getAngleRads());

        } else {
            SmartDashboard.putNumber("pivotEncoder", pivotEncoderValue.get());
            SmartDashboard.putNumber("angle", Angle);
            SmartDashboard.putNumber("pivot voltage", algaePivot.getAppliedOutput());
            algaePivot.setVoltage(calculatevoltage());
        }


} 

}