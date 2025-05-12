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
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
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
    public static final int kAlgaePivotMotorId = 41;
    public static final int kAlgaeIntakeMotorId = 42;
    public static final int kAlgaeMotorLimit = 110;
    public static final int kAlgaeIntakeLimit = 100000;
    private double pivotEncoderScale = 360;
    private double pivotOffset = 0;
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

    public AlgaeSubsystem() {
        if (Utils.isSimulation()) {
            algaeIntake = new SparkMax(kAlgaeIntakeMotorId, MotorType.kBrushless);
            pivotController = new PIDController(1.0, 0.0, 0.0);
            pivotFeedforward = new ArmFeedforward(0.0, 0.1632552, 0.2, 0.2);
           
        } else {
            algaeIntake = new SparkMax(kAlgaeIntakeMotorId, MotorType.kBrushless);
            pivotController = new PIDController(1.0, 0.0, 0.0);
            pivotFeedforward = new ArmFeedforward(0.0, 0.1632552, 0.2, 0.2);

        }
      
        pivotMotorConfig.idleMode(IdleMode.kBrake);
        pivotMotorConfig.smartCurrentLimit(kAlgaeMotorLimit, kAlgaeMotorLimit);
        pivotMotorConfig.absoluteEncoder.positionConversionFactor(pivotEncoderScale);
        pivotMotorConfig.absoluteEncoder.zeroOffset(pivotOffset);
        algaePivot.configure(pivotMotorConfig, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        SmartDashboard.putData("pivot Viz", pivMech);

     
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
            return pivotFeedforward.calculate(pivotFeedforward.getKa(), Angle)
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
            SmartDashboard.putNumber("angle", Angle);
            SmartDashboard.putNumber("pivot Encoder", simEncoder);
            SmartDashboard.putNumber("pivot angle", Angle);
            SmartDashboard.putNumber("sim pivot angle", pivotSim.getAngleRads());
        } else {
            SmartDashboard.putNumber("pivotEncoder", pivotEncoderValue.get());
        }


} 

}