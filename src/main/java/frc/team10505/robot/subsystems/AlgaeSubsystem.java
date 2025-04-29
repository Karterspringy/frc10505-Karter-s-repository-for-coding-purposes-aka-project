package frc.team10505.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeSubsystem extends SubsystemBase{
    //varis
    public static final int kAlgaePivotMotorId = 41;
    public static final int kAlgaeIntakeMotorId = 42;
    public static final int kAlgaeMotorLimit = 110;
    public static final int kAlgaeIntakeLimit = 100000;
    // PID
    public static double KP; //= 0.0;
    public static double KI; //= 0.0;
    public static double KD; //= 0.0;
    // Suff in voltage feed forward
    public static double KS; //= 0.0;
    public static double KG; //= 0.0;
    public static double KV; //= 0.0;
    public static double KA; //= 0.0;
    //Motors
    private final SparkMax algaePivot;
    private final SparkMax algaeIntake;
    private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0.0);
    //Encoders real and sim
    private DutyCycleEncoder pivotEncoderValue = new DutyCycleEncoder(2);
    private double simEncoder;
    //controls
    private final PIDController pivotController = new PIDController(KP, KI, KD);
    private final ElevatorFeedforward pivotFeedforward = new ElevatorFeedforward(KG, KV, KA);
     // Operator interface
    private final SendableChooser<Double> pivotAngle = new SendableChooser<>();
    private double Angle = 0.0;
    // Sim vars
    public final Mechanism2d pivMech = new Mechanism2d(6.0, 12.0);
    private final MechanismRoot2d pivRoot = pivMech.getRoot("elevRoot", 3.0, 0.0);
    public final MechanismLigament2d pivViz = pivRoot.
    append(new MechanismLigament2d("ElevatorLigament", 5.0, 0, 10.0, new Color8Bit(Color.kRed)));
    private final ElevatorSim elevatorSim = new ElevatorSim(DCMotor.getKrakenX60(2), 12, 10, Units.inchesToMeters(1.5), 0, 30,
            true, Angle);

    public boolean usePID = true;

    public AlgaeSubsystem() {
        if (Utils.isSimulation()) {
            algaePivot = new SparkMax(kAlgaePivotMotorId, MotorType.kBrushless);
            algaeIntake = new SparkMax(kAlgaeIntakeMotorId, MotorType.kBrushless);
        } else {
            algaePivot = new SparkMax(kAlgaePivotMotorId, MotorType.kBrushless);
            algaeIntake = new SparkMax(kAlgaeIntakeMotorId, MotorType.kBrushless);
        }
    
        SmartDashboard.putData("Pivot Thingy", pivMech);
} 

public Command setAngle(double newAngle) {
    return runOnce(() -> {
        Angle =newAngle;
    });
}








}