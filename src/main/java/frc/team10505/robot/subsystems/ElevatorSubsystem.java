package frc.team10505.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismObject2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    // Variables//
    public static final int kElevatorFollowerId = 52;
    public static final int kElevatorLeaderId = 53;
    public static final int kElevatorLeaderCurrentLimit = 40;
    // PID
    public static double KP = 0.0;
    public static double KI = 0.0;
    public static double KD = 0.0;
    // Suff in voltage feed forward
    public static double KS = 0.0;
    public static double KG = 0.0;
    public static double KV = 0.0;
    public static double KA = 0.0;

    private final TalonFX elevatorFxLeader;// = new TalonFX(kElevatorLeaderId, "Kingcan");
    private final TalonFX elevatorFxFollower;// = new TalonFX(kElevatorFollowerId, "Kingcan");
    private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(2.0);
    // Encoders both real and simulated.
    private DutyCycleEncoder elevatorEncoderValue = new DutyCycleEncoder(1);
    private double totalEffort;
    private double simEncoder = 0.0;
    // controls
    private final PIDController elevatorController = new PIDController(KP, KI, KD);
    private final ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(KG, KV, KA);
    // Operator interface
    private final SendableChooser<Double> elevatorHeight = new SendableChooser<>();
    private double Height = 6.0;
    // Sim vars
    public final Mechanism2d elevMech = new Mechanism2d(6.0, 12.0);
    private final MechanismRoot2d elevRoot = elevMech.getRoot("elevRoot", 3.0, 0.0);
    public final MechanismLigament2d elevatorViz = elevRoot.
    append(new MechanismLigament2d("ElevatorLigament", 0, 90, 70.0, new Color8Bit(Color.kBlanchedAlmond)));
    private final ElevatorSim elevatorSim = new ElevatorSim(DCMotor.getKrakenX60(2), 12, 10, Units.inchesToMeters(1.5), 0, 30,
            true, Height);

    public boolean usePID = true;

    public ElevatorSubsystem() {
        if (Utils.isSimulation()) {
            elevatorFxLeader = new TalonFX(kElevatorLeaderId);
            elevatorFxFollower = new TalonFX(kElevatorFollowerId);
        } else {
            elevatorFxLeader = new TalonFX(kElevatorLeaderId, "kingCan");
            elevatorFxFollower = new TalonFX(kElevatorFollowerId, "Kingcan");
        }

         TalonFXConfiguration cfg = new TalonFXConfiguration();

        FeedbackConfigs fdb = cfg.Feedback;
        fdb.SensorToMechanismRatio = 12;// TODO check gearstack irl

        MotionMagicConfigs motionMagic = cfg.MotionMagic;
        motionMagic.withMotionMagicCruiseVelocity(RotationsPerSecond.of(10))//1000
                .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(20))//2400
                .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(10));//500000

        Slot0Configs slot0 = cfg.Slot0;
        slot0.kS = 0.0; // Add 0.25 V output to overcome static friction
        slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        slot0.kG = 0.26;// .528
        slot0.kP = 0.0;//15// A position error of 0.2 rotations results in 12 V output
        slot0.kI = 0; // No output for integrated error
        slot0.kD = 0.0;//1.9// A velocity error of 1 rps results in 0.5 V output

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = elevatorFxLeader.getConfigurator().apply(cfg);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println("Could not configure device. Error: " + status.toString());
        }

        SmartDashboard.putData("elevatorViz", elevMech);

    }

    // refrence commands
    public Command setElevatorHight(double newHeight) {
            return runOnce(() -> {
                Height = newHeight;
            });
        }
    

    public Command setMotor(double voltage) {
        return runEnd(() -> {
            usePID = false;
            elevatorFxLeader.setVoltage(voltage);
        },
                () -> {
                    elevatorFxLeader.setVoltage(0);
                    usePID = true;
                });

    }

    public double calculatevoltage() {
        if (Utils.isSimulation()) {
            return elevatorFeedforward.calculate(0) + elevatorController.calculate(Height);
        } else {
            return elevatorFeedforward.calculate(elevatorEncoderValue.get(), 0)
                    + elevatorController.calculate(elevatorEncoderValue.get(), Height);
        }
    }

    @Override
    public void periodic() {
        if (Utils.isSimulation()) {
            simEncoder = elevatorViz.getLength();
            var change = (Height) - (simEncoder);
            elevatorFxLeader.setControl(motionMagicVoltage.withPosition(change).withSlot(0));
            elevatorSim.setInput(elevatorFxLeader.getMotorVoltage().getValueAsDouble());
            elevatorSim.update(0.01);
            elevatorViz.setLength(elevatorSim.getPositionMeters());
            SmartDashboard.putNumber("height", Height);
            SmartDashboard.putNumber("heightEncoder", elevatorEncoderValue.get());
            SmartDashboard.putData("elevatorSim", elevMech);
            SmartDashboard.putNumber("Elevator Encoder", simEncoder);
            SmartDashboard.putNumber("Elevator set voltage", elevatorFxLeader.getMotorVoltage().getValueAsDouble());
            SmartDashboard.putNumber("Elevator follower set voltage",
                    elevatorFxFollower.getMotorVoltage().getValueAsDouble());
            SmartDashboard.putNumber("Elevator Height", Height);
            SmartDashboard.putNumber("sim elev position", elevatorSim.getPositionMeters());
            SmartDashboard.putNumber("sim elev motor position", elevatorFxLeader.getPosition().getValueAsDouble());
        }else{

        }

       


    }

}
