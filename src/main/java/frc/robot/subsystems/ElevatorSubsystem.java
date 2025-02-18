package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.LaserCanSim;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;
import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import au.grapplerobotics.interfaces.LaserCanInterface.RegionOfInterest;

public class ElevatorSubsystem extends SubsystemBase {
    //Settting up
    private final DCMotor m_elevatorGearbox = DCMotor.getNEO(1);
    private final DCMotor m_elevatorGearbox2 = DCMotor.getNEO(1);
    private final SparkMax ElevatorMotor1 = new SparkMax(9, MotorType.kBrushless);
    private final SparkMax ElevatorMotor2 = new SparkMax(10, MotorType.kBrushless);
    private final RelativeEncoder m_Encoder = ElevatorMotor1.getEncoder();
    private final RelativeEncoder m_Encoder2 = ElevatorMotor2.getEncoder();

    private final SparkMaxSim motorsim1 = new SparkMaxSim(ElevatorMotor1, m_elevatorGearbox);
    private final SparkMaxSim motorsim2 = new SparkMaxSim(ElevatorMotor2, m_elevatorGearbox2);

    private final ProfiledPIDController m_elevatorPidController = new ProfiledPIDController(ElevatorConstants.Kelevatorkp, ElevatorConstants.Kelevatorki, ElevatorConstants.Kelevatorkd, new TrapezoidProfile.Constraints(ElevatorConstants.kMaxVelocity,ElevatorConstants.KMaxAccel ));

    private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(ElevatorConstants.kElevatorS,ElevatorConstants.kElevatorG,ElevatorConstants.kElevatorV,ElevatorConstants.kElevatorA);
    private ElevatorSim m_ElevatorSim = null;

    private final LaserCan m_elevatorLaserCan = new LaserCan(0);
    private final LaserCanSim m_elevatorLaserCanSim = new LaserCanSim(0);


    public ElevatorSubsystem(){

        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(40).openLoopRampRate(ElevatorConstants.kElevatorRampRate);

        if (RobotBase.isSimulation())
        {
            m_ElevatorSim = new ElevatorSim(m_elevatorGearbox,
            ElevatorConstants.kElevatorGearing,
            ElevatorConstants.kElevatorCarriageMass,
            ElevatorConstants.kElevatorDrumRadius,
            ElevatorConstants.kElevatorMinHeight,
            ElevatorConstants.kElevatorMaxHeight,
            true,
            0.0,
            0.02);
        }
    }

  
}
