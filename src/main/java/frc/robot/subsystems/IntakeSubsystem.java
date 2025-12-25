package frc.robot.subsystems;

import java.util.concurrent.DelayQueue;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.PIDGains;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

  private SparkMax m_motor;
  private RelativeEncoder m_encoder;
  private SparkClosedLoopController m_controller;

  private boolean m_positionMode;
  private double m_targetPosition;
  private double m_power;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    // create a new SPARK MAX and configure it
    m_motor = new SparkMax(Constants.Intake.kCanId, MotorType.kBrushless);

    SparkMaxConfig config = new SparkMaxConfig();
    config.inverted(false).smartCurrentLimit(Constants.Intake.kCurrentLimit).idleMode(IdleMode.kBrake);

    config.encoder.positionConversionFactor(1).velocityConversionFactor(1);

    config.closedLoop.p(Constants.Intake.kPositionGains.p).i(Constants.Intake.kPositionGains.i).d(Constants.Intake.kPositionGains.d);
    m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    m_controller = m_motor.getClosedLoopController();

    m_encoder = m_motor.getEncoder();

    m_positionMode = false;
    m_targetPosition = m_encoder.getPosition();
    m_power = 0.0;
  }

  /**
   * Set the power to spin the motor at. This only applies outside of position mode.
   *
   * @param _power The power to apply to the motor (from -1.0 to 1.0).
   */
  public void setPower(double _power) {
    m_positionMode = false;
    m_targetPosition = m_encoder.getPosition();
    m_power = _power;
  }

  /**
   * Constructs a command that drives the rollers a specific distance (number of rotations) from the
   * current position and then ends the command.
   *
   * @return The retract command
   */
  public Command retract() {
    Command newCommand =
        new Command() {
          @Override
          public void initialize() {
            m_positionMode = true;
            m_targetPosition = m_encoder.getPosition() + Constants.Intake.kRetractDistance;
          }

          @Override
          public boolean isFinished() {
            return isNearTarget();
          }
        };

    newCommand.addRequirements(this);

    return newCommand;
  }

  /**
   * Constructs a command that feeds a note into the launcher by running the intake for a set amount
   * of time. This command takes control of the launcher subsystem to make sure the wheels keep
   * spinning during the launch sequence.
   *
   * @param _launcher The instance of the launcher subsystem
   * @return The launch command
   */
  public Command feedLauncher(LauncherSubsystem _launcher) {
    Command newCommand =
        new Command() {
          private Timer m_timer;

          @Override
          public void initialize() {
            m_timer = new Timer();
            m_timer.start();
          }

          @Override
          public void execute() {
            setPower(1.0);
            _launcher.runLauncher();
          }

          @Override
          public boolean isFinished() {
            return m_timer.get() > Constants.Intake.kShootFeedTime;
          }

          @Override
          public void end(boolean interrupted) {
            _launcher.stopLauncher();
            setPower(0.0);
          }
        };

    newCommand.addRequirements(this, _launcher);

    return newCommand;
  }

  public Command feedLauncherTwo(LauncherSubsystem _launcher) {
    Command newCommand =
        new Command() {
          private Timer m_timer;
          private boolean launcher_running = false;

          @Override
          public void initialize() {
            m_timer = new Timer();
            m_timer.start();
          }

          @Override
          public void execute() {
            _launcher.runLauncher();
            setPower(.5);
          //  setPower(1.0);
          }

          @Override
          public boolean isFinished() {
            if (m_timer.get() > .16 && launcher_running == false){
              launcher_running = true;
          //    setPower(.5);
            } 
            return m_timer.get() > Constants.Intake.kShootFeedTime;
  
          }

          @Override
          public void end(boolean interrupted) {
            _launcher.stopLauncher();
            setPower(0.0);
          }
        };

    newCommand.addRequirements(this, _launcher);

    return newCommand;
  }


  @Override
  public void periodic() { // This method will be called once per scheduler run
    // if we've reached the position target, drop out of position mode
    if (m_positionMode && isNearTarget()) {
      m_positionMode = false;
      m_power = 0.0;
    }

    // update the motor power based on mode and setpoint
    if (m_positionMode) {
      m_controller.setReference(m_targetPosition, ControlType.kPosition);
    } else {
      m_motor.set(m_power);
    }

    SmartDashboard.putNumber("Intake Current", m_motor.getOutputCurrent());
  }

  /**
   * Check if the encoder is within the position tolerance.
   *
   * @return Whether the position is within the tolerance.
   */
  public boolean isNearTarget() {
    return Math.abs(m_encoder.getPosition() - m_targetPosition)
        < Constants.Intake.kPositionTolerance;
  }

  public double getOutputCurrent() {
    return m_motor.getOutputCurrent();
  }

  public static Command runIntake(IntakeSubsystem _intake) {
    Command newCommand =
        new Command() {
          private Timer m_timer;
          private boolean currentLimitMet = false;
          private boolean runningBack = false;
          private Timer m_startTimer;
          

          @Override
          public void initialize() {
            m_startTimer = new Timer();
            m_startTimer.start();
          //  m_timer = new Timer();
           // m_timer.start();
          }

          @Override
          public void execute() {
            _intake.setPower(Constants.Intake.kIntakePower);            
          //  setPower(1.0);
          }

          @Override
          public boolean isFinished() {
            if(_intake.getOutputCurrent() > Constants.Intake.kCurrentThresholdIntake && currentLimitMet == false && m_startTimer.get() > 0.3 ){
              SmartDashboard.putString("YEET", "WORKSDDD");
              currentLimitMet = true;
             // return true;
              m_timer = new Timer();
              m_timer.start();
            }
            if(!currentLimitMet){
              return false;
            }
            if(m_timer.get() < Constants.Intake.kIntakeAutonRunTime){
              SmartDashboard.putString("HEHEHE", "RUNBACKEARDS");

              if(runningBack == false){
                runningBack = true;
              //  _intake.setPower(-.5);
              }
         //     _intake.setPower(-.5);
              return false;
            }

          //  _intake.setPower(-.1);
            return true;
          }

          @Override
          public void end(boolean interrupted) {
             _intake.setPower(0.0);
          //   _intake.retract();
          }
        };

    newCommand.addRequirements(_intake);
    return newCommand;
  }
}
