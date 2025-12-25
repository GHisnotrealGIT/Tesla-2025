package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LauncherSubsystem extends SubsystemBase {

  private SparkMax m_topMotor;
  private SparkMax m_bottomMotor;

  private boolean m_launcherRunning;

  /** Creates a new LauncherSubsystem. */
  public LauncherSubsystem() {
    // create two new SPARK MAXs and configure them
    m_topMotor =
        new SparkMax(Constants.Launcher.kTopCanId, MotorType.kBrushless);

    SparkMaxConfig topConfig = new SparkMaxConfig();
    topConfig.inverted(false).smartCurrentLimit(Constants.Launcher.kCurrentLimit).idleMode(IdleMode.kBrake);

    m_bottomMotor =
        new SparkMax(Constants.Launcher.kBottomCanId, MotorType.kBrushless);
    SparkMaxConfig bottomConfig = new SparkMaxConfig();
    bottomConfig.inverted(false).smartCurrentLimit(Constants.Launcher.kCurrentLimit).idleMode(IdleMode.kBrake);

    m_topMotor.configure(topConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_launcherRunning = false;
  }

  /**
   * Turns the launcher on. Can be run once and the launcher will stay running or run continuously
   * in a {@code RunCommand}.
   */
  public void runLauncher() {
    m_launcherRunning = true;
  }

  /**
   * Turns the launcher off. Can be run once and the launcher will stay running or run continuously
   * in a {@code RunCommand}.
   */
  public void stopLauncher() {
    m_launcherRunning = false;
  }

  public boolean ampMode = false;

  public void ampMode() {
    ampMode = true;
  //  m_topMotor.set(-0.5);
   // m_bottomMotor.set(-0.6);
  }

  public void disableAmpMode() {
    ampMode = false;
  }

  @Override
  public void periodic() { // this method will be called once per scheduler run
    // set the launcher motor powers based on whether the launcher is on or not
    if (m_launcherRunning) {
      m_topMotor.set(Constants.Launcher.kTopPower);
      m_bottomMotor.set(Constants.Launcher.kBottomPower);
    } else if (ampMode) {
      m_topMotor.set(-0.1);
      m_bottomMotor.set(-0.1);
    } else {
      m_topMotor.set(0);
      m_bottomMotor.set(0);
    }
  }
}
