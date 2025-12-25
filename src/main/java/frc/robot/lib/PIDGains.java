package frc.robot.lib;

import com.revrobotics.spark.SparkClosedLoopController;

public class PIDGains {
  public final double p;
  public final double i;
  public final double d;

  public PIDGains(double _p, double _i, double _d) {
    p = _p;
    i = _i;
    d = _d;
  }
}
