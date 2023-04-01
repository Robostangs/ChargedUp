package com.pathplanner.lib;

public class PathConstraints {
  public final double maxVelocity;
  public final double maxPositiveAcceleration;
  public final double maxNegativeAcceleration;

  public PathConstraints(double maxVelocity, double maxAcceleration) {
    this(maxVelocity,maxAcceleration,maxAcceleration);
  }


  /**
   * @param maxVelocity
   * @param maxPositiveAcceleration 
   * @param maxNegativeAcceleration Absolute value of maximum negative acceleration
   */
  public PathConstraints(double maxVelocity, double maxPositiveAcceleration, double maxNegativeAcceleration) {
    this.maxVelocity = maxVelocity;
    this.maxPositiveAcceleration = maxPositiveAcceleration;
    this.maxNegativeAcceleration = Math.abs(maxNegativeAcceleration);
  }
}
