package com.team6962.lib.math;

import org.apache.commons.math3.analysis.MultivariateFunction;
import org.apache.commons.math3.analysis.UnivariateFunction;

public class ConstantFunction implements UnivariateFunction, MultivariateFunction {
  private final double value;

  public ConstantFunction(double value) {
    this.value = value;
  }

  @Override
  public double value(double x) {
    return value;
  }

  @Override
  public double value(double[] point) {
    return value;
  }
}
