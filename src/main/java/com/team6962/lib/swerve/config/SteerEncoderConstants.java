package com.team6962.lib.swerve.config;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

/** The constants that define the steer encoder behavior. */
public class SteerEncoderConstants {
  /**
   * The CANcoder configuration for the steer encoder. Some fields in this configuration may be
   * overriden by other setting.
   *
   * <p>It's very unlikely thay you'll need to modify this, but perhaps new configurable features
   * will be added in the future. Currently, the only possibly useful value that is not set
   * elsewhere is the sensor direction.
   */
  public CANcoderConfiguration DeviceConfiguration = new CANcoderConfiguration();

  /**
   * The method to use for fusing absolute steer encoder data with more quickly updating data from
   * the steer motor's internal encoder. When using Phoenix Pro, Fused mode is recommended, and
   * otherwise Remote mode is the only option.
   */
  public DataFusionMethod DataFusion = DataFusionMethod.Fused;

  /** Constructs a SteerEncoderConstants object with default values. */
  public SteerEncoderConstants() {
    DeviceConfiguration = new CANcoderConfiguration();
  }

  /**
   * Sets the device configuration for the steer encoder, and returns this SteerEncoderConstants for
   * chaining.
   *
   * @param deviceConfiguration The CANcoder configuration
   * @return This SteerEncoderConstants object
   */
  public SteerEncoderConstants withDeviceConfiguration(CANcoderConfiguration deviceConfiguration) {
    DeviceConfiguration = deviceConfiguration;
    return this;
  }

  /**
   * Sets the data fusion method for the steer encoder, and returns this SteerEncoderConstants for
   * chaining.
   *
   * @param dataFusion The data fusion method
   * @return This SteerEncoderConstants object
   */
  public SteerEncoderConstants withDataFusion(DataFusionMethod dataFusion) {
    DataFusion = dataFusion;
    return this;
  }

  /**
   * The method to use for fusing absolute steer encoder data with more quickly updating data from
   * the steer motor's internal encoder. When using Phoenix Pro, Fused mode is recommended, and
   * otherwise Remote mode is the only option.
   */
  public static enum DataFusionMethod {
    /**
     * The motor controller will update its position and velocity whenever the encoder publishes its
     * information on CAN bus, and the motor's internal encoder will not be used.
     */
    Remote(FeedbackSensorSourceValue.RemoteCANcoder),

    /**
     * Requires Phoenix Pro; the motor controller will fuse the absolute encoder's information with
     * the data from the internal encoder, providing the best possible position and velocity for
     * accuracy and. FusedCANcoder is designed for swerve steering motors.
     */
    Fused(FeedbackSensorSourceValue.FusedCANcoder);

    /** The FeedbackSensorSourceValue that is equivalent to this DataFusionMethod. */
    public FeedbackSensorSourceValue feedbackSensorSource;

    private DataFusionMethod(FeedbackSensorSourceValue feedbackSensorSource) {
      this.feedbackSensorSource = feedbackSensorSource;
    }
  }
}
