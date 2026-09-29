package com.michaelgrundvig.frc.pitwall;

import java.lang.annotation.Documented;
import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/**
 * Marks a configuration value that can be dialed in while the robot runs: a gain or a limit, the
 * kind of value you adjust while watching the robot. Facts about the hardware (sizes, gearing,
 * motors) are not tunable; they change only when the robot does.
 *
 * <p>{@link ConfigTuner} makes these values adjustable, checks each new one exactly as a committed
 * one is checked, and lists what differs from the committed configuration.
 */
@Documented
@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.RECORD_COMPONENT)
public @interface Tunable {}
