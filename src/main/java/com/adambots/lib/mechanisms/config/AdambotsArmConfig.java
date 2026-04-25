package com.adambots.lib.mechanisms.config;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.adambots.lib.actuators.BaseMotor;
import com.adambots.lib.visualization.MechanismVisualizer;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Subsystem;

import yams.mechanisms.config.ArmConfig;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

/**
 * Fluent configuration for {@link com.adambots.lib.mechanisms.AdambotsArm}.
 *
 * <p>Combines motor tuning (PID, feedforward, current limits, Phoenix Pro features)
 * with arm-specific physics (length, mass, angle range) in a single builder.
 * Translates to YAMS {@link SmartMotorControllerConfig} and {@link ArmConfig}
 * internally — callers never see YAMS types.
 *
 * <p>Phoenix Pro features ({@code withFOC}, {@code withMotionMagicExpo}) log a
 * warning if applied to a non-Phoenix motor (NEO).
 */
public final class AdambotsArmConfig {

    // ===== Motor control =====
    private Double kP, kI, kD;
    private int pidSlot = 0;
    private Double trapezoidCruiseRPS;
    private Double trapezoidAccelRPSps;
    private ArmFeedforward armFeedforward;
    private Double statorCurrentAmps;
    private Double supplyCurrentAmps;
    private double gearRatio = 1.0;
    private Angle tolerance = Degrees.of(1.0);
    private Boolean brakeMode = true;
    private Voltage voltageCompensation;

    // ===== Phoenix Pro (validated against motor type at build time) =====
    private boolean focEnabled = false;
    private Double expoKV;
    private Double expoKA;
    private Angle continuousWrapMin, continuousWrapMax;

    // ===== Arm-specific =====
    private Distance length;
    private Mass mass;
    private Angle minAngle, maxAngle;
    private Angle startingAngle;
    private Angle horizontalZero;
    private Double momentOfInertia;
    private DCMotor simMotorOverride;

    // ===== AdambotsLib-specific =====
    private Subsystem subsystem;
    private String telemetryName;
    private TelemetryVerbosity telemetryVerbosity;
    private MechanismVisualizer visualizer;
    private int visualizerIndex = -1;
    private Translation3d visualizerPivot = Translation3d.kZero;

    // ---------------- Motor control ----------------

    public AdambotsArmConfig withPID(double kP, double kI, double kD) {
        this.kP = kP; this.kI = kI; this.kD = kD;
        return this;
    }

    public AdambotsArmConfig withPIDSlot(int slot) {
        if (slot < 0 || slot > 2) {
            throw new IllegalArgumentException("PID slot must be 0-2, got " + slot);
        }
        this.pidSlot = slot;
        return this;
    }

    /** Trapezoidal motion profile (cruise velocity + acceleration). */
    public AdambotsArmConfig withMotionMagic(double cruiseRPS, double accelRPSps) {
        this.trapezoidCruiseRPS = cruiseRPS;
        this.trapezoidAccelRPSps = accelRPSps;
        return this;
    }

    /** Arm feedforward gains (kS, kG, kV, kA). Applied to current PID slot. */
    public AdambotsArmConfig withFeedforward(double kS, double kG, double kV, double kA) {
        this.armFeedforward = new ArmFeedforward(kS, kG, kV, kA);
        return this;
    }

    public AdambotsArmConfig withCurrentLimits(double statorAmps, double supplyAmps) {
        this.statorCurrentAmps = statorAmps;
        this.supplyCurrentAmps = supplyAmps;
        return this;
    }

    public AdambotsArmConfig withGearRatio(double reductionRatio) {
        this.gearRatio = reductionRatio;
        return this;
    }

    public AdambotsArmConfig withTolerance(Angle tolerance) {
        this.tolerance = tolerance;
        return this;
    }

    public AdambotsArmConfig withBrakeMode(boolean brake) {
        this.brakeMode = brake;
        return this;
    }

    public AdambotsArmConfig withVoltageCompensation(Voltage nominal) {
        this.voltageCompensation = nominal;
        return this;
    }

    // ---------------- Phoenix Pro ----------------

    /**
     * Enables FOC (Field Oriented Control) on Phoenix motors. No-op with warning
     * on non-Phoenix motors. Requires Phoenix Pro license to take effect.
     */
    public AdambotsArmConfig withFOC(boolean enable) {
        this.focEnabled = enable;
        return this;
    }

    /**
     * Configures Motion Magic exponential profile (Phoenix Pro feature).
     * Warns on non-Phoenix motors.
     */
    public AdambotsArmConfig withMotionMagicExpo(double expoKV, double expoKA) {
        this.expoKV = expoKV;
        this.expoKA = expoKA;
        return this;
    }

    /**
     * Enables continuous position wrapping between min and max.
     * For turrets, pivots, and other rotational mechanisms with absolute encoders.
     */
    public AdambotsArmConfig withContinuousWrap(Angle min, Angle max) {
        this.continuousWrapMin = min;
        this.continuousWrapMax = max;
        return this;
    }

    // ---------------- Arm-specific ----------------

    public AdambotsArmConfig withLength(Distance length) {
        this.length = length;
        return this;
    }

    public AdambotsArmConfig withMass(Mass mass) {
        this.mass = mass;
        return this;
    }

    public AdambotsArmConfig withMass(double kilograms) {
        this.mass = Kilograms.of(kilograms);
        return this;
    }

    public AdambotsArmConfig withAngleRange(Angle min, Angle max) {
        this.minAngle = min;
        this.maxAngle = max;
        return this;
    }

    public AdambotsArmConfig withStartingAngle(Angle startingAngle) {
        this.startingAngle = startingAngle;
        return this;
    }

    /** Angle at which the arm is horizontal (used for gravity feedforward). */
    public AdambotsArmConfig withHorizontalZero(Angle horizontalZero) {
        this.horizontalZero = horizontalZero;
        return this;
    }

    public AdambotsArmConfig withMomentOfInertia(double kgm2) {
        this.momentOfInertia = kgm2;
        return this;
    }

    /** Override the WPILib DCMotor model used for simulation. */
    public AdambotsArmConfig withSimMotor(DCMotor motor) {
        this.simMotorOverride = motor;
        return this;
    }

    // ---------------- AdambotsLib-specific ----------------

    public AdambotsArmConfig withSubsystem(Subsystem subsystem) {
        this.subsystem = subsystem;
        return this;
    }

    public AdambotsArmConfig withTelemetry(String name, TelemetryVerbosity verbosity) {
        this.telemetryName = name;
        this.telemetryVerbosity = verbosity;
        return this;
    }

    /**
     * Attaches this arm to a {@link MechanismVisualizer} at the given index.
     * The visualizer will auto-receive this arm's Pose3d each update cycle.
     *
     * @param visualizer the shared visualizer instance
     * @param index      component index (maps to {@code model_N.glb} in AdvantageScope)
     * @param pivotOffset pivot location relative to robot origin (meters)
     */
    public AdambotsArmConfig withVisualizer(MechanismVisualizer visualizer,
                                             int index,
                                             Translation3d pivotOffset) {
        this.visualizer = visualizer;
        this.visualizerIndex = index;
        this.visualizerPivot = pivotOffset;
        return this;
    }

    public AdambotsArmConfig withVisualizer(MechanismVisualizer visualizer, int index) {
        return withVisualizer(visualizer, index, Translation3d.kZero);
    }

    // ---------------- Internal: build YAMS configs ----------------

    /** Validates Pro feature usage against motor type and logs warnings. */
    public void validateAgainst(BaseMotor motor) {
        MotorConfigHelpers.warnProFeaturesOnNonPhoenix(
            motor,
            "AdambotsArm[" + (telemetryName != null ? telemetryName : "unnamed") + "]",
            focEnabled,
            expoKV != null || expoKA != null);
    }

    /** Builds the YAMS motor config. Package-private — used by AdambotsArm. */
    public SmartMotorControllerConfig buildMotorConfig() {
        SmartMotorControllerConfig cfg = subsystem != null
            ? new SmartMotorControllerConfig(subsystem)
            : new SmartMotorControllerConfig();

        // PID
        if (kP != null) {
            cfg = cfg.withClosedLoopController(kP, kI, kD, MotorConfigHelpers.slotOf(pidSlot));
        }

        // Trapezoidal profile (independent of PID slot)
        if (trapezoidCruiseRPS != null && trapezoidAccelRPSps != null) {
            cfg = cfg.withTrapezoidalProfile(
                RotationsPerSecond.of(trapezoidCruiseRPS),
                RotationsPerSecondPerSecond.of(trapezoidAccelRPSps));
        }

        // Exponential (Motion Magic Expo) profile — Phoenix-native; non-Phoenix wrappers ignore.
        if (expoKV != null && expoKA != null) {
            Voltage v = voltageCompensation != null ? voltageCompensation : Volts.of(12);
            double volts = v.in(Volts);
            cfg = cfg.withExponentialProfile(
                v,
                RotationsPerSecond.of(volts / expoKV),
                RotationsPerSecondPerSecond.of(volts / expoKA));
        }

        if (armFeedforward != null) {
            cfg = cfg.withFeedforward(armFeedforward, MotorConfigHelpers.slotOf(pidSlot));
        }

        // Limits + gearing
        cfg = cfg.withGearing(gearRatio);
        if (statorCurrentAmps != null) cfg = cfg.withStatorCurrentLimit(Amps.of(statorCurrentAmps));
        if (supplyCurrentAmps != null) cfg = cfg.withSupplyCurrentLimit(Amps.of(supplyCurrentAmps));
        if (tolerance != null) cfg = cfg.withClosedLoopTolerance(tolerance);
        if (brakeMode != null) {
            cfg = cfg.withIdleMode(brakeMode
                ? SmartMotorControllerConfig.MotorMode.BRAKE
                : SmartMotorControllerConfig.MotorMode.COAST);
        }
        if (voltageCompensation != null) cfg = cfg.withVoltageCompensation(voltageCompensation);

        // Soft limits from angle range
        if (minAngle != null && maxAngle != null) {
            cfg = cfg.withSoftLimit(minAngle, maxAngle);
        }

        // Continuous wrap (handled by YAMS across all motor types)
        if (continuousWrapMin != null && continuousWrapMax != null) {
            cfg = cfg.withContinuousWrapping(continuousWrapMin, continuousWrapMax);
        }

        if (startingAngle != null) cfg = cfg.withStartingPosition(startingAngle);

        if (telemetryName != null && telemetryVerbosity != null) {
            cfg = cfg.withTelemetry(telemetryName, telemetryVerbosity);
        }

        return cfg;
    }

    /** Builds the YAMS arm config. Package-private — used by AdambotsArm. */
    public ArmConfig buildYamsConfig(SmartMotorController smartMotor) {
        ArmConfig armCfg = new ArmConfig(smartMotor);
        if (length != null) armCfg = armCfg.withLength(length);
        if (mass != null) armCfg = armCfg.withMass(mass);
        if (minAngle != null && maxAngle != null) armCfg = armCfg.withSoftLimits(minAngle, maxAngle);
        if (startingAngle != null) armCfg = armCfg.withStartingPosition(startingAngle);
        if (horizontalZero != null) armCfg = armCfg.withHorizontalZero(horizontalZero);
        if (momentOfInertia != null) {
            armCfg = armCfg.withMOI(edu.wpi.first.units.Units.KilogramSquareMeters.of(momentOfInertia));
        }
        if (telemetryName != null && telemetryVerbosity != null) {
            armCfg = armCfg.withTelemetry(telemetryName, telemetryVerbosity);
        }
        return armCfg;
    }

    public DCMotor getSimMotorOverride() { return simMotorOverride; }

    public MechanismVisualizer getVisualizer() { return visualizer; }
    public int getVisualizerIndex() { return visualizerIndex; }
    public Translation3d getVisualizerPivot() { return visualizerPivot; }
    public Distance getLength() { return length; }

    /** True if {@code withFOC(true)} was requested. Honored on Phoenix motors. */
    public boolean isFocEnabled() { return focEnabled; }
}
