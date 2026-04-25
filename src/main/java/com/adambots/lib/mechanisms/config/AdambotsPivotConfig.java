package com.adambots.lib.mechanisms.config;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.adambots.lib.actuators.BaseMotor;
import com.adambots.lib.visualization.MechanismVisualizer;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Subsystem;

import yams.mechanisms.config.PivotConfig;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

/**
 * Fluent configuration for {@link com.adambots.lib.mechanisms.AdambotsPivot}.
 *
 * <p>Pivot is a distinct rotational mechanism — no arm length or mass-at-distance.
 * Use for turrets, wrist pivots, or any rotational axis where you care about angle
 * but not gravitational torque from a cantilevered mass.
 */
public final class AdambotsPivotConfig {

    // Motor control
    private Double kP, kI, kD;
    private int pidSlot = 0;
    private Double trapezoidCruiseRPS;
    private Double trapezoidAccelRPSps;
    private SimpleMotorFeedforward feedforward;
    private Double statorCurrentAmps;
    private Double supplyCurrentAmps;
    private double gearRatio = 1.0;
    private Angle tolerance;
    private Boolean brakeMode = true;
    private Voltage voltageCompensation;

    // Phoenix Pro
    private boolean focEnabled = false;
    private Double expoKV, expoKA;
    private Angle continuousWrapMin, continuousWrapMax;

    // Pivot-specific
    private Angle minAngle, maxAngle;
    private Angle startingAngle;
    private Double momentOfInertia;
    private DCMotor simMotorOverride;

    // AdambotsLib
    private Subsystem subsystem;
    private String telemetryName;
    private TelemetryVerbosity telemetryVerbosity;
    private MechanismVisualizer visualizer;
    private int visualizerIndex = -1;
    private Translation3d visualizerPivot = Translation3d.kZero;

    // ---------- Motor control ----------

    public AdambotsPivotConfig withPID(double kP, double kI, double kD) {
        this.kP = kP; this.kI = kI; this.kD = kD;
        return this;
    }

    public AdambotsPivotConfig withPIDSlot(int slot) {
        this.pidSlot = slot;
        return this;
    }

    public AdambotsPivotConfig withMotionMagic(double cruiseRPS, double accelRPSps) {
        this.trapezoidCruiseRPS = cruiseRPS;
        this.trapezoidAccelRPSps = accelRPSps;
        return this;
    }

    /** Simple motor feedforward (kS, kV, kA). No gravity term — use Arm for gravity. */
    public AdambotsPivotConfig withFeedforward(double kS, double kV, double kA) {
        this.feedforward = new SimpleMotorFeedforward(kS, kV, kA);
        return this;
    }

    public AdambotsPivotConfig withCurrentLimits(double statorAmps, double supplyAmps) {
        this.statorCurrentAmps = statorAmps;
        this.supplyCurrentAmps = supplyAmps;
        return this;
    }

    public AdambotsPivotConfig withGearRatio(double reductionRatio) {
        this.gearRatio = reductionRatio;
        return this;
    }

    public AdambotsPivotConfig withTolerance(Angle tolerance) {
        this.tolerance = tolerance;
        return this;
    }

    public AdambotsPivotConfig withBrakeMode(boolean brake) {
        this.brakeMode = brake;
        return this;
    }

    public AdambotsPivotConfig withVoltageCompensation(Voltage nominal) {
        this.voltageCompensation = nominal;
        return this;
    }

    // ---------- Phoenix Pro ----------

    public AdambotsPivotConfig withFOC(boolean enable) {
        this.focEnabled = enable;
        return this;
    }

    public AdambotsPivotConfig withMotionMagicExpo(double expoKV, double expoKA) {
        this.expoKV = expoKV;
        this.expoKA = expoKA;
        return this;
    }

    /** Continuous wrapping for turret-style mechanisms. */
    public AdambotsPivotConfig withContinuousWrap(Angle min, Angle max) {
        this.continuousWrapMin = min;
        this.continuousWrapMax = max;
        return this;
    }

    // ---------- Pivot-specific ----------

    public AdambotsPivotConfig withAngleRange(Angle min, Angle max) {
        this.minAngle = min;
        this.maxAngle = max;
        return this;
    }

    public AdambotsPivotConfig withStartingAngle(Angle angle) {
        this.startingAngle = angle;
        return this;
    }

    public AdambotsPivotConfig withMomentOfInertia(double kgm2) {
        this.momentOfInertia = kgm2;
        return this;
    }

    public AdambotsPivotConfig withSimMotor(DCMotor motor) {
        this.simMotorOverride = motor;
        return this;
    }

    // ---------- AdambotsLib ----------

    public AdambotsPivotConfig withSubsystem(Subsystem subsystem) {
        this.subsystem = subsystem;
        return this;
    }

    public AdambotsPivotConfig withTelemetry(String name, TelemetryVerbosity verbosity) {
        this.telemetryName = name;
        this.telemetryVerbosity = verbosity;
        return this;
    }

    public AdambotsPivotConfig withVisualizer(MechanismVisualizer visualizer,
                                                int index,
                                                Translation3d pivotOffset) {
        this.visualizer = visualizer;
        this.visualizerIndex = index;
        this.visualizerPivot = pivotOffset;
        return this;
    }

    public AdambotsPivotConfig withVisualizer(MechanismVisualizer visualizer, int index) {
        return withVisualizer(visualizer, index, Translation3d.kZero);
    }

    // ---------- Build ----------

    public void validateAgainst(BaseMotor motor) {
        MotorConfigHelpers.warnProFeaturesOnNonPhoenix(
            motor,
            "AdambotsPivot[" + (telemetryName != null ? telemetryName : "unnamed") + "]",
            focEnabled,
            expoKV != null || expoKA != null);
    }

    public SmartMotorControllerConfig buildMotorConfig() {
        SmartMotorControllerConfig cfg = subsystem != null
            ? new SmartMotorControllerConfig(subsystem)
            : new SmartMotorControllerConfig();

        if (kP != null) {
            cfg = cfg.withClosedLoopController(kP, kI, kD, MotorConfigHelpers.slotOf(pidSlot));
        }

        if (trapezoidCruiseRPS != null && trapezoidAccelRPSps != null) {
            cfg = cfg.withTrapezoidalProfile(
                RotationsPerSecond.of(trapezoidCruiseRPS),
                RotationsPerSecondPerSecond.of(trapezoidAccelRPSps));
        }

        if (expoKV != null && expoKA != null) {
            Voltage v = voltageCompensation != null ? voltageCompensation : Volts.of(12);
            double volts = v.in(Volts);
            cfg = cfg.withExponentialProfile(
                v,
                RotationsPerSecond.of(volts / expoKV),
                RotationsPerSecondPerSecond.of(volts / expoKA));
        }

        if (feedforward != null) {
            cfg = cfg.withFeedforward(feedforward, MotorConfigHelpers.slotOf(pidSlot));
        }

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

        if (minAngle != null && maxAngle != null) {
            cfg = cfg.withSoftLimit(minAngle, maxAngle);
        }
        if (continuousWrapMin != null && continuousWrapMax != null) {
            cfg = cfg.withContinuousWrapping(continuousWrapMin, continuousWrapMax);
        }
        if (startingAngle != null) cfg = cfg.withStartingPosition(startingAngle);

        if (telemetryName != null && telemetryVerbosity != null) {
            cfg = cfg.withTelemetry(telemetryName, telemetryVerbosity);
        }
        return cfg;
    }

    public PivotConfig buildYamsConfig(SmartMotorController smartMotor) {
        PivotConfig pCfg = new PivotConfig(smartMotor);
        if (minAngle != null && maxAngle != null) pCfg = pCfg.withSoftLimits(minAngle, maxAngle);
        if (continuousWrapMin != null && continuousWrapMax != null) {
            pCfg = pCfg.withWrapping(continuousWrapMin, continuousWrapMax);
        }
        if (startingAngle != null) pCfg = pCfg.withStartingPosition(startingAngle);
        if (momentOfInertia != null) pCfg = pCfg.withMOI(KilogramSquareMeters.of(momentOfInertia));
        if (telemetryName != null && telemetryVerbosity != null) {
            pCfg = pCfg.withTelemetry(telemetryName, telemetryVerbosity);
        }
        return pCfg;
    }

    public DCMotor getSimMotorOverride() { return simMotorOverride; }
    public MechanismVisualizer getVisualizer() { return visualizer; }
    public int getVisualizerIndex() { return visualizerIndex; }
    public Translation3d getVisualizerPivot() { return visualizerPivot; }

    /** True if {@code withFOC(true)} was requested. Honored on Phoenix motors. */
    public boolean isFocEnabled() { return focEnabled; }
}
