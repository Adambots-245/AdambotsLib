package com.adambots.lib.utils.tuning;

import com.adambots.lib.actuators.BaseMotor;
import com.adambots.lib.actuators.TalonFXMotor;
import com.adambots.lib.actuators.config.PIDConfig;

/**
 * Result record containing PID and feedforward values from auto-tuning.
 *
 * <p>This record stores the calculated gains from PID auto-tuning and provides
 * helper methods to apply them to motors or convert to other formats.
 *
 * <p><strong>Usage Example:</strong>
 * <pre>{@code
 * // After tuning completes
 * TuningResult result = PIDAutoTuner.getLastResult("Turret");
 *
 * // Apply to motor
 * result.applyTo(turretMotor, 0);
 *
 * // Or get code string for copying to Constants
 * System.out.println(result.toCodeString());
 * // Output: kP = 0.05, kI = 0.001, kD = 0.02, kF = 0.0
 * }</pre>
 *
 * @param kP Proportional gain
 * @param kI Integral gain
 * @param kD Derivative gain
 * @param kF Feed-forward gain (simple)
 * @param kV Velocity feedforward (output per unit velocity)
 * @param kS Static feedforward (overcomes friction)
 * @param kA Acceleration feedforward
 * @param kG Gravity feedforward (for elevator/arm mechanisms)
 */
public record TuningResult(
    double kP,
    double kI,
    double kD,
    double kF,
    double kV,
    double kS,
    double kA,
    double kG
) {
    /**
     * Creates a TuningResult with only PID values (no feedforward).
     *
     * @param kP Proportional gain
     * @param kI Integral gain
     * @param kD Derivative gain
     * @return TuningResult with PID values only
     */
    public static TuningResult pid(double kP, double kI, double kD) {
        return new TuningResult(kP, kI, kD, 0.0, 0.0, 0.0, 0.0, 0.0);
    }

    /**
     * Creates a TuningResult with PID and simple feed-forward.
     *
     * @param kP Proportional gain
     * @param kI Integral gain
     * @param kD Derivative gain
     * @param kF Feed-forward gain
     * @return TuningResult with PID and kF values
     */
    public static TuningResult pidWithFF(double kP, double kI, double kD, double kF) {
        return new TuningResult(kP, kI, kD, kF, 0.0, 0.0, 0.0, 0.0);
    }

    /**
     * Creates a TuningResult with extended feedforward values (for Phoenix 6).
     *
     * @param kP Proportional gain
     * @param kI Integral gain
     * @param kD Derivative gain
     * @param kV Velocity feedforward
     * @param kS Static feedforward
     * @param kA Acceleration feedforward
     * @param kG Gravity feedforward
     * @return TuningResult with all feedforward values
     */
    public static TuningResult extended(double kP, double kI, double kD,
                                        double kV, double kS, double kA, double kG) {
        return new TuningResult(kP, kI, kD, 0.0, kV, kS, kA, kG);
    }

    /**
     * Applies the PID values to a BaseMotor.
     *
     * <p>This sets kP, kI, kD, and kF on the motor using the standard setPID method.
     *
     * @param motor The motor to configure
     * @param slot PID slot index (typically 0-2)
     */
    public void applyTo(BaseMotor motor, int slot) {
        motor.setPID(slot, kP, kI, kD, kF);
    }

    /**
     * Applies the extended PID and feedforward values to a TalonFXMotor.
     *
     * <p>This sets kP, kI, kD, kV, kS, kA, and kG using the extended Phoenix 6 API.
     *
     * @param motor The TalonFXMotor to configure
     * @param slot PID slot index (typically 0-2)
     */
    public void applyExtendedTo(TalonFXMotor motor, int slot) {
        motor.setPID(slot, kP, kI, kD, kV, kS, kA, kG);
    }

    /**
     * Converts this result to a PIDConfig record.
     *
     * @param slot PID slot index for the config
     * @return PIDConfig with the tuned values
     */
    public PIDConfig toPIDConfig(int slot) {
        return new PIDConfig(slot, kP, kI, kD, kF);
    }

    /**
     * Returns a formatted string suitable for copying into code.
     *
     * <p>Example output: {@code kP = 0.05, kI = 0.001, kD = 0.02, kF = 0.0}
     *
     * @return Code-friendly string representation
     */
    public String toCodeString() {
        StringBuilder sb = new StringBuilder();
        sb.append(String.format("kP = %.6f, kI = %.6f, kD = %.6f, kF = %.6f", kP, kI, kD, kF));

        // Only include extended values if they're non-zero
        if (kV != 0.0 || kS != 0.0 || kA != 0.0 || kG != 0.0) {
            sb.append(String.format("\nkV = %.6f, kS = %.6f, kA = %.6f, kG = %.6f", kV, kS, kA, kG));
        }

        return sb.toString();
    }

    /**
     * Returns a formatted string with Java constant declarations.
     *
     * <p>Example output:
     * <pre>
     * public static final double kP = 0.05;
     * public static final double kI = 0.001;
     * public static final double kD = 0.02;
     * public static final double kF = 0.0;
     * </pre>
     *
     * @return Java constant declarations string
     */
    public String toConstantsString() {
        StringBuilder sb = new StringBuilder();
        sb.append(String.format("public static final double kP = %.6f;%n", kP));
        sb.append(String.format("public static final double kI = %.6f;%n", kI));
        sb.append(String.format("public static final double kD = %.6f;%n", kD));
        sb.append(String.format("public static final double kF = %.6f;", kF));

        // Only include extended values if they're non-zero
        if (kV != 0.0 || kS != 0.0 || kA != 0.0 || kG != 0.0) {
            sb.append(String.format("%npublic static final double kV = %.6f;%n", kV));
            sb.append(String.format("public static final double kS = %.6f;%n", kS));
            sb.append(String.format("public static final double kA = %.6f;%n", kA));
            sb.append(String.format("public static final double kG = %.6f;", kG));
        }

        return sb.toString();
    }

    /**
     * Checks if this result has extended feedforward values.
     *
     * @return true if any extended feedforward value (kV, kS, kA, kG) is non-zero
     */
    public boolean hasExtendedFF() {
        return kV != 0.0 || kS != 0.0 || kA != 0.0 || kG != 0.0;
    }
}
