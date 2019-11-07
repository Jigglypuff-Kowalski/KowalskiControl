#include <math.h>

//Simple PID Controller that assumes regular loop intervals

class PIDController 
{
public:
	/** The p gain. */
	double pGain;
	double iGain;
	double dGain;

	double pOut;
	double iOut;
	double dOut;

	double error;
	double errorSum = 0;
	double lastError = 0;

	double dProcessVar;

	double output = 0;

	double previousValue = 0;

	double previousAverage = 0;

	double currentAverage;

	/** The average. */
	double average;

	/** The at target. */
	bool atTarget = false;

	/**
	 * Instantiates a new PID controller.
	 *
	 * @param p
	 *            the p
	 * @param i
	 *            the i
	 * @param d
	 *            the d
	 */
	PIDController(double p, double i, double d) {
		errorSum = 0; // initialize errorSum to 0
		lastError = 0; // initialize lastError to 0
		pGain = p;
		iGain = i;
		dGain = d;
	}

	/**
	 * Reset integral.
	 */
	void resetIntegral() {
		errorSum = 0.0;
	}

	/**
	 * Reset derivative.
	 */
	void resetDerivative() {
		lastError = 0.0;
	}

	/**
	 * Reset PID.
	 */
	void resetPID() {
		resetIntegral();
		resetDerivative();
		atTarget = false;
	}

	/**
	 * Change PID gains.
	 *
	 * @param kP
	 *            the k P
	 * @param kI
	 *            the k I
	 * @param kD
	 *            the k D
	 */
	void changePIDGains(double kP, double kI, double kD) {
		pGain = kP;
		iGain = kI;
		dGain = kD;
	}

	/**
	 * Calc PID.
	 *
	 * @param setPoint
	 *            the set point
	 * @param currentValue
	 *            the current value
	 * @param epsilon
	 *            the epsilon
	 * @return the double
	 */
	double calcPID(double setPoint, double currentValue, double epsilon) {
		error = setPoint - currentValue;

		if (fabs(error) <= epsilon) {
			atTarget = true;
		}
		else {
			atTarget = false;
		}

		// P
		pOut = pGain * error;

		// I
		errorSum += error;
		iOut = iGain * errorSum;

		// D
		dProcessVar = (error - lastError);
		dOut = dGain * dProcessVar;

		lastError = error;

		// PID Output
		output = pOut + iOut + dOut;

		// Scale output to be between 1 and -1
		if (output != 0.0)
			output = output / fabs(output) * (1.0 - pow(0.1, (fabs(output))));

		return output;
	}

	/**
	 * Calc PID drive.
	 *
	 * @param setPoint
	 *            the set point
	 * @param currentValue
	 *            the current value
	 * @param epsilon
	 *            the epsilon
	 * @return the double
	 */
	double calcPIDDrive(double setPoint, double currentValue, double epsilon) {
		error = setPoint - currentValue;

		if (fabs(error) <= epsilon) {
			atTarget = true;
		}
		else {
			atTarget = false;
		}

		// P
		pOut = pGain * error;

		// I
		errorSum += error;
		iOut = iGain * errorSum;

		// D
		dProcessVar = (error - lastError);
		dOut = dGain * dProcessVar;

		lastError = error;

		// PID Output
		output = pOut + iOut + dOut;

		return output;
	}

	/**
	 * Checks if is done.
	 *
	 * @return true, if is done
	 */
	bool isDone() {
		return atTarget;
	}

	/**
	 * Gets the p gain.
	 *
	 * @return the p gain
	 */
	double getPGain() {
		return pGain;
	}

	/**
	 * Gets the i gain.
	 *
	 * @return the i gain
	 */
	double getIGain() {
		return iGain;
	}

	/**
	 * Gets the d gain.
	 *
	 * @return the d gain
	 */
	double getDGain() {
		return dGain;
	}
};