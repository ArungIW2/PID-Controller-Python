# PID Theory and Discrete Implementation

## Feedback error

The controller compares setpoint `SP` with process variable `PV`:

\[
e(t) = SP(t) - PV(t)
\]

Positive error requests a positive controller response when gains are positive.

## Continuous reference equation

\[
u(t) = K_p e(t) + K_i \int e(t)\,dt + K_d \frac{de(t)}{dt}
\]

- Proportional action reacts to present error.
- Integral action accumulates past error and can eliminate offset.
- Derivative action reacts to the rate of change and can add damping.

## Implemented discrete form

With fixed sample time \(\Delta t\):

\[
P_k = K_p e_k
\]

\[
I_k = I_{k-1} + K_i e_k\Delta t
\]

The practical default differentiates measurement instead of error:

\[
D_k = -K_d\frac{PV_k-PV_{k-1}}{\Delta t}
\]

This avoids derivative kick caused only by a setpoint step. Error-based
derivative remains available for comparison. The first derivative sample is
zero because no previous sample exists.

## Derivative filter

The raw derivative rate can be filtered with a first-order low-pass update:

\[
d_k = d_{k-1} + \alpha(d_{raw,k} - d_{k-1})
\]

\[
\alpha = \frac{\Delta t}{\tau_f + \Delta t}
\]

`derivative_filter_tau = 0` disables filtering.

## Saturation and anti-windup

Output limits model a bounded controller command. Without protection, the
integral can continue growing while output is saturated. The implementation
offers:

- `NONE`: preserves classic windup for study;
- `CLAMP`: rejects integration that pushes farther into saturation;
- `BACK_CALCULATION`: feeds limited-minus-raw output into integral state.

Independent integral limits are also supported.

## Operating modes

Manual mode applies an explicit limited output while freezing integral state.
Returning to automatic initializes the integral contribution so the first
automatic output matches the last achievable manual output. This is the
implemented bumpless-transfer policy.

## Mathematical process

The default model is:

\[
\frac{dy}{dt}=\frac{Ku-y}{\tau}
\]

Forward Euler integration gives:

\[
y_{k+1}=y_k+\Delta t\frac{Ku_k-y_k}{\tau}
\]

This model is deliberately simple and deterministic. It is not a machine,
sensor, PLC runtime, or digital twin.

