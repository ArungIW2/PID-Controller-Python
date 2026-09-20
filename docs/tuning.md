# PID Tuning

Tuning is performed only after the controller, process equation, simulation
order, and response metrics are verified. A tuning is meaningful only for the
scenario used to evaluate it.

## Manual tuning

1. Start with `Ki = 0` and `Kd = 0`.
2. Increase `Kp` until the response is acceptably fast without unacceptable
   oscillation or saturation.
3. Increase `Ki` gradually to reduce steady-state error. Check overshoot,
   settling time, and time spent saturated.
4. Add a small `Kd` only when additional damping is useful. Keep
   derivative-on-measurement enabled and use filtering when the derivative is
   sensitive to fast signal changes.
5. Re-evaluate the final gains against process-parameter variations.

Manual tuning makes cause and effect visible, but it is slow and depends on the
engineer's judgment.

## Bounded parameter sweep

`parameter_sweep` evaluates a declared Cartesian grid. Candidates are ranked by
an explicit cost containing IAE, ISE, overshoot, settling time, and final error.
Weights are part of the engineering decision: changing them changes what
"best" means.

Advantages:

- deterministic and reproducible;
- transparent search bounds and scoring;
- useful for comparing trade-offs.

Limitations:

- it can miss good gains between grid points;
- cost weights can hide undesirable behavior;
- computation grows with every dimension;
- the best model result is not proof of real-process performance.

## Ziegler–Nichols

The classic closed-loop rules require an ultimate gain `Ku` and sustained
oscillation period `Pu`. The repository provides
`ziegler_nichols_closed_loop` as a transparent gain calculator when those
values are legitimately available.

The default stable first-order process without delay does not provide a useful
ultimate-oscillation experiment by itself. Therefore the project does not
pretend to identify `Ku` and `Pu` from that model or present Ziegler–Nichols as
the default tuning. This method is aggressive, can cause substantial overshoot,
and must be followed by validation and refinement.

## Robustness study

`evaluate_robustness` applies the same gains to named mathematical variations,
such as lower/higher process gain and shorter/longer time constant. This tests
model sensitivity only; it is not hardware or safety validation.

