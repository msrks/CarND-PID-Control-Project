## Describe the effect each of the P, I, D components had in your implementation.

>Student describes the effect of the P, I, D component of the PID algorithm in their implementation. Is it what you expected?

#### P (Proportional gain)

= decrease CTE(Cross Track Error)

* low P Gain: big overshoot, low frequency oscillation, take long time to converge
* high P Gain: small overshoot, high frequency oscillation
* too high P Gain: spin out of control

#### D (Differential gain)

= decrease overshoot

* low D Gain: underdamped (still oscillate)
* high D Gain: overdamped (not oscillate, but take long time to correct offset)

#### I (Integral gain)

= decreases lane offset

* low I Gain: take long time to converge
* high I Gain: unstable (normal controller fluctuations will be exaggerated)

>Visual aids are encouraged, i.e. record of a small video of the car in the simulator and describe what each component is set to.

https://youtu.be/fr0zbR_6ecQ

## Describe how the final hyperparameters were chosen.

>Student discusses how they chose the final hyperparameters (P, I, D coefficients). This could be have been done through manual tuning, twiddle, SGD, or something else, or a combination!

At first, I manually choose PD-control parameters to drive well by try-and-error. I got the parameters as following.

- P: 0.35
- D: 8.0

For fine tuning, I used twiddle, with the folloing initial value.

- P: 0.35
- I: 0.001
- D: 8.0

```
./pid twiddle
```

Finally I got the parameters as following.

- P: 0.353815
- I: 0.00117217
- D: 9.17881
