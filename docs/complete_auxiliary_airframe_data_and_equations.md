# Supplementary Material S1. Complete Auxiliary-Airframe Aerodynamic Data and Interpolation Rules

This document accompanies *Whirl-Flutter Analysis of a Flexible Tiltrotor Model under Prescribed Steady Maneuvers*. It contains the complete XV-15/GTRS auxiliary-airframe lookup tables and interpolation equations used by the aircraft-level trim model. The data are separated from the main paper so that the discussion can remain focused on the maneuvering whirl-flutter results. Reference numbering follows the main manuscript; the source model is Ref. 38.

This appendix lists the XV-15/GTRS data used in the trim model of Sec. 3.2. Baseline rigid-wing loads are not added to the flexible MTR wing loads, although the tabulated wing lift coefficient is retained in the aileron–yaw coupling. The vertical-tail model keeps the empirical rotor-sidewash multiplier but omits explicit rotor-to-fin induced velocities and wake-boundary tests. The source data are from Ref. 38.

## S1.1. Interpolation, Clamping, and Mach Convention

Every one-dimensional table uses piecewise-linear interpolation with endpoint clamping,

\[
\mathcal I(x)=
\begin{cases}
y_1, & x\le x_1,\\[1mm]
y_i+
\dfrac{x-x_i}{x_{i+1}-x_i}
(y_{i+1}-y_i),
&x_i<x<x_{i+1},\\[2mm]
y_N, &x\ge x_N.
\end{cases}
\tag{S1.1}
\]

In the equations below, \(\mathcal I_\xi(Y)\) denotes interpolation of the tabulated ordinate \(Y\) at the current value of \(\xi\). Two-dimensional tables use bilinear interpolation and are clamped independently in both input directions. The auxiliary-airframe Mach number and the interpolation fraction between the low-Mach and \(M=0.4\) branches are

\[
M=\frac{V}{340.3},
\qquad
\mu_M=\operatorname{clip}
\left(
\frac{M-0.2}{0.4-0.2},0,1
\right).
\tag{S1.2}
\]

Here, \(V\) is the auxiliary-airframe reference speed in \(\mathrm{m\,s^{-1}}\).

## S1.2. Tabulated Fuselage Model

The fuselage model uses dimensional area-like force ordinates and volume-like moment ordinates rather than conventional nondimensional coefficients.

**Table S1.1. Angle-of-attack-dependent dimensional fuselage ordinates.**

| \(\alpha_F\), deg | \(A_{L,\alpha}\), ft\(^2\) | \(A_{D,\alpha}\), ft\(^2\) | \(A_{m,\alpha}\), ft\(^3\) |
|---:|---:|---:|---:|
| -20 | -10.87 | 15.39 | -380.0 |
| -16 | -7.25 | 10.78 | -370.0 |
| -12 | -3.63 | 6.17 | -295.0 |
| -8 | -0.01 | 3.00 | -219.0 |
| -4 | 3.61 | 1.80 | -142.5 |
| 0 | 7.23 | 1.56 | -66.5 |
| 4 | 10.85 | 1.80 | 9.5 |
| 8 | 14.47 | 2.30 | 85.5 |
| 12 | 18.09 | 3.67 | 123.5 |
| 16 | 21.71 | 5.78 | 142.5 |
| 20 | 25.33 | 7.89 | 133.0 |

**Table S1.2. Sideslip-dependent fuselage force, roll-moment, and yaw-moment ordinates.**

| \(\lvert\beta_F\rvert\), deg | \(A_{L,\beta}\), ft\(^2\) | \(A_{D,\beta}\), ft\(^2\) | \(A_{Y,\beta}\), ft\(^2\) | \(A_{\ell,\beta}\), ft\(^3\) | \(A_{n,\beta}\), ft\(^3\) |
|---:|---:|---:|---:|---:|---:|
| 0 | 7.23 | 1.56 | 0.0 | 0.0 | 0.0 |
| 10 | 5.00 | 5.00 | -14.5 | -75.0 | -202.0 |
| 20 | 0.00 | 10.00 | -29.0 | -150.0 | -404.0 |

**Table S1.3. Sideslip-dependent fuselage pitching-moment ordinate.**

| \(\lvert\beta_F\rvert\), deg | \(A_{m,\beta}\), ft\(^3\) |
|---:|---:|
| 0 | -66.5 |
| 2 | -66.5 |
| 4 | -54.8 |
| 6 | -34.0 |
| 8 | -14.0 |
| 10 | 0.0 |
| 20 | 70.0 |

The implemented additive constants are

\[
\begin{aligned}
L_{BFO}&=-7.23\ \mathrm{ft^2},&
D_{BFO}&=-1.56\ \mathrm{ft^2},&
M_{BFO}&=66.5\ \mathrm{ft^3},\\
L_{LANG}&=0,&
D_{LANG}&=-0.5\ \mathrm{ft^2},&
D_{POD}&=1.15\ \mathrm{ft^2}.
\end{aligned}
\tag{S1.3}
\]

The dimensional ordinates are scaled at run time by

\[
s_2=0.09290304\lambda^2
=0.0033547264\ \mathrm{m^2/ft^2},
\tag{S1.4}
\]

\[
s_3=0.02831685\lambda^3
=0.000194305776473161\ \mathrm{m^3/ft^3}.
\tag{S1.5}
\]

The rounded cubic conversion \(0.02831685\ \mathrm{m^3/ft^3}\) is retained because it is the value used by the implementation. With \(\bar\alpha_F\) and \(\bar\beta_F\) denoting angles clamped to \(\pm20^\circ\), the active fuselage combinations are

\[
A_L=s_2\left[
\mathcal I_\alpha(A_{L,\alpha})\cos^2\bar\beta_F
+\mathcal I_{\lvert\beta\rvert}(A_{L,\beta})
+L_{BFO}+L_{LANG}
\right],
\tag{S1.6}
\]

\[
A_D=s_2\left[
\mathcal I_\alpha(A_{D,\alpha})\cos^2\bar\beta_F
+\mathcal I_{\lvert\beta\rvert}(A_{D,\beta})
+D_{BFO}+D_{LANG}
\right],
\tag{S1.7}
\]

\[
\begin{aligned}
A_Y&=s_2\operatorname{sgn}(\bar\beta_F)
\mathcal I_{\lvert\beta\rvert}(A_{Y,\beta}),\\
A_m&=s_3\left[
\mathcal I_\alpha(A_{m,\alpha})\cos^2\bar\beta_F
+\mathcal I_{\lvert\beta\rvert}(A_{m,\beta})
+M_{BFO}
\right],\\
A_\ell&=s_3\operatorname{sgn}(\bar\beta_F)
\mathcal I_{\lvert\beta\rvert}(A_{\ell,\beta}),\\
A_n&=s_3\operatorname{sgn}(\bar\beta_F)
\mathcal I_{\lvert\beta\rvert}(A_{n,\beta}).
\end{aligned}
\tag{S1.8}
\]

The corresponding dimensional loads are obtained by multiplying these ordinates by dynamic pressure. The two landing-gear-related drag terms are kept distinct: \(D_{LANG}s_2=-0.0016773632\ \mathrm{m^2}\) is included in the fuselage formula, whereas \(D_{POD}s_2=0.00385793536\ \mathrm{m^2}\) is applied as the separate pod-drag contribution listed in Table 3.

## S1.3. Baseline Wing Lookup Retained in the Aileron–Yaw Coupling

The GTRS baseline rigid-wing forces are not added to the flexible MTR wing loads. Nevertheless, the baseline \(C_{L,0}\) lookup remains active in the aileron-induced yawing-moment increment.

**Table S1.4. Low-Mach, zero-flap baseline wing lookup.**

| \(\alpha\), deg | -20 | -16 | -12 | -8 | -4 | 0 | 4 | 8 | 11 | 12 | 13 | 16 | 17 | 20 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| \(C_L\) | -1.15 | -0.95 | -0.67 | -0.33 | -0.04 | 0.38 | 0.72 | 1.04 | 1.28 | 1.37 | 1.42 | 1.57 | 1.57 | 1.38 |
| \(C_D\) | 0.150 | 0.089 | 0.042 | 0.025 | 0.0170 | 0.0204 | 0.0418 | 0.072 | 0.1065 | 0.118 | 0.131 | 0.171 | 0.189 | 0.247 |

**Table S1.5. \(M=0.4\), zero-flap baseline wing lookup.**

| \(\alpha\), deg | -20 | -16 | -13 | -12 | -11 | -8 | -4 | 0 | 4 | 8 | 11 | 12 | 13 | 16 | 20 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| \(C_L\) | -0.84 | -0.94 | -0.85 | -0.772 | -0.67 | -0.37 | -0.025 | 0.38 | 0.75 | 1.12 | 1.41 | 1.46 | 1.45 | 1.32 | 1.20 |
| \(C_D\) | 0.175 | 0.089 | 0.058 | 0.042 | 0.0335 | 0.025 | 0.0170 | 0.0204 | 0.0418 | 0.072 | 0.114 | 0.128 | 0.1445 | 0.194 | 0.305 |

The active lift interpolation and aileron–yaw coupling are

\[
C_{L,0}
=(1-\mu_M)\mathcal I_\alpha(C_{L,\mathrm{low}})
+\mu_M\mathcal I_\alpha(C_{L,M=0.4}),
\tag{S1.9}
\]

\[
\Delta C_L=0.00316\,\delta_{a,\mathrm{eff}},
\qquad
\Delta C_n
=0.00143\,\delta_{a,\mathrm{eff}}
-0.415\,C_{L,0}\Delta C_L,
\tag{S1.10}
\]

where angular deflections in Eq. (S1.10) are in degrees. The two \(C_D\) rows are documented because they are loaded with the configuration, but the rigid-wing drag branch is inactive in the present auxiliary model and does not contribute to the reported trim loads.

## S1.4. Horizontal-Tail and Elevator Data

Let \(\mathbf V_O^B=[U_O,V_O,W_O]^T\) denote the translational velocity of the aircraft reference point relative to the stationary air, resolved in the body axes, and let \(\boldsymbol\omega_B\equiv{}^B\boldsymbol\omega_{B/I}\). The horizontal-tail local velocity, angle of attack, and dynamic pressure are

\[
\begin{aligned}
\mathbf V_H&=\mathbf V_O^B+\boldsymbol\omega_B\times\mathbf r_H,
\qquad \mathbf V_H=[U_H,V_H,W_H]^T,\\
\alpha_H&=\operatorname{atan2}(W_H,U_H)
-\frac{\pi}{180}\epsilon_{\deg}(\alpha_{F,\deg}),
\qquad
\alpha_{H,\deg}=\frac{180}{\pi}\alpha_H,
\end{aligned}
\tag{S1.11}
\]

\[
q_H=\tfrac12\rho K_H(U_H^2+W_H^2),
\qquad K_H=0.8.
\tag{S1.12}
\]

**Table S1.6. Wing-to-horizontal-tail downwash.**

| \(\alpha_F\), deg | -8 | -4 | 0 | 4 | 8 | 12 | 16 | 20 | 24 | 28 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| \(\epsilon\), deg | 0.06 | 1.32 | 2.58 | 3.84 | 5.10 | 5.90 | 6.30 | 6.00 | 4.00 | 0.00 |

**Table S1.7. Low-Mach elevator lift increment.**

| \(\delta_e\), deg | -20 | -15 | -10 | 0 | 10 | 15 | 20 |
|---|---:|---:|---:|---:|---:|---:|---:|
| \(\Delta C_{L,e}\) | -0.75118 | -0.612375 | -0.40825 | 0 | 0.40825 | 0.612375 | 0.75118 |

**Table S1.8. Elevator/rudder Mach corrections and high-Mach horizontal-tail lift slope.**

*(a) Elevator and rudder Mach-correction factors.*

| \(M\) | 0.0 | 0.2 | 0.4 | 0.5 | 0.6 | 0.7 |
|---|---:|---:|---:|---:|---:|---:|
| \(X_{Ke}=X_{Kr}\) | 1.0 | 1.0 | 0.965 | 0.95 | 0.93 | 0.90 |

*(b) High-Mach horizontal-tail lift-curve slope.*

| \(M\) | 0.2 | 0.4 |
|---|---:|---:|
| \(a_H\), rad\(^{-1}\) | 4.068 | 4.44 |

**Table S1.9. Horizontal-tail drag lookup.**

| \(\alpha_{D,H}\), deg | 0 | 4 | 8 | 12 |
|---|---:|---:|---:|---:|
| \(C_{D,H}^{\mathrm{tab}}\) | 0.00875 | 0.015 | 0.035 | 0.068 |

The horizontal-tail lift and drag branches are

\[
C_{L,H}=
\begin{cases}
0.071\,\alpha_{H,\deg}
+\mathcal I_{\delta_e}(\Delta C_{L,e}),
&M\le0.2,\\[1mm]
a_H(M)\left[
\alpha_H+X_{Ke}(M)\tau_e\delta_e\pi/180
\right],
&M>0.2,
\end{cases}
\tag{S1.13}
\]

\[
\alpha_{D,H}=\lvert\alpha_{H,\deg}+\tau_e\delta_e\rvert,
\qquad
C_{D,H}=\mathcal I_{\alpha_{D,H}}(C_{D,H}^{\mathrm{tab}}),
\qquad \tau_e=0.518.
\tag{S1.14}
\]

## S1.5. Vertical-Tail, Sidewash, Rudder, and Drag Data

For each vertical tail,

\[
\begin{aligned}
\mathbf V_V&=\mathbf V_O^B+\boldsymbol\omega_B\times\mathbf r_V,
\qquad \mathbf V_V=[U_V,V_V,W_V]^T,\\
\beta_{\mathrm{geom}}
&=\operatorname{atan2}
\left(V_V,\sqrt{U_V^2+W_V^2}\right),
\end{aligned}
\tag{S1.15}
\]

\[
\beta_V=
\beta_{\mathrm{geom}}
-\frac{b_{W,\mathrm{ref}}}{2U_{\mathrm{eff}}}
(-0.1p+0r),
\qquad
U_{\mathrm{eff}}
=s_U\max(\lvert U_V\rvert,10.668),
\qquad
s_U=
\begin{cases}
-1,&U_V<0,\\
+1,&U_V\ge0.
\end{cases}
\tag{S1.16}
\]

The degree-valued quantity used in the tabulated low-Mach and drag relations is \(\beta_{V,\deg}=(180/\pi)\beta_V\).

The baseline vertical-tail multiplier is

\[
K_V
=K_{\beta R}(V_{\mathrm{kt}},\lvert\beta_F\rvert)
K_\sigma(\alpha_F,\lvert\beta_F\rvert).
\tag{S1.17}
\]

Here, \(V_{\mathrm{kt}}\) denotes the reference speed \(V\) from Eq. (S1.2), expressed in knots.

**Table S1.10. Low-Mach rudder side-force increment.**

| \(\delta_r\), deg | -20 | -15 | 0 | 15 | 20 |
|---|---:|---:|---:|---:|---:|
| \(\Delta C_{Y,r}\) | -0.3984 | -0.305 | 0 | 0.305 | 0.3984 |

**Table S1.11. Rotor-sidewash multiplier \(K_{\beta R}\).**

| \(V\), kt \(\backslash\) \(\lvert\beta_F\rvert\), deg | 0 | 5 | 10 | 15 | 20 | 25 | 30 |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 0 | 1.0 | 1.0 | 1.0 | 1.0 | 1.0 | 1.0 | 1.0 |
| 20 | 1.0 | 1.0 | 1.0 | 1.0 | 1.0 | 1.0 | 1.0 |
| 40 | -0.5 | 0.25 | 0.80 | 1.25 | 1.50 | 1.0 | 1.0 |
| 60 | 0.2 | 0.40 | 0.80 | 1.10 | 1.40 | 1.0 | 1.0 |
| 80 | 0.5 | 0.60 | 0.80 | 1.00 | 1.20 | 1.0 | 1.0 |
| 100 | 0.75 | 0.80 | 0.80 | 1.00 | 1.00 | 1.0 | 1.0 |
| 120 | 1.0 | 1.0 | 1.0 | 1.0 | 1.0 | 1.0 | 1.0 |
| 350 | 1.0 | 1.0 | 1.0 | 1.0 | 1.0 | 1.0 | 1.0 |

The literal value \(-0.5\) at 40 kt and zero sideslip is retained without positivity clipping.

**Table S1.12. Fuselage-sidewash multiplier \(K_\sigma=1-\partial\sigma/\partial\beta_F\).**

| \(\alpha_F\), deg \(\backslash\) \(\lvert\beta_F\rvert\), deg | 0 | 4 | 8 | 12 | 16 | 20 |
|---:|---:|---:|---:|---:|---:|---:|
| -10 | 1.000 | 1.000 | 1.000 | 1.000 | 1.000 | 1.000 |
| -3 | 1.090 | 1.090 | 1.100 | 1.180 | 1.150 | 1.040 |
| 0 | 1.000 | 1.000 | 1.000 | 1.000 | 1.000 | 1.000 |
| 7 | 0.834 | 0.834 | 0.865 | 0.866 | 0.842 | 0.924 |
| 13 | 0.659 | 0.659 | 0.676 | 0.622 | 0.642 | 0.680 |
| 28 | 1.000 | 1.000 | 1.000 | 1.000 | 1.000 | 1.000 |

**Table S1.13. Vertical-tail drag lookup.**

| \(\beta_D\), deg | 0 | 4 | 8 | 12 | 16 |
|---|---:|---:|---:|---:|---:|
| \(C_{D,V,\mathrm{low}}\) | 0.0071 | 0.014 | 0.024 | 0.080 | 0.160 |
| \(C_{D,V,M=0.4}\) | 0.0071 | 0.014 | 0.044 | 0.150 | 0.330 |

With \(\tau_r=0.27\), the vertical-tail side-force coefficient is

\[
C_{Y,V}=
\begin{cases}
K_V(0.053125\,\beta_{V,\deg})
+\mathcal I_{\delta_r}(\Delta C_{Y,r}),
&M\le0.2,\\[1mm]
K_V(3.02522\,\beta_V)
+3.02522X_{Kr}\tau_r\delta_r\pi/180,
&M>0.2.
\end{cases}
\tag{S1.18}
\]

The multiplier \(K_V\) applies to the baseline side-force term but not to the rudder increment. For drag,

\[
\beta_D=
\lvert\beta_{V,\deg}+X_{Kr}(M)\tau_r\delta_r\rvert,
\tag{S1.19}
\]

\[
C_{D,V}=K_V\left[
(1-\mu_M)\mathcal I_{\beta_D}(C_{D,V,\mathrm{low}})
+\mu_M\mathcal I_{\beta_D}(C_{D,V,M=0.4})
\right].
\tag{S1.20}
\]

Thus, unlike the side-force equation, the empirical multiplier applies to the complete vertical-tail drag interpolation. The additional GTRS rigid-wing rate derivatives, explicit rotor-to-fin induced-velocity vectors, and wake-boundary tests are inactive and are not included in the implementation.

## S1.6. Scope of Use

The data in this supplement are used only in the algebraic aircraft trim. They are not applied as flexible fuselage or tail loads in the nonlinear rotor–wing time-domain calculation. Baseline GTRS rigid-wing lift, drag, and pitching moment are not superposed on the flexible MTR wing loads; only the documented baseline lift lookup used in the aileron–yaw coupling remains active.
