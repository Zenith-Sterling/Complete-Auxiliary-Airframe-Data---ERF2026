# Whirl-Flutter Analysis of a Flexible Tiltrotor Model under Prescribed Steady Maneuvers

## Abstract

Steady maneuvers change both the trim operating point and the inertial environment of a tiltrotor, but their influence on whirl-flutter stability has not been systematically quantified. A wind-tunnel-assessed Maryland Tiltrotor Rig rotor–wing model is extended to a full-span flexible system and coupled sequentially with a hybrid MTR–XV-15 trim model. Straight flight, coordinated turns, and prescribed steady pull-ups are examined over \(1\le n\le2\) using nonlinear transient damping identification. The flutter boundary, controlled by a high-frequency torsional branch, decreases from 253.9 kt in level flight to 221.2 kt in the \(n=2\) turn and 221.0 kt in the corresponding pull-up. At matched load factors, the two maneuver boundaries differ by less than 1 kt. An \(n=2\) ablation shows that the transferred pitch attitude and rotor collectives account for nearly all of the 33–34-kt reduction, whereas prescribed motion at fixed trim changes the boundary by only 0.6–1.2 kt. The results indicate that maneuver-induced changes in the rotor–wing operating point dominate the predicted stability shift.

**Keywords:** tiltrotor; whirl flutter; prescribed maneuver; coordinated turn; prescribed steady pull-up; aeroelastic stability

## 1. Introduction

Tiltrotor aircraft combine vertical takeoff and landing with efficient wing-borne cruise by rotating the proprotor–nacelle assemblies between helicopter and airplane modes. In airplane mode, the proprotors and nacelles remain mounted near the tips of a flexible wing, where aerodynamic and gyroscopic coupling can excite wing bending, wing torsion, nacelle motion, and proprotor motion. Whirl flutter occurs when the damping of one of these coupled modes becomes negative (Refs. 1–3). Its onset depends on the structural and aerodynamic model, but also on the operating condition about which the system is evaluated.

Much of the experimental basis for tiltrotor whirl-flutter analysis comes from semispan wind-tunnel models. Tests with the 1/5-scale V-22 model and the Wing and Rotor Aeroelastic Test System established early benchmarks for correlation and parameter studies (Refs. 4 and 5). TRAST, the Maryland Tiltrotor Rig, and ATTILA later added measurements of modal frequency, damping, and sensitivity to rotor, pylon, and wing properties (Refs. 6–11). Alongside these programs, comprehensive rotorcraft codes, frequency- and time-domain methods, and multibody formulations have been used to model increasingly detailed rotor–wing–pylon systems (Refs. 12–20). Rotor design, structural changes, aerodynamic fidelity, and active control have also been studied extensively (Refs. 5, 17, and 20–22).

A smaller body of work has treated full-span aircraft, conversion flight, and coupled flight-dynamics/aeroelastic problems (Refs. 23–31). Studies of conventional flexible aircraft have shown that steady maneuvers and rigid–elastic inertial coupling can alter flutter characteristics (Refs. 32 and 39). Tiltrotor whirl-flutter studies, however, have usually focused on fixed-support models or nominal straight flight. Integrated aircraft simulations have generally emphasized loads, handling qualities, control, or overall vehicle response rather than isolating the effect of a prescribed maneuver on the whirl-flutter boundary.

A maneuver changes both the trim state and the rigid-body motion imposed on the rotor–wing system. The resulting changes in attitude, rotor operating point, aircraft-root acceleration, and body angular velocity may alter modal damping. This question is also relevant to powered-lift certification, which requires aeromechanical and aeroelastic stability over the applicable configurations and operating conditions (Refs. 33–35).

The analysis builds on the authors’ MTR semispan model based on Lagrange’s equations and nonlinear transient responses (Ref. 36). The fixed-support rotor–wing model is compared with available wind-tunnel data in Sec. 3. Two mirrored flexible half-wings and two counter-rotating proprotors are then assembled into a full-span system. Scaled XV-15/GTRS aerodynamic, mass, and inertia data are used only for the aircraft-level trim. The resulting pitch attitude and rotor collectives, together with the prescribed bank angle and aircraft-root motion, are passed to the flexible model. Coordinated turns and prescribed steady pull-ups are compared at matched load factors over \(1\le n\le2\), with an additional \(n=2.5\) pair used only as a high-load check. The final ablation separates the effect of the transferred trim inputs from that of the prescribed motion itself.

## 2. Coupled Modeling Methodology

The MTR rotor–wing model developed in Ref. 36 is extended here to a full-span configuration under prescribed steady maneuvers. Each case starts with an algebraic trim on the undeformed geometry. The trimmed pitch angle and right and left rotor collectives are then passed to the nonlinear flexible model, while the bank angle, turn direction, and trajectory remain prescribed. Control-surface deflections are retained in the trim solution but are not applied as aerodynamic inputs to the flexible subsystem.

The structural and aerodynamic states remain coupled within the rotor–wing model: deformation changes the local flow, and the resulting loads act back on the structure. The aircraft trajectory, however, is not altered by the flexible loads. The aircraft motion is prescribed; a released six-degree-of-freedom trajectory is not solved.

### 2.1. Full-Span Model Architecture and Coupling Scope

The full-span wing is assembled from two geometrically mirrored flexible half-wings. Their coincident inboard nodes are assembled into a common elastic center node, and the wing center is connected to the aircraft root through a fixed geometric transformation. A three-bladed gimballed MTR proprotor is installed at each outboard nacelle. The two proprotors have the same rotational speed magnitude and rotate in opposite directions. The structural, inertial, and sectional aerodynamic properties of the baseline MTR rotor–wing model are retained.

The nacelle–rotor assembly and its host wing element share their interface generalized coordinates. If \(\mathbf q_g\) is the assembled coordinate vector and \(\mathbf q_r\) is the local coordinate vector of a rotor subsystem, the assembly relation is

\[
\mathbf q_r=\mathbf S_r\mathbf q_g,
\qquad
\mathbf Q_g\leftarrow\mathbf Q_g+\mathbf S_r^{T}\mathbf Q_r,
\qquad
\mathbf A_g\leftarrow\mathbf A_g+\mathbf S_r^{T}\mathbf A_r\mathbf S_r,
\tag{1}
\]

Here, \(\mathbf S_r\) is the signed assembly operator that maps the global coordinates \(\mathbf q_g\) to the local rotor-subsystem coordinates \(\mathbf q_r\); \(\mathbf Q_r\) is a local generalized-force vector, and \(\mathbf Q_g\) is its assembled global counterpart. The matrix \(\mathbf A\in\{\mathbf M,\mathbf C,\mathbf K\}\) denotes a local mass, damping, or stiffness contribution. This shared-coordinate assembly retains the inertial, elastic, gyroscopic, and aerodynamic coupling between each proprotor and the corresponding flexible wing.

Two global reference frames are used in the aircraft-level kinematic description: the inertial frame \(I\) and the body-fixed frame \(B\). The aircraft-root point \(O\) coincides with the reference center of gravity, and its axes are aligned with \(B\); the aircraft root is therefore a point rather than a third independently rotating frame. The homogeneous transformation \({}^{I}\mathbf H_B\) maps coordinates from \(B\) to \(I\). With \(W_0\) denoting the full-span wing center, the pose of a blade or wing section \(A\) is written as

\[
{}^{I}\mathbf H_A
=
{}^{I}\mathbf H_B(t)\,
{}^{B}\mathbf H_{W_0}
\prod_{k=1}^{m}\mathbf H_k(\mathbf q,\boldsymbol{\Psi}_r,t),
\tag{2}
\]

where \({}^{B}\mathbf H_{W_0}\) is the fixed pose of the wing-center frame \(W_0\) in \(B\), mapping coordinates from \(W_0\) into the body frame. The prescribed aircraft motion enters only through \({}^{I}\mathbf H_B(t)\), whereas elastic deformation, gimbal motion, and the counter-rotating rotor azimuths enter through the remaining transformations.

Figure 1 summarizes the analysis sequence.

![Coupling architecture of the prescribed-motion tiltrotor model](figures/Manuscript/Figure1_Coupling_Architecture.svg)

*Figure 1. Coupling architecture of the full-span prescribed-motion tiltrotor model. Aircraft-level trim supplies the pitch angle and rotor collectives, while bank angle and trajectory kinematics are prescribed by the selected maneuver and the auxiliary control-surface solution remains confined to trim. Aerodynamic and structural states are mutually coupled in the flexible transient calculation, but the resulting flexible loads do not update the prescribed aircraft-root motion.*

The scaled auxiliary airframe aerodynamics are used to close the full-aircraft trim balance. No feedback path returns the flexible rotor–wing response to a released aircraft trajectory.

### 2.2. MTR-Based Rotor–Wing Aeroelastic Formulation

The structural and aerodynamic formulations follow the authors' preceding MTR study (Ref. 36); only the elements needed to define the present extension are summarized here. The blades and wings are represented by moderate-deformation beam elements with 15 degrees of freedom. In the maneuver calculations, the rotor azimuths are prescribed at 1050 rpm and the rotor-spin degrees of freedom are removed from the solved system coordinates. After removal of the rotor-spin coordinate, the corresponding constraint reaction is assembled as an equal-and-opposite moment on the nacelle–wing coordinates. Thus, rotor-speed perturbations are not solved, but the structural load path associated with the shaft constraint is preserved. The remaining equations follow Lagrange's equations of the second kind:

\[
\frac{\mathrm d}{\mathrm dt}
\left(\frac{\partial T}{\partial \dot q_i}\right)
-\frac{\partial T}{\partial q_i}
+\frac{\partial U}{\partial q_i}
+Q_{D,i}
=Q_{A,i},
\tag{3}
\]

where \(T\) and \(U\) are the kinetic and strain energies, and \(Q_{D,i}\) and \(Q_{A,i}\) are the dissipative and aerodynamic generalized forces. The prescribed body transformation is included in the position and velocity fields used to construct \(T\). Its explicit time dependence generates the maneuver-induced translational-inertia, Coriolis, tangential, centrifugal, and gyroscopic terms when Lagrange's equations are evaluated.

A central feature of the beam model is the second-order consistency of the section kinematics. Define the transverse slopes as

\[
s_v=v',\qquad s_w=w',
\tag{4}
\]

where \((\,\cdot\,)'\equiv \mathrm d(\,\cdot\,)/\mathrm ds\) denotes differentiation with respect to the beam span coordinate \(s\).

The section-rotation tensor is constructed by retaining all terms through second order in \(s_v\) and \(s_w\), while the total elastic and built-in twist \(\Phi\) is represented trigonometrically:

\[
\mathbf R^{(2)}
=
\mathcal T_2
\left\{
\mathbf R_x(\Phi)\mathbf R_y(-s_w)\mathbf R_z(s_v)
\right\}.
\tag{5}
\]

Here, \(\mathcal T_2\{\cdot\}\) denotes consistent truncation after the second-order slope terms. Since an exact rotation tensor satisfies \(\mathbf R\mathbf R^T=\mathbf I\), the retained approximation obeys

\[
\mathbf R^{(2)}\mathbf R^{(2)T}
=
\mathbf I+
\mathcal O\!\left(\left\|[s_v\;s_w]\right\|^3\right).
\tag{6}
\]

This second-order rotational consistency is required to preserve orthogonality to the retained order and to avoid spurious terms caused by an inconsistent first-order orientation approximation. The interpolation functions, sectional mass and stiffness integrals, structural-damping formulation, and gimbal formulation follow Ref. 36. The structural-damping data adopted in the calculations follow the Glenn L. Martin MTR model used for the extended-speed comparison in Sec. 3.1.

For a section point \(P\), the air-relative velocity is formed from the ambient and induced-flow velocities minus the instantaneous point velocity of Eq. (11) and is then resolved in the deformed section frame. Denoting its chordwise and normal components by \(U_t\) and \(U_p\), respectively,

\[
U=\sqrt{U_t^2+U_p^2},
\qquad
\alpha=\operatorname{atan2}(U_p,U_t),
\tag{7}
\]

and the sectional lift, drag, and pitching moment are evaluated from the corresponding C81-format aerodynamic-coefficient tables,

\[
L'=\tfrac12\rho U^2cC_l,
\qquad
D'=\tfrac12\rho U^2cC_d,
\qquad
M'=\tfrac12\rho U^2c^2C_m.
\tag{8}
\]

A uniform induced-velocity model is employed, with the mean inflow determined from the Glauert momentum relation as in Ref. 36. The mean inflow is updated during the flexible calculation. The distributed aerodynamic loads are mapped to the structural coordinates through virtual work,

\[
Q_{A,i}^{(e)}
=
\int_{0}^{l_e}
\left(
\mathbf f_A\cdot\frac{\partial\mathbf r}{\partial q_i}
+
\mathbf m_A\cdot\frac{\partial\boldsymbol\vartheta}{\partial q_i}
\right)\mathrm ds.
\tag{9}
\]

Here, \(l_e\) is the element length, \(\mathbf f_A\) and \(\mathbf m_A\) are the sectional aerodynamic force and moment resultants per unit span, \(\mathbf r\) is the section position, and \(\boldsymbol\vartheta\) denotes the section-orientation parameters used in the virtual-work mapping.

Structural deformation changes the local velocity and orientation seen by the aerodynamic model; the updated forces and moments then act on the nonlinear structure.

### 2.3. Prescribed Steady-Maneuver Kinematics

At \(t=0\), the inertial frame \(I\) is oriented forward–right–down, whereas \(B\) remains aligned with the aircraft forward–right–down axes. The attitude is described by the \(Z\)-\(Y\)-\(X\) yaw–pitch–roll angles \((\psi,\theta,\phi)\), with \(\mathbf C_{IB}=\mathbf R_z(\psi)\mathbf R_y(\theta)\mathbf R_x(\phi)\) mapping body-frame components into \(I\) and \(\mathbf C_{BI}=\mathbf C_{IB}^{T}\) denoting its inverse. The vector of rotor azimuths is denoted by \(\boldsymbol{\Psi}_r\), and the body angular velocity is written consistently as

\[
\boldsymbol\omega_B\equiv{}^{B}\boldsymbol\omega_{B/I}.
\tag{10}
\]

For a structural or rotor point \(P\), let \(\boldsymbol\rho_P^B(\mathbf q,\boldsymbol{\Psi}_r)\) be its position relative to the aircraft-root point \(O\), resolved in \(B\). Its inertial kinematics are

\[
\begin{aligned}
\mathbf r_P^I
&=\mathbf r_O^I+\mathbf C_{IB}\boldsymbol\rho_P^B,\\
\mathbf v_P^I
&=\mathbf v_O^I+\mathbf C_{IB}
\left(\dot{\boldsymbol\rho}_P^B+
\boldsymbol\omega_B\times\boldsymbol\rho_P^B\right),\\
\mathbf a_P^I
&=\mathbf a_O^I+\mathbf C_{IB}
\left[
\ddot{\boldsymbol\rho}_P^B
+2\boldsymbol\omega_B\times\dot{\boldsymbol\rho}_P^B
+\dot{\boldsymbol\omega}_B\times\boldsymbol\rho_P^B
+\boldsymbol\omega_B\times
(\boldsymbol\omega_B\times\boldsymbol\rho_P^B)
\right].
\end{aligned}
\tag{11}
\]

Equation (11) introduces the translational, Coriolis, tangential, and centrifugal acceleration terms without requiring a separate root-frame notation.

For straight-and-level flight at speed \(V\),

\[
\mathbf v_O^I=[V\;0\;0]^T,
\qquad
\mathbf a_O^I=\mathbf0,
\qquad
(\phi,\theta,\psi)=(0,\theta_{\mathrm{tr}},0),
\qquad
\boldsymbol\omega_B=\mathbf0.
\tag{12}
\]

For a coordinated level turn, the load factor \(n\), bank angle \(\phi\), turn radius \(R_T\), and turn rate \(\Omega_T\) satisfy

\[
n=\frac{1}{\cos\lvert\phi\rvert},
\qquad
R_T=\frac{V^2}{g\sqrt{n^2-1}},
\qquad
\Omega_T=\frac{g\sqrt{n^2-1}}{V}.
\tag{13}
\]

Let \(s_T=\pm1\) denote the turn direction, let \(\chi=\Omega_Tt\), and define the signed bank angle by \(\phi=s_T\cos^{-1}(1/n)\). At nonzero pitch and bank, zero sideslip at the center of gravity requires the constant heading–track offset

\[
\Delta\psi
=
\tan^{-1}\!\left(\tan\phi\sin\theta_{\mathrm{tr}}\right).
\]

Accordingly, the prescribed attitude is \((\phi,\theta,\psi)=(\phi,\theta_{\mathrm{tr}},\Delta\psi+s_T\chi)\), and the aircraft-root trajectory is

\[
\mathbf r_O^I=
\begin{bmatrix}
R_T\sin\chi\\
s_TR_T(1-\cos\chi)\\
0
\end{bmatrix},
\qquad
\mathbf a_O^I=
\begin{bmatrix}
-V\Omega_T\sin\chi\\
s_TV\Omega_T\cos\chi\\
0
\end{bmatrix}.
\tag{14}
\]

The roll and pitch angles remain constant, while the yaw rate follows the trajectory. The body angular velocity is obtained from the Euler-angle rates,

\[
\boldsymbol\omega_B
=
s_T\Omega_T
\begin{bmatrix}
-\sin\theta_{\mathrm{tr}}\\
\sin\phi\cos\theta_{\mathrm{tr}}\\
\cos\phi\cos\theta_{\mathrm{tr}}
\end{bmatrix},
\qquad
\dot{\boldsymbol\omega}_B=\mathbf0.
\tag{15}
\]

For a prescribed steady pull-up, \(n\) is defined at the initial horizontal-tangent state. The prescribed vertical-circle radius and pitch rate are

\[
n=1+\frac{V^2}{gR_P},
\qquad
R_P=\frac{V^2}{(n-1)g},
\qquad
\Omega_P=\frac{(n-1)g}{V}.
\tag{16}
\]

For a pull-up beginning at \(t=0\),

\[
\begin{aligned}
\mathbf r_O^I
&=
\begin{bmatrix}
R_P\sin(\Omega_Pt)\\
0\\
-R_P[1-\cos(\Omega_Pt)]
\end{bmatrix},\\
\mathbf a_O^I
&=
\begin{bmatrix}
-V\Omega_P\sin(\Omega_Pt)\\
0\\
-V\Omega_P\cos(\Omega_Pt)
\end{bmatrix},\\
(\phi,\theta,\psi)
&=(0,\theta_{\mathrm{tr}}+\Omega_Pt,0),
\qquad
\boldsymbol\omega_B=[0\;\Omega_P\;0]^T.
\end{aligned}
\tag{17}
\]

The pull-up is therefore a prescribed constant-curvature continuation of the horizontal-tangent trim condition rather than a sequence of globally retrimmed points. The aircraft-root acceleration in Eq. (17) supplies the maneuver-induced inertial excitation. Structural gravity is omitted from the flexible time-domain equations and enters only the aircraft-level trim balance in Sec. 2.4; consequently, no time-varying body-frame gravity vector is applied during the prescribed pull-up response. The three maneuver definitions are illustrated with the numerical cases in Sec. 3.3.

### 2.4. Maneuver-Specific Algebraic Trim

Each speed and maneuver condition is first trimmed on the undeformed reference geometry. The trim unknown vector is

\[
\mathbf x_{\mathrm{tr}}
=
\begin{bmatrix}
\theta_{\mathrm{tr}}&
\theta_{0,c}&
\Delta\theta_0&
\delta_e&
\delta_a&
\delta_r
\end{bmatrix}^{T},
\tag{18}
\]

where \(\theta_{0,c}\) and \(\Delta\theta_0\) are the common and differential collectives. The individual collective pitches are

\[
\theta_{0,R}=\theta_{0,c}+\tfrac12\Delta\theta_0,
\qquad
\theta_{0,L}=\theta_{0,c}-\tfrac12\Delta\theta_0.
\tag{19}
\]

The maneuver type, speed, load factor, and turn direction are prescribed. The aerodynamic forces and moments of both proprotors and wings are averaged over one rotor revolution. Scaled fuselage and auxiliary tail/control-surface aerodynamic data from the XV-15 Generic Tilt-Rotor Simulation (GTRS) are then added to close the aircraft-level balance. The detailed component definitions and numerical parameters are given in Sec. 3.2.

With \(+\mathbf e_{z_I}\) directed downward, the force and moment residuals at the reference center of gravity are

\[
\begin{aligned}
\mathbf r_F^B
&=
\mathbf C_{BI}
\left(
\overline{\mathbf F}_A^I
+mg\mathbf e_{z_I}
-m\mathbf a_O^I
\right),\\
\mathbf r_M^B
&=
\overline{\mathbf M}_{A,O}^B
-\mathbf I_B\dot{\boldsymbol\omega}_B
-\boldsymbol\omega_B\times
(\mathbf I_B\boldsymbol\omega_B).
\end{aligned}
\tag{20}
\]

The overbars in Eq. (20) denote rotor-revolution averages, \(m\) is the total trim mass, and \(\mathbf I_B\) is the aircraft inertia tensor about \(O\), resolved in \(B\). For the maneuver trims, Eq. (20) is evaluated at the reference operating point \(t=0\). Thus, \(\mathbf a_O^I\), \(\mathbf C_{BI}\), and the attitude take their initial values from Eqs. (14) and (17), including the coordinated-turn heading–track offset defined above.

The six algebraic trim equations are

\[
\mathbf r_{\mathrm{tr}}
=
\begin{bmatrix}
\mathbf r_F^B\\
\mathbf r_M^B
\end{bmatrix}
=\mathbf0.
\tag{21}
\]

The equal and opposite mean spin angular momenta of the two nominally identical counter-rotating proprotors cancel in the aircraft-level trim balance. Rotor gyroscopic and elastic inertial effects remain fully retained in the flexible subsystem equations. The trimmed pitch angle and right and left rotor collectives are passed to the flexible calculation as fixed operating-point inputs together with the prescribed bank angle and maneuver kinematics. The elevator, aileron, and rudder deflections are retained only as trim variables because the corresponding auxiliary airframe loads are not applied in the flexible stage. The mean inflow is reinitialized from the transferred rotor inputs and updated in the flexible calculation rather than transferred as a stored trim state.

### 2.5. Coupled Governing Equations and Stability Evaluation

The assembled nonlinear flexible equations are expressed in residual form as

\[
\boldsymbol{\mathcal R}
(\mathbf q,\dot{\mathbf q},\ddot{\mathbf q},\boldsymbol\lambda,t)
=
\mathbf Q_I
+\mathbf Q_E
+\mathbf C_s\dot{\mathbf q}
-\mathbf Q_A
=\mathbf0,
\tag{22}
\]

where \(\mathbf Q_I\) contains the structural and rotor inertia together with all terms generated by the prescribed body motion, \(\mathbf Q_E\) is the nonlinear elastic restoring force, \(\mathbf C_s\dot{\mathbf q}\) is the structural damping force, \(\mathbf Q_A\) is the blade and wing aerodynamic generalized force, and \(\boldsymbol\lambda\) contains the mean-inflow variables. The equations are integrated using the same implicit Newmark-based nonlinear solution strategy as in Ref. 36, with the aerodynamic loads and inflow reevaluated during the coupled response calculation.

The transient-stability procedure follows Ref. 36. The beamwise, chordwise, and torsional responses at the nacelle–wing attachment are used to track the corresponding physical branches. In each channel, a branch-specific frequency band is used to track the relevant spectral peak, and the decay or growth constant is obtained by a least-squares fit to the logarithm of the resulting amplitude envelope. The two nearby torsional branches are separated by iterative complex demodulation before their envelopes are fitted. The numerical bands, windows, and record-acceptance rules are reported in Sec. 3.3. This processing is used only to estimate branch damping and is not treated as a separate contribution of the present paper. For a response branch \(m\),

\[
y_m(t)
\simeq
A_m e^{-\alpha_m t}
\cos(2\pi f_m t+\varphi_m),
\qquad
\zeta_m^{\mathrm{id}}
\simeq
\frac{\alpha_m}{2\pi f_m}.
\tag{23}
\]

Here, \(f_m\) is the identified damped response frequency, and the second relation in Eq. (23) is the light-damping approximation. A positive \(\zeta_m^{\mathrm{id}}\) denotes decay, a negative value denotes growth, and zero denotes the estimated stability boundary. If adjacent airspeeds \(V_1<V_2\) bracket a zero crossing, with \(\zeta_i=\zeta(V_i)\), the onset speed is evaluated by

\[
V_f
=
V_1-\zeta_1
\frac{V_2-V_1}{\zeta_2-\zeta_1}.
\tag{24}
\]

The flutter-onset speed is taken as the first positive-to-negative damping crossing of the controlling physical branch, provided that a lower-speed stable interval has been established. Cases whose lowest retained point is already unstable, or whose nonmonotonic damping sequence precludes a unique onset assignment, are classified separately and are not assigned a unique onset speed by Eq. (24). Numerical discretization and airspeed grids are reported with the computational setup in Sec. 3 rather than as part of the theoretical formulation.

## 3. Model Validation and Numerical Setup

The fixed-support MTR model is first compared with two straight-blade wind-tunnel datasets. The Carderock data cover the low-speed range to 100 kt, while the Glenn L. Martin tests extend the comparison to 200 kt and include a torsional-response channel. The semispan model is then mirrored for the maneuver calculations, and scaled XV-15/GTRS data are added to obtain the aircraft-level trim.

### 3.1. Two-Stage Wind-Tunnel Assessment of the Straight-Blade MTR Model

The validation subset considered here is the straight-blade, gimbal-free, freewheeling MTR configuration with the NACA 0018 wing section installed. Here, *gimbal free* denotes an unlocked gimballed hub rather than the absence of a gimbal. The same three-bladed, 4.75-ft-diameter rotor–wing–pylon test article was tested in two facilities at a nominal Froude-scaled rotor speed of 1050 rpm. Consistently, the validation simulations release the rotor-spin degree of freedom. The two datasets are presented as successive assessment stages rather than merged into a single comparison, because the first establishes the low-speed beamwise and chordwise trends and the second broadens both the airspeed range and the observed modal content. These tests exclude released aircraft rigid-body motion and therefore assess the rotor–wing aeroelastic core before the prescribed-maneuver extension is introduced. In contrast, the maneuver calculations in Secs. 3.2 and 3.3 prescribe the two rotor speeds at 1050 rpm, remove the rotor-spin degrees of freedom from the solved system coordinates, and retain the associated constraint reaction torques in the nacelle–wing load path. The wind-tunnel comparisons therefore do not directly validate this prescribed-speed formulation.

**Stage 1: Carderock validation up to 100 kt.** The first test campaign was conducted in the Naval Surface Warfare Center Carderock Division 8-by-10-ft Large Subsonic Wind Tunnel, with the wing fairings installed. The straight-blade baseline configuration was tested to 100 kt, and beamwise and chordwise modal damping were measured. The authors' previous study compared the corresponding predictions with these data (Ref. 36). As shown first in Fig. 2, the calculated beam damping follows the measured increase with airspeed, while the chord damping remains weakly dependent on speed and within the overall experimental scatter. Those predictions are also consistent with the published analysis used in the earlier comparison. No torsional damping measurement suitable for validation was available from this campaign, so this first-stage assessment is limited to the beam and chord branches.

![Stage-1 Carderock MTR damping comparison up to 100 kt](figures/Manuscript/Figure2_Carderock_MTR_Validation.svg)

*Figure 2. Stage-1 low-speed validation of the straight-blade, gimbal-free, freewheeling MTR with wing fairings installed in the Carderock wind tunnel: comparison of the calculation reported in Ref. 36 with the published analysis and measurements of Ref. 9 for beamwise damping (upper panel) and chordwise damping (lower panel), with available data extending to 100 kt.*

**Stage 2: Glenn L. Martin extended-speed comparison.** The second campaign used the same MTR and straight blades in the University of Maryland 7.75-by-11-ft Glenn L. Martin Wind Tunnel and provided high-speed beamwise, chordwise, and torsional-response measurements (Ref. 37). The present calculation spans 30–200 kt. For the straight-blade, gimbal-free baseline, the available beam and chord damping measurements extend to approximately 193 kt. Because the torsion mode was more difficult to excite experimentally, usable torsion damping measurements were obtained only over approximately 80–122 kt.

For Fig. 3, each calculated decay constant is converted to a damping ratio using the modal frequency identified from the same transient response, consistently with Eq. (23). The calculated modal frequency is therefore allowed to vary with airspeed and response branch rather than being replaced by a fixed reference value. The experimental damping ratios are digitized directly from Ref. 37.

![Stage-2 Glenn L. Martin MTR damping comparison up to 200 kt](figures/Manuscript/Figure3_Glenn_Martin_MTR_Comparison.svg)

*Figure 3. Stage-2 extended-speed comparison for the straight-blade, gimbal-free, freewheeling MTR initialized at the nominal Froude-scaled rotor speed of 1050 rpm: present calculations from 30 to 200 kt and Glenn L. Martin measurements digitized from Fig. 13 of Ref. 37 for (a) beamwise, (b) chordwise, and (c) torsional damping. Beamwise and chordwise measurements extend to approximately 193 kt, whereas torsional measurements are available only from approximately 80 to 122 kt.*

Figure 3 shows that the calculation reproduces the measured increase in beam damping through the low- and intermediate-speed range, although it approaches or exceeds the upper portion of the experimental scatter at the highest measured speeds. The calculated chord damping captures the broad reduction toward high speed but is systematically higher than most of the measured values. The calculated torsional branch remains positively damped over the available measurement interval and reproduces the measured order of magnitude, although it does not reproduce all local variations in the experimental data. Given the limited excitation range and substantial trial-to-trial scatter reported in Ref. 37, the torsion comparison is treated as a qualitative, limited-range check. The structural-damping data used for Fig. 3 and the subsequent maneuver calculations are adopted consistently from the Glenn L. Martin MTR model.

Figures 2 and 3 reproduce the main beam and chord damping trends and confirm the sign and approximate level of the measured torsional damping over the available interval. Because torsion data end near 122 kt, the high-speed torsional response and the flutter boundaries reported below remain numerical predictions. The wind-tunnel comparisons also apply only to the fixed-support rotor–wing model, not to the auxiliary airframe, trim model, or prescribed maneuver kinematics.

### 3.2. Hybrid MTR–XV-15 Full-Aircraft Trim Configuration

The maneuver model is a hybrid MTR–XV-15 configuration rather than a dynamically scaled replica of a single aircraft. The flexible rotor–wing subsystem retains the MTR geometry, structural distributions, C81 airfoil data, hub model, and rotor operating speed of the fixed-support baseline assessed in Sec. 3.1. Two mirrored MTR half-wings and two counter-rotating MTR proprotors form the full-span flexible system. The fuselage, tail, control-surface increments, aircraft mass properties, and their reference locations are derived from selected airplane-mode terms of the XV-15 Generic Tilt-Rotor Simulation (GTRS) model (Ref. 38). These auxiliary components participate in the algebraic trim but are not added as flexible components in the time-domain aeroelastic model.

The principal retained MTR properties are listed in Table 1. The spanwise stiffness, mass, elastic-axis, twist, and inertia distributions are unchanged from the baseline model in Ref. 36 and are therefore not re-identified here.

**Table 1. Retained properties of the full-span MTR rotor–wing subsystem.**

| Property | Value used in the model |
|---|---:|
| Full-span wing construction | Two mirrored flexible half-wings |
| Half-span / full span | 0.927 m / 1.854 m |
| Wing chord / geometric planform area | 0.392 m / 0.7268 m\(^2\) |
| Wing section aerodynamics | NACA 0018 C81 data |
| Wing discretization | 7 beam elements per half-wing; 5 Gauss points per element |
| Number of proprotors | 2, with opposite rotation directions |
| Blades per proprotor | 3 straight blades |
| Rotor radius / blade chord | 0.724 m / 0.080 m |
| Blade section aerodynamics | VR-7 C81 data |
| Blade discretization | 7 beam elements per blade; 5 Gauss points per element |
| Hub model | Three-bladed gimballed hub; gimbal unlocked |
| Inflow model | Uniform mean inflow obtained from the Glauert momentum relation |
| Rotor-speed treatment in maneuver calculations | Prescribed 1050 rpm on both sides, with opposite rotation senses |
| Angular speed / nominal tip speed | \(109.96\ \mathrm{rad\,s^{-1}}\) / \(79.61\ \mathrm{m\,s^{-1}}\) |
| Structural-damping data | Adopted from the Glenn L. Martin MTR model |
| Modeled rotor-axis direction | Body \(+x\); zero mast cant |

The geometric scale is defined by the MTR rotor radius and the 12.5-ft full-scale XV-15 rotor radius. A single mass scale maps the 6818-lb XV-15 wing–rotor–nacelle group to the adopted 81.3599-kg MTR rotor–wing–nacelle group. Thus,

\[
\lambda
=\frac{0.724}{12.5(0.3048)}
=0.190026,
\qquad
k_m
=\frac{81.3599}{6818(0.45359237)}
=0.0263080,
\qquad
k_I=k_m\lambda^2=0.000949981.
\tag{25}
\]

Equation (25) is applied only to the selected XV-15/GTRS auxiliary-airframe quantities; the MTR flexible wing and proprotors are retained directly and are not obtained by scaling an XV-15 structure. Auxiliary-component lengths and areas are scaled by \(\lambda\) and \(\lambda^2\), dimensionless tail and control-effectiveness coefficients retain their tabulated values, and the dimensional fuselage force and moment ordinates are scaled by \(\lambda^2\) and \(\lambda^3\), respectively. The mass and inertia substitutions use the hybrid engineering scales in Eq. (25). Thus,

\[
\mathbf r_{a,m}=\lambda\mathbf r_{a,f},
\qquad
S_m=\lambda^2 S_f,
\qquad
A_{F,m}=\lambda^2 A_{F,f},
\qquad
A_{M,m}=\lambda^3 A_{M,f},
\qquad
m_m=k_m m_f,
\qquad
\mathbf I_m=k_m\lambda^2\mathbf I_f.
\tag{26}
\]

Here, subscripts \(f\) and \(m\) denote the full-scale GTRS and scaled-model quantities, respectively, while \(a\) identifies an auxiliary-component reference location. The quantities \(A_F\) and \(A_M\) denote the area-like and volume-like dimensional ordinates used by the GTRS fuselage force and moment tables, respectively.

All locations are expressed in the body axes defined in Sec. 2, with \(+x\) forward, \(+y\) toward the right wing, and \(+z\) downward. The adopted center of gravity corresponds to \((SL_{CG},BL_{CG},WL_{CG})=(311.900582,0,79.127831)\ \mathrm{in}\) in the GTRS coordinate system. For a GTRS location given by station line \(SL\), butt line \(BL\), and water line \(WL\), in inches, the scaled coordinates relative to this point are

\[
\begin{aligned}
x&=-0.0254\lambda(SL-SL_{CG}),\\
y&= \phantom{-}0.0254\lambda(BL-BL_{CG}),\\
z&=-0.0254\lambda(WL-WL_{CG}).
\end{aligned}
\tag{27}
\]

The mass, inertia, and principal assembly locations obtained from this procedure are summarized in Table 2. The aircraft root and the trim center of gravity coincide with the body-axis origin. The adopted inertia tensor about this point is

\[
\mathbf I_B=
\begin{bmatrix}
65.62369 & 0 & -1.38537\\
0 & 26.20878 & 0\\
-1.38537 & 0 & 86.51302
\end{bmatrix}
\ \mathrm{kg\,m^2}.
\tag{28}
\]

**Table 2. Scaling, mass, inertia, and principal-location data for the hybrid trim configuration.**

| Quantity | Value |
|---|---:|
| Geometric scale, \(\lambda\) | 0.19003 |
| Area scale, \(\lambda^2\) | 0.036110 |
| Cubic geometric scale, \(\lambda^3\) | 0.0068618 |
| Mass scale, \(k_m\) | 0.026308 |
| Inertia scale, \(k_I\) | 0.00094998 |
| Adopted MTR wing–nacelle–rotor group mass | 81.36 kg |
| Scaled equivalent auxiliary-airframe group mass | 73.77 kg |
| Total trim mass | 155.13 kg |
| Weight at \(g=9.80665\ \mathrm{m\,s^{-2}}\) | 1521.3 N |
| Aircraft root / trim center of gravity | \([0,\ 0,\ 0]\) m |
| Full-span wing-center attachment | \([0.09750,\ 0,\ -0.08071]\) m |
| Right / left shaft-pivot locations | \([0.05744,\ \pm0.93155,\ -0.10074]\) m |
| Right / left hub locations | \([0.32775,\ \pm0.93626,\ -0.10074]\) m |
| Nacelle angle | \(90^\circ\) |

The auxiliary model contains only the airframe terms needed to close the six trim equations. Baseline rotor and wing loads still come from the flexible C81 models, so the GTRS rigid-wing lift, drag, pitching moment, and rate derivatives are not added a second time. The only retained rigid-wing lookup is the baseline \(C_L\) used in the aileron-induced yawing-moment increment. Table 3 lists the active auxiliary components and their main parameters. The complete lookup arrays and interpolation equations are available at https://github.com/YOUR_GITHUB_USERNAME/MTR-XV15-maneuver-flutter-data/releases/tag/v1.0.

**Table 3. Active components of the scaled XV-15/GTRS auxiliary aerodynamic model.**

| Component | Principal geometry | Retained model terms |
|---|---|---|
| Tabulated fuselage | Reference point \([0.09123,0,-0.02352]\) m; \(\alpha,\beta\in[-20^\circ,20^\circ]\) | Tabulated force and moment ordinates; internal landing-gear term |
| External pod/gear drag | Area \(0.003858\ \mathrm{m^2}\); point \([-0.05840,0,0.34621]\) m | Axial drag |
| Aileron increments | \(S_{\mathrm{ref}}=0.6072\ \mathrm{m^2}\), \(b_{\mathrm{ref}}=1.8631\) m; wing point \([0.09750,0,-0.08071]\) m | Lift, roll, and yaw increments; effectiveness reduced beyond \(\lvert\alpha\rvert=8^\circ\) |
| Pylon-interference drag | Area \(0.003355\ \mathrm{m^2}\); point \([0.05744,0,-0.10074]\) m | Axial drag |
| Horizontal tail/elevator | \(S_H=0.1686\ \mathrm{m^2}\); point \([-1.19749,0,-0.11522]\) m | Local-flow lift and drag; dynamic-pressure ratio 0.8; \(\tau_e=0.518\); \(\delta_e=\pm20^\circ\) |
| Two vertical tails/rudders | Each \(S_V=0.06793\ \mathrm{m^2}\); points \([-1.24586,\pm0.37165,-0.17647]\) m | Local-flow side force and drag; \(\tau_r=0.27\); \(\delta_r=\pm20^\circ\); sidewash-rate correction |

The tail models use the local velocity at each reference point, including \(\boldsymbol\omega_B\times\mathbf r\). The vertical-tail sidewash-rate correction uses \(\partial\sigma/\partial p=-0.1\), \(\partial\sigma/\partial r=0\), and a minimum effective axial speed of \(10.668\ \mathrm{m\,s^{-1}}\). Fuselage tables use the translational velocity at the aircraft center of gravity, with the reference-point offset included when moments are transferred to the trim center of gravity.

The trim calculation is performed on the undeformed reference geometry. At each residual evaluation, the loads of the two rotor–wing subsystems are averaged over one revolution using 12 equally spaced azimuths, and the mean inflow is iterated to convergence. After trim, the trimmed pitch angle, right and left collectives, airspeed, prescribed bank angle, and motion definition are passed to the flexible MTR rotor–wing calculation. The auxiliary fuselage, tail, and control-surface loads are not applied in that time-domain stage, and no flexible fuselage or tail degrees of freedom are introduced. The empirical rotor-sidewash multiplier documented in the accompanying data repository (https://github.com/YOUR_GITHUB_USERNAME/MTR-XV15-maneuver-flutter-data/releases/tag/v1.0) is retained in the auxiliary trim model, but explicit rotor-to-fin induced velocities and the associated wake-boundary logic are not represented; the vertical-tail wake dynamic-pressure factor is taken as unity. Accordingly, the model is a full-aircraft trim extension of the wind-tunnel-assessed MTR rotor–wing system, not a complete flexible XV-15 simulation.

### 3.3. Maneuver Cases and Computational Setup

Three prescribed operating conditions are considered: straight-and-level flight, a coordinated level turn, and a prescribed steady pull-up initialized at the horizontal-tangent point of the circular trajectory. The nacelles are fixed in airplane mode at \(90^\circ\), both rotors operate at 1050 rpm, the airframe flap setting is zero, and the ambient density is \(1.2\ \mathrm{kg\,m^{-3}}\) for all cases. The maneuver definitions follow Sec. 2.3 and are illustrated schematically in Fig. 4.

![Prescribed straight flight, coordinated turn, and prescribed steady pull-up](figures/Manuscript/Figure4_Prescribed_Maneuvers.svg)

*Figure 4. Prescribed steady operating conditions: (a) straight-and-level flight, (b) coordinated level turn, and (c) prescribed steady pull-up initialized at the horizontal-tangent operating point. The arrows identify the trajectory velocity, curvature radius, and normal acceleration used in the kinematic definitions.*

The load factors were selected so that the turn and pull-up cases can be compared at matched values of \(n\). The primary range, \(1\le n\le2\), is adopted here as a representative interval spanning level flight through demanding but conventional prescribed steady-maneuver conditions. For the coordinated turn, \(n=1/\cos\lvert\phi\rvert\); for the pull-up, \(R_P=V^2/[g(n-1)]\) at the selected operating point. A single turn sense, \(s_T=-1\), is used for all coordinated-turn calculations, and Table 4 reports the corresponding bank-angle magnitudes and case matrix. One additional matched pair at \(n=2.5\) is retained only as a high-load check outside the primary range and is excluded from the subsequent correlation fit.

**Table 4. Primary prescribed-maneuver matrix and high-load checks outside the primary range.**

| Load factor, \(n\) | Straight flight | Coordinated-turn bank angle, \(\lvert\phi\rvert\) | Prescribed steady pull-up | Nominal airspeed grid | Retained comparison interval |
|---:|---:|---:|---:|---:|---:|
| 1.0000 | Baseline | \(0^\circ\) | Level-flight limit | 100–360 kt, 20-kt increments | 140–320 kt common primary interval |
| 1.1547 | — | \(30^\circ\) | \(n=1.1547\) | 100–360 kt, 20-kt increments | 140–320 kt common primary interval |
| 1.5000 | — | \(48.1897^\circ\) | \(n=1.5\) | 120–360 kt, 20-kt increments | 140–320 kt common primary interval |
| 2.0000 | — | \(60^\circ\) | \(n=2.0\) | 140–360 kt, 20-kt increments | 140–320 kt common primary interval |
| 2.5000\(^a\) | — | \(66.4218^\circ\) | \(n=2.5\) | 100–360 kt, 20-kt increments | 160–300 kt |

\(^a\) High-load check outside the empirical-fit calibration interval; excluded from correlation fitting and from the primary maneuver-range plots in Secs. 4.1 and 4.2.

For the primary cases, the different nominal lower speed limits reflect the converged airplane-mode trim envelope obtained as the load factor is increased. Only converged trim points were scheduled for transient analysis, and only complete response records were retained for damping estimation. The seven primary conditions possess a common 140–320-kt interval of complete records; direct cross-maneuver comparisons over \(1\le n\le2\) are therefore made on this common grid, while the additional condition-specific points outside it extend branch tracking. For the two \(n=2.5\) checks, complete records used for beamwise, chordwise, and torsional branch tracking were retained from 160 to 300 kt. These two cases are used only to examine whether the monotonic boundary trend and the close turn–pull-up comparison persist beyond the fitted range; they do not enlarge the stated validity range of the correlation.

The algebraic trim and nonlinear time-domain settings are summarized in Table 5. At every trim point, the six unknowns defined in Eq. (18) are solved simultaneously. Twelve equally spaced rotor azimuths are used to obtain the revolution-averaged rotor–wing loads. The mean-inflow fixed-point iteration is limited to 15 iterations with a convergence tolerance of \(10^{-7}\). A trim point is accepted only when every force and moment residual is below \(0.01\ \mathrm{N}\) and \(0.01\ \mathrm{N\,m}\), respectively.

In the flexible calculation, the aircraft-root translation and rotation are prescribed and the ambient air is stationary; hence, the local relative velocity is generated consistently from the prescribed aircraft motion, rotor rotation, and elastic motion. Structural gravity loading is disabled, as stated in Sec. 2.3; weight is used only in the aircraft-level trim balance. The time step is defined by 90 rotor-azimuth increments per revolution, corresponding to \(\Delta\Psi_r=4^\circ\). At 1050 rpm, the rotor period is 0.0571429 s and the resulting time step is \(6.349\times10^{-4}\) s. Each condition is scheduled for 50 rotor revolutions, or 2.857 s of response. Some high-speed cases did not complete because the within-step nonlinear iteration ceased to converge; these incomplete records are excluded from damping estimation and are not used by themselves to classify stability.

**Table 5. Common trim and transient-analysis settings.**

| Setting | Value |
|---|---:|
| Air density | 1.2 kg m\(^{-3}\) |
| Speed of sound used for auxiliary Mach interpolation | 340.3 m s\(^{-1}\) |
| Gravitational acceleration | 9.80665 m s\(^{-2}\) |
| Nacelle angle / airframe flap setting | \(90^\circ/0^\circ\) |
| Right / left rotational-speed magnitude | 1050 / 1050 rpm, with opposite rotation senses |
| Azimuth samples per trim residual evaluation | 12 per revolution |
| Maximum mean-inflow iterations / tolerance | 15 / \(10^{-7}\) |
| Trim force / moment residual limits | \(0.01\ \mathrm{N}\) / \(0.01\ \mathrm{N\,m}\) |
| Numerical bounds on trim variables | \(\theta_{\mathrm{tr}}\in[-20^\circ,20^\circ]\); \(\theta_{0,c}\in[0^\circ,90^\circ]\); \(\Delta\theta_0,\delta_e,\delta_a,\delta_r\in[-20^\circ,20^\circ]\) |
| Time-domain azimuth increments | 90 per revolution (\(4^\circ\)) |
| Time step at 1050 rpm | \(6.349\times10^{-4}\) s |
| Scheduled response duration | 50 rotor revolutions (2.857 s) |
| Newmark parameters, \(\gamma\) and \(\beta\) | 0.5 and 0.25 |
| Structural gravity in flexible stage | Disabled |
| Auxiliary XV-15/GTRS airframe aerodynamics in flexible stage | Not applied |

The nominal response-processing settings are fixed across all maneuver families and load factors. The beam branch is tracked over 2–8 Hz and fitted from 1.00 to 2.27 s using a moving-peak logarithmic envelope. The chord branch is tracked over 6–14 Hz; a fourth-order forward–backward zero-phase Butterworth bandpass with a half-width equal to 20% of the tracked frequency is applied before fitting from 0.50 to 1.77 s. For torsion, iterative Gaussian complex demodulation separates low- and high-frequency branches searched over 10.0–17.5 and 17.5–22.5 Hz, respectively. Eight demodulation iterations and a Gaussian width of 0.12 s are used; the low- and high-frequency logarithmic envelopes are fitted over 0.25–0.65 and 0.50–1.77 s, respectively.

Only response records that complete all 50 scheduled revolutions, contain finite samples throughout, and return finite branch-separation and regression results are retained. The coefficient of determination of each logarithmic-envelope fit is recorded as a quality diagnostic rather than imposed as a universal rejection threshold. Apparent noncontrolling beam or chord estimates contaminated by growing torsional content are therefore not used to redefine the earlier system boundary. These rules define the nominal basis against which the one-at-a-time postprocessing variations in Sec. 4.4 are evaluated.

The transient-stability evaluation follows Ref. 36 and the definition in Sec. 2.5. Beamwise, chordwise, and torsional response channels are used to follow the corresponding physical branches, without treating the response-processing procedure as a separate contribution of the present paper. When the identified damping changes sign between two adjacent 20-kt points, the zero-damping speed is obtained by the linear interpolation in Eq. (24). Thus, the reported flutter-onset values are interpolated stability boundaries, not additional simulations performed at the exact interpolated speeds. The boundary-bracketing torsional-response cases for the matched turn–pull-up pairs were rerun using the same nonlinear transient solver and time-integration settings.

## 4. Results and Discussion

The results are presented in four steps. Sections 4.1 and 4.2 give the turn and pull-up trends, Sec. 4.3 compares the two maneuvers at matched load factor, and Sec. 4.4 separates the effects of trim and prescribed motion. The \(n=2.5\) cases are shown only as a check outside the fitted range.

For all damping plots in this section, the solid curve denotes the arithmetic mean of the paired full-span response-channel estimates and the surrounding band spans those two estimates. The mean and range summarize the two full-span response channels; small channel-to-channel differences are not interpreted physically. The stability boundary is not obtained from the mean curve. Consistent with Sec. 2.5, the system flutter-onset speed is the earliest qualifying positive-to-negative damping crossing among all tracked physical branches of the full-span model, provided that a lower-speed stable interval has been established.

### 4.1. Coordinated-Turn Stability

Figure 5 compares the algebraic trim solutions for straight-and-level flight and coordinated turns at \(n=1.1547\), 1.5, and 2.0, corresponding to bank-angle magnitudes of \(30^\circ\), \(48.1897^\circ\), and \(60^\circ\), respectively. At a fixed airspeed, the required aerodynamic resultant increases with load factor. The trim solution accommodates this requirement primarily through increases in effective angle of attack, common collective, and elevator deflection. The coordinated-turn solutions also require small differential-collective and lateral-control inputs to balance the residual rolling and yawing moments. These lateral inputs increase with bank angle but remain much smaller than the common collective.

![Coordinated-turn trim results](figures/Manuscript/Figure5_Coordinated_Turn_Trim.svg)

*Figure 5. Primary-range full-aircraft trim results for straight-and-level flight and coordinated level turns at \(n=1.1547\), 1.5, and 2.0: pitch attitude, common and differential collective, elevator, aileron, and rudder deflections.*

The trim variables change smoothly over the converged speed range. At 220 kt, increasing the bank angle from \(0^\circ\) to \(60^\circ\) raises the common collective from \(61.387^\circ\) to \(61.579^\circ\), the differential collective from zero to about \(0.169^\circ\), and the elevator from \(4.933^\circ\) to \(6.545^\circ\). The pitch attitude changes less because the bank angle helps align the aircraft with the required resultant. Effective angle of attack and component loads are therefore more useful than pitch attitude alone when comparing the turn cases.

Figure 6 shows the corresponding beamwise, chordwise, and high-frequency torsional damping trends. The beamwise branch remains positively damped over the speed range containing the first stability boundary and follows a similar trend for all four load factors. At the highest completed speeds, particularly for \(n=1.5\) and 2.0, the decrease in the beam-response estimate coincides with growing torsional content in the response channel and is therefore treated as modal contamination rather than an independently established beam-mode instability. The chordwise branch is comparatively insensitive to the turn load factor and remains positively damped over the completed range.

![Coordinated-turn damping results](figures/Manuscript/Figure6_Coordinated_Turn_Damping.svg)

*Figure 6. Primary-range beamwise, chordwise, and high-frequency torsional damping trends for straight-and-level flight and coordinated level turns. Curves show the paired-channel mean and bands show the paired-channel range; the system boundary is determined independently from the earliest qualifying stable-to-unstable crossing of a physical branch.*

The high-frequency torsional branch controls the first instability. Its damping falls after a low-speed maximum, and the zero crossing moves steadily downward as \(n\) increases. The onset speed is 253.9 kt in level flight and 250.7, 238.0, and 221.2 kt at \(n=1.1547\), 1.5, and 2.0. The low-frequency torsional branch remains stable, and neither the beamwise nor chordwise branch sets the first boundary.

### 4.2. Prescribed Steady-Pull-Up Stability

Figure 7 presents the trim solutions for prescribed steady pull-ups at the same four load factors. The pull-up is evaluated at the initial horizontal-tangent state defined in Sec. 2.3. Because the pull-up cases are longitudinally symmetric, differential collective, aileron, and rudder remain numerically zero. Increasing \(n\) requires a larger effective angle of attack, common collective, and elevator deflection. In contrast to the coordinated turn, the required resultant is aligned in the aircraft symmetry plane, so the pitch attitude changes more strongly with load factor. At 220 kt, for example, the pitch attitude increases from \(2.115^\circ\) in level flight to \(4.444^\circ\) at \(n=2\), whereas the common collective increases from \(61.387^\circ\) to \(61.578^\circ\).

![Steady-pull-up trim results](figures/Manuscript/Figure7_Steady_Pullup_Trim.svg)

*Figure 7. Primary-range full-aircraft trim results for straight-and-level flight and prescribed steady pull-ups at \(n=1.1547\), 1.5, and 2.0: pitch attitude, common collective, and elevator deflection. Differential collective, aileron, and rudder remain numerically zero for these longitudinally symmetric cases and are therefore omitted from the panels.*

The pull-up and turn require different body attitudes, but their common collectives are almost identical at matched \(n\). Section 4.3 compares the corresponding rotor–wing operating points in more detail.

The pull-up damping trends in Fig. 8 closely parallel the coordinated-turn results. The beamwise damping changes only modestly through the speed range containing the first zero crossing. The apparent beamwise reduction at the highest speeds coincides with strong torsional content in the response channel and is therefore treated as modal contamination rather than a separately established beam instability. The chordwise damping curves remain closely grouped and positive. The low-frequency torsional branch (not shown) also remains positive, whereas the plotted high-frequency torsional branch decreases toward and through zero over the boundary-bracketing speed range and controls the system boundary.

![Steady-pull-up damping results](figures/Manuscript/Figure8_Steady_Pullup_Damping.svg)

*Figure 8. Primary-range beamwise, chordwise, and high-frequency torsional damping trends for straight-and-level flight and prescribed steady pull-ups. Curves show the paired-channel mean and bands show the paired-channel range; the system boundary is determined independently from the earliest qualifying stable-to-unstable crossing of a physical branch.*

The pull-up onset speeds are 250.9, 237.3, and 221.0 kt at \(n=1.1547\), 1.5, and 2.0, compared with 253.9 kt in level flight. Their close agreement with the turn results shows that the reduction is not tied to the horizontal turning path. Both maneuvers lead to nearly the same change in the rotor–wing operating point.

### 4.3. Load-Factor Dependence and Empirical Boundary Representation

Table 6 shows the same trend for both maneuvers. Relative to level flight, the boundary drops by about 16 kt at \(n=1.5\) and 33 kt at \(n=2\). At a given \(n\), the turn and pull-up values differ by less than 1 kt—well below the resolution implied by the 20-kt bracketing grid. The \(n=2.5\) pair also remains close, with a 1.3-kt difference.

**Table 6. System flutter-onset speeds over the primary range and at the \(n=2.5\) high-load condition.**

| Load factor, \(n\) | Coordinated turn \(V_f\), kt | Steady pull-up \(V_f\), kt | Absolute difference, kt |
|---:|---:|---:|---:|
| 1.0000 | 253.9 | 253.9 | 0.0 |
| 1.1547 | 250.7 | 250.9 | 0.2 |
| 1.5000 | 238.0 | 237.3 | 0.7 |
| 2.0000 | 221.2 | 221.0 | 0.3 |
| 2.5000\(^a\) | 205.7 | 207.0 | 1.3 |

\(^a\) High-load check outside the empirical-fit calibration interval; excluded from Eqs. (35)–(37) and all primary-range empirical fits.

The full-aircraft force balance gives a useful starting point for interpreting this agreement. With the inertial \(z\)-axis positive downward, the revolution-averaged balance at the reference center of gravity is

\[
\overline{\mathbf F}_A^I+W\mathbf e_{z_I}=m\mathbf a_O^I,
\tag{29}
\]

where \(W=mg\) and \(\overline{\mathbf F}_A^I\) is the total trim aerodynamic force after revolution averaging of the two proprotors and flexible wings and inclusion of the auxiliary-airframe contributions.

At the horizontal-tangent point of the prescribed pull-up,

\[
\mathbf a_{O,P}^{I}=-(n-1)g\mathbf e_{z_I},
\qquad
\overline{\mathbf F}_{A,P}^{I}=-nW\mathbf e_{z_I},
\qquad
\left\|\overline{\mathbf F}_{A,P}^{I}\right\|=nW.
\tag{30}
\]

For a coordinated level turn evaluated at the reference azimuth \(\chi=0\), let \(\phi_T=\lvert\phi\rvert\) denote the bank-angle magnitude and \(s_T=\pm1\) the turn direction. Then,

\[
\mathbf a_{O,T}^I=s_Tg\tan\phi_T\,\mathbf e_{y_I},
\qquad
\overline{\mathbf F}_{A,T}^I=s_TW\tan\phi_T\,\mathbf e_{y_I}-W\mathbf e_{z_I},
\tag{31}
\]

and therefore

\[
\left\|\overline{\mathbf F}_{A,T}^{I}\right\|
=W\sqrt{1+\tan^2\phi_T}
=\frac{W}{\cos\phi_T}
=nW.
\tag{32}
\]

Equations (30) and (32) give the same resultant magnitude at matched \(n\), even though the inertial-frame directions differ. Once the turn force is resolved in body axes, both maneuvers require nearly the same longitudinal and normal components,

\[
\overline{\mathbf F}_A^B
\simeq
nW
\begin{bmatrix}
\sin\alpha_{\mathrm{eff}} & 0 & -\cos\alpha_{\mathrm{eff}}
\end{bmatrix}^{T}.
\tag{33}
\]

where \(\overline{\mathbf F}_A^B=\mathbf C_{BI}\overline{\mathbf F}_A^I\) and \(\alpha_{\mathrm{eff}}=\operatorname{atan2}(w_O^B,u_O^B)\) is the effective body-axis angle of attack defined from the trim translational-velocity components in the body \(x\)–\(z\) plane.

Equal resultant force does not by itself imply equal flutter speed; the load distribution, moments, inflow, and phase relationships still matter. The trim results nevertheless show that the two operating points are very close. Over 200–300 kt and \(n=1.1547\), 1.5, and 2.0, the effective-angle-of-attack difference is at most \(0.0034^\circ\), the common-collective difference is at most \(0.0026^\circ\), and the summed proprotor-plus-wing body-\(z\) force differs by less than 0.13%. These comparisons apply to the primary range only. They do not imply identical local inflow, spanwise loading, or periodic loads.

The operating points are not identical. Coordinated turns retain differential collective and small lateral-control inputs, whereas the pull-ups are longitudinally symmetric. The two trajectories also have different prescribed angular velocities,

\[
\Omega_T=\frac{g\sqrt{n^2-1}}{V},
\qquad
\Omega_P=\frac{g(n-1)}{V},
\tag{34}
\]

and different directions of translational acceleration. The agreement in Table 6 is therefore a result of this model, not a general identity. The similar trim states point to a common cause, which is tested directly in Sec. 4.4 at \(n=2\).

The dependence of the system boundary on load factor is represented by

\[
V_{f,j}(n)
=V_{f,1}\sqrt{1-\kappa_j(n-1)},
\qquad
V_{f,1}=253.9\ \mathrm{kt},
\tag{35}
\]

where \(j\) denotes the maneuver family and \(V_{f,1}\) is the \(n=1\) straight-and-level baseline. With \(V_{f,1}\) fixed, \(\kappa_j\) is obtained by ordinary least squares in the transformed relation \(1-(V_f/V_{f,1})^2=\kappa_j(n-1)\). Fits over \(1\le n\le2\) give

\[
\kappa_{\mathrm{turn}}=0.240,
\qquad
\kappa_{\mathrm{pull}}=0.243.
\tag{36}
\]

The corresponding speed root-mean-square errors are 0.8 and 1.0 kt. Because the fitted coefficients differ by only approximately 1.3%, the two datasets can be pooled to give

\[
V_f(n)
\approx
253.9\sqrt{1-0.241(n-1)}\ \mathrm{kt},
\qquad 1\le n\le2.
\tag{37}
\]

with a pooled root-mean-square error of approximately 0.9 kt. Figure 9 compares the two primary-range maneuver datasets, their separate fits, the combined empirical representation, and the \(n=2.5\) high-load check outside the fitted range.

![Flutter speed versus load factor](figures/Manuscript/Figure9_Flutter_Speed_vs_Load_Factor.svg)

*Figure 9. Predicted system flutter-onset speed versus load factor for coordinated turns and prescribed steady pull-ups. The maneuver-specific fits and the combined empirical representation use only the \(1\le n\le2\) cases. The filled symbols at \(n=2.5\) denote high-load checks excluded from the fits, and the gray dash-dotted segment shows the combined empirical representation extrapolated beyond its \(1\le n\le2\) calibration interval.*

At \(n=2.5\), the turn and pull-up boundaries are 205.7 and 207.0 kt, with both zero crossings lying in the 200–220-kt bracket. Equation (37) gives 202.8 kt without refitting, 2.9 and 4.2 kt below the calculated values. The trend therefore continues at this higher load, but the residuals are already large enough to discourage extrapolation beyond the fitted range.

For the exploratory \(n=3\) calculations, the tracked high-frequency torsional channels produced a system-level unstable–stable–unstable classification at 160, 180, and 200 kt, respectively, and therefore multiple zero crossings on the retained 160–300-kt grid. Because the system was already unstable at the lowest retained speed, a unique first loss-of-stability speed could not be assigned within the available speed interval; these cases are excluded from Table 6, Fig. 9, and all primary-range empirical fits.

Equation (37) is only a compact fit to the calculated \(1\le n\le2\) boundaries for this model at 1050 rpm. The square-root form is equivalent to a linear fit in \(V_f^2\), and its coefficient depends on the structural, aerodynamic, and prescribed-motion assumptions used here. The \(n=2.5\) cases are checks, not fitting points.

### 4.4. Separation of Trim and Prescribed-Motion Effects

The matched-\(n\) results point to the trim state, rather than the imposed trajectory, as the main source of the boundary reduction. Here, *trim inputs* means the pitch attitude and the two rotor collectives passed from the aircraft-level solution to the flexible model. A six-state ablation was therefore run at \(n=2\) and 200–300 kt. The pull-up cases form a \(2	imes2\) combination of trim inputs and prescribed motion; two turn cases provide the corresponding check for the turning trajectory. In total, 36 fifty-revolution simulations were completed.

Table 7 defines the six states and lists their system onset speeds. The parameter envelopes were obtained by varying the branch-isolation and decay-fitting settings one at a time while reprocessing the same response records. They represent numerical postprocessing-sensitivity ranges, not statistical confidence intervals.

**Table 7. System flutter-onset speeds from the \(n=2\) trim–motion ablation.**

| State | Transferred trim inputs | Prescribed motion | System \(V_f\), kt | Change from A, kt | Parameter-sensitivity envelope, kt |
|---|---|---|---:|---:|---:|
| A | Level-flight trim inputs | Straight flight | 253.9 | 0.0 | 253.1–254.7 |
| BP | \(n=2\) pull-up trim inputs | Straight flight | 219.7 | -34.2 | 219.1–221.1 |
| CP | Level-flight trim inputs | \(n=2\) pull-up | 257.0 | +3.1 | 256.3–258.3 |
| DP | \(n=2\) pull-up trim inputs | \(n=2\) pull-up | 221.0 | -33.0 | 220.2–222.3 |
| BT | \(n=2\) coordinated-turn trim inputs | Straight flight | 220.6 | -33.3 | 219.5–222.6 |
| DT | \(n=2\) coordinated-turn trim inputs | \(n=2\) coordinated turn | 221.2 | -32.7 | 220.2–223.2 |

![Trim and prescribed-motion ablation](figures/Manuscript/Figure10_Trim_Motion_Ablation.svg)

*Figure 10. High-frequency torsional damping used to separate transferred-trim-input and prescribed-motion effects at \(n=2\): (a) the pull-up \(2\times2\) decomposition and (b) the coordinated-turn trim/motion check. Curves show paired-channel means; Table 7 reports the system boundaries obtained from the earliest qualifying stable-to-unstable crossing of a physical branch.*

Changing only the transferred trim inputs produces the dominant reduction within the \(n=2\), 200–300-kt ablation set. The pull-up trim inputs with straight-flight kinematics, BP, lower the boundary by 34.2 kt relative to A. The corresponding coordinated-turn trim inputs, BT, lower it by 33.3 kt. In the pointwise damping comparison over 200–300 kt, the mean high-frequency torsional-damping changes BP–A and BT–A are \(-0.4347\) and \(-0.4320\) percentage points, respectively. The close agreement of these independent trim-input changes is consistent with the comparable trim-level indicators established in Sec. 4.3.

Adding the prescribed motion at fixed maneuver-trim inputs has a much smaller effect. DP–BP raises the onset by 1.2 kt and DT–BT by 0.6 kt. Their mean damping increments, \(+0.0167\) and \(+0.0200\) percentage points, are only about 4–5% of the trim-input-induced change. The motion-only pull-up case CP stays close to the level-flight baseline, with a 3.1-kt increase rather than the roughly 33-kt decrease of the full maneuver. Across the three motion comparisons, the damping changes are about 4–8% of the trim-input effect. The pull-up interaction term, \((\mathrm{DP}-\mathrm{BP})-(\mathrm{CP}-\mathrm{A})\), is also small at \(-0.0164\) percentage points.

The DP–BP and DT–BT increments are smaller than the widths of the corresponding postprocessing-sensitivity envelopes, which overlap. Their slightly stabilizing sign is consistent with the mean damping, but the exact increment is not resolved. What is clear is the difference in scale: the trim-input effect is about 33–34 kt, whereas the prescribed-motion correction is small.

The angular-rate scales are consistent with this result. Over 200–300 kt at \(n=2\), the pull-up pitch rate is \(0.0635\)–\(0.0953\ \mathrm{rad\,s^{-1}}\) and the turn rate is \(0.110\)–\(0.165\ \mathrm{rad\,s^{-1}}\), compared with a rotor speed of \(109.96\ \mathrm{rad\,s^{-1}}\). This comparison does not isolate individual inertial terms, but it helps explain why their net effect is small in the ablation.

A useful comparison is the flexible flying-wing study of Naftaly and Raveh (Ref. 39). In their A3TB model, increasing the coordinated-turn load factor from \(1g\) to \(5g\) reduced the critical symmetric body-freedom-flutter speed by 2.6%; another mechanism was unchanged and a third was stabilized by about 3%. The present tiltrotor boundary drops by 6.3% at \(n=1.5\) and 12.9% at \(n=2\). The full-maneuver boundary change predicted for the tiltrotor is therefore substantially larger than the maneuver-coupling change reported for the flying-wing model.

The comparison is not one-to-one. The aircraft, instability mechanisms, aerodynamic models, and analysis methods differ. Naftaly and Raveh held the aerodynamic operating point fixed and isolated selected inertial-coupling terms. Here, maneuver trim also changes pitch attitude, collective pitch, mean inflow, and rotor–wing loading. The ablation points instead to the changed rotor–wing operating point as the main source of the larger shift.

For the \(n=2\), 200–300-kt cases, the transferred trim inputs clearly dominate the boundary reduction. The prescribed motion contributes only a small nominal stabilization, and its exact size is not resolved by the postprocessing variations. The lower-load cases show the same pattern indirectly through their close trim states and boundaries. This conclusion applies to the prescribed-root model used here; releasing the aircraft motion or adding flexible fuselage and tail dynamics, gravity prestress, or flight-control feedback could change the balance.

## 5. Conclusions

A full-span flexible tiltrotor model was used to examine straight flight, coordinated turns, and prescribed steady pull-ups. The main findings are:

1. The fixed-support MTR rotor–wing model reproduces the main trends in the available wind-tunnel damping data. These comparisons support the rotor–wing core but do not validate the hybrid aircraft trim model or the prescribed maneuver extension.

2. The flutter boundary decreases as load factor increases. The high-frequency torsional branch controls the first instability. From a level-flight value of 253.9 kt, the boundary falls to 221.2 kt in the \(n=2\) turn and 221.0 kt in the corresponding pull-up. The \(n=2.5\) checks give 205.7 and 207.0 kt.

3. At matched \(n\), the turn and pull-up boundaries differ by less than 1 kt over \(1\le n\le2\). Their effective angle of attack, common collective, and mean rotor–wing normal force are also nearly identical. For this model, the calculated primary-range boundaries are summarized by

   \[
   V_f(n)=253.9\sqrt{1-0.241(n-1)}\ \mathrm{kt},
   \qquad 1\le n\le2.
   \]

   The relation is an empirical fit to the calculations, not a general tiltrotor design law.

4. The \(n=2\) ablation shows that the maneuver-trim inputs dominate the boundary shift. Applying the pull-up or turn pitch attitude and rotor collectives with straight-flight kinematics lowers the boundary by about 34 kt. Adding the corresponding prescribed motion changes it by only 0.6–1.2 kt, a difference smaller than the postprocessing-sensitivity range.

The model prescribes the aircraft motion and does not include two-way feedback from the flexible loads to the trajectory. The auxiliary fuselage and tail aerodynamics are used for trim only, and the available torsion measurements cover a limited speed range. Further work should use a finer speed grid near the boundary and extend the model toward periodic linearization and released-flight coupling.

## Appendix A. Auxiliary-Airframe Implementation Summary

The auxiliary XV-15/GTRS model is used only to close the aircraft-level algebraic trim. It does not add flexible fuselage or tail degrees of freedom, and its loads are not applied in the nonlinear rotor–wing time-domain calculation.

**Table A1. Use of the principal GTRS model branches.**

| Model branch | Use in this study |
|---|---|
| Tabulated fuselage forces and moments | Active in trim |
| External landing-gear/pod and pylon-interference drag | Active in trim |
| Aileron lift, roll, and yaw increments | Active in trim |
| Horizontal-tail/elevator loads | Active in trim |
| Vertical-tail/rudder loads and empirical sidewash multipliers | Active in trim |
| Baseline rigid-wing lift, drag, and pitching moment | Inactive, except for the baseline \(C_L\) lookup used in the aileron–yaw increment |
| GTRS rigid-wing roll- and yaw-rate derivatives | Inactive |
| Explicit rotor-to-fin induced velocities and wake-boundary tests | Inactive |

One-dimensional data are linearly interpolated and two-dimensional data are bilinearly interpolated, with inputs clamped to the tabulated ranges. The low-Mach branch is retained through \(M=0.2\), followed by linear interpolation to the \(M=0.4\) branch where applicable. Dimensional force and moment ordinates are scaled according to Eqs. (25) and (26).

The full fuselage, wing, downwash, sidewash, elevator, rudder, and drag arrays, together with the exact interpolation equations used by the trim implementation, are available at https://github.com/YOUR_GITHUB_USERNAME/MTR-XV15-maneuver-flutter-data/releases/tag/v1.0.

## References
1. Reed, W. H., III, “Propeller-Rotor Whirl Flutter: A State-of-the-Art Review,” *Journal of Sound and Vibration*, Vol. 4, No. 3, 1966, pp. 526–544. https://doi.org/10.1016/0022-460X(66)90142-8.

2. Johnson, W., *Dynamics of Tilting Proprotor Aircraft in Cruise Flight*, NASA TN D-7677, National Aeronautics and Space Administration, 1974. https://ntrs.nasa.gov/citations/19740019387.

3. Kvaternik, R. G., and Kohn, J. S., *An Experimental and Analytical Investigation of Proprotor Whirl Flutter*, NASA TP-1047, Report No. L-11656, National Aeronautics and Space Administration, Dec. 1977. https://ntrs.nasa.gov/citations/19780004096.

4. Popelka, D., Sheffler, M., and Bilger, J., “Correlation of Test and Analysis for the 1/5-Scale V-22 Aeroelastic Model,” *Journal of the American Helicopter Society*, Vol. 32, No. 2, 1987, pp. 21–33. https://doi.org/10.4050/JAHS.32.21.

5. Piatak, D. J., Kvaternik, R. G., Nixon, M. W., Langston, C. W., Singleton, J. D., Bennett, R. L., and Brown, R. K., “A Parametric Investigation of Whirl-Flutter Stability on the WRATS Tiltrotor Model,” *Journal of the American Helicopter Society*, Vol. 47, No. 2, 2002, pp. 134–144. https://doi.org/10.4050/JAHS.47.134.

6. Kreshock, A. R., Acree, C. W., Kang, H., and Yeo, H., “Development of a New Aeroelastic Tiltrotor Wind Tunnel Testbed,” *AIAA SciTech 2019 Forum*, San Diego, CA, 2019, AIAA Paper 2019-2133. https://doi.org/10.2514/6.2019-2133.

7. Kreshock, A. R., Thornburgh, R. P., Wilbur, M. L., Kang, H., Piatak, D. J., and Sekula, M. K., “Initial Whirl-Flutter Characterization of the TiltRotor Aeroelastic Stability Testbed,” *VFS 79th Annual Forum & Technology Display*, West Palm Beach, FL, 2023. https://doi.org/10.4050/F-0079-2023-18037.

8. Tsai, F., Sutherland, J., Akinwale, A., Morin, A., Gul, S., and Datta, A., “Development and Whirl Flutter Test of the Maryland Tiltrotor Rig,” *Journal of the American Helicopter Society*, Vol. 69, Article 012009, 2024. https://doi.org/10.4050/JAHS.69.012009.

9. Gul, S., and Datta, A., “Prediction and Validation of Whirl Flutter Data of the Maryland Tiltrotor Rig,” *Journal of the American Helicopter Society*, Vol. 69, Article 012011, 2024. https://doi.org/10.4050/JAHS.69.012011.

10. van ’t Hoff, S., Kapteijn, K., Schneider, O., Soal, K. I., and Fonte, F., “Whirl Flutter Testing of ATTILA Tiltrotor Testbed—Initial Results,” *VFS 80th Annual Forum & Technology Display*, Montréal, Québec, Canada, 2024. https://doi.org/10.4050/F-0080-2024-1117.

11. Zhang, W., Tang, M., Wu, J., Peng, X., Zhang, G., Nie, B., Wang, L., and Li, C., “Overview of Wind Tunnel Test Research on Tiltrotor Aircraft,” *Acta Aeronautica et Astronautica Sinica*, Vol. 45, No. 9, Article 530114, 2024. https://doi.org/10.7527/S1000-6893.2024.30114.

12. Srinivas, V., and Chopra, I., “Validation of a Comprehensive Aeroelastic Analysis for Tiltrotor Aircraft,” *Journal of the American Helicopter Society*, Vol. 43, No. 4, 1998, pp. 333–341. https://doi.org/10.4050/JAHS.43.333.

13. Kim, T., and Shin, S. J., “Advanced Analysis on Tiltrotor Aircraft Flutter Stability, Including Unsteady Aerodynamics,” *AIAA Journal*, Vol. 46, No. 4, 2008, pp. 1002–1012. https://doi.org/10.2514/1.33474.

14. Krüger, W. R., “Multibody Analysis of Whirl Flutter Stability on a Tiltrotor Wind Tunnel Model,” *Proceedings of the Institution of Mechanical Engineers, Part K: Journal of Multi-body Dynamics*, Vol. 230, No. 2, 2016, pp. 121–133. https://doi.org/10.1177/1464419315582128.

15. Yeo, H., and Kreshock, A. R., “Whirl Flutter Investigation of Hingeless Proprotors,” *Journal of Aircraft*, Vol. 57, No. 4, 2020, pp. 586–596. https://doi.org/10.2514/1.C035609.

16. Cocco, A., Mazzetti, S., Masarati, P., van ’t Hoff, S., and Timmerman, B., “Numerical Whirl–Flutter Analysis of a Tiltrotor Semi-Span Wind Tunnel Model,” *CEAS Aeronautical Journal*, Vol. 13, 2022, pp. 923–938. https://doi.org/10.1007/s13272-022-00605-2.

17. Gul, S., and Yeo, H., “Correlation of High-Speed Tiltrotor Stability Predictions with Test Data and Parametric Study,” *Journal of Aircraft*, Vol. 61, No. 4, 2024, pp. 1283–1292. https://doi.org/10.2514/1.C037807.

18. Masarati, P., Piatak, D. J., Quaranta, G., Singleton, J. D., and Shen, J., “Soft-Inplane Tiltrotor Aeromechanics Investigation Using Two Comprehensive Multibody Solvers,” *Journal of the American Helicopter Society*, Vol. 53, No. 2, 2008, pp. 179–192. https://doi.org/10.4050/JAHS.53.179.

19. Yeo, H., Kang, H., and Kreshock, A. R., “Unified Modeling of the TiltRotor Aeroelastic Stability Testbed for Proprotor Whirl Flutter,” *Journal of Aircraft*, Vol. 60, No. 3, 2023, pp. 857–869. https://doi.org/10.2514/1.C036950.

20. Cocco, A., and Savino, A., “Tiltrotor Whirl-Flutter Assessment by Multifidelity Aerodynamic Models,” *AIAA SciTech 2024 Forum*, Orlando, FL, 2024, AIAA Paper 2024-1850. https://doi.org/10.2514/6.2024-1850.

21. Dong, L., and Li, Q., “Whirl Flutter Suppression of Tiltrotor Aircraft Using Actively Controlled Aileron,” *Aerospace*, Vol. 9, No. 12, Article 795, 2022. https://doi.org/10.3390/aerospace9120795.

22. Acree, C. W., Jr., Peyran, R. J., and Johnson, W., *Rotor Design Options for Improving XV-15 Whirl-Flutter Stability Margins*, NASA/TP-2004-212262, AFDD/TR-04-001, National Aeronautics and Space Administration, Mar. 2004. https://ntrs.nasa.gov/citations/20040081235.

23. Li, Z., and Xia, P., “Aeroelastic Stability of Full-Span Tiltrotor Aircraft Model in Forward Flight,” *Chinese Journal of Aeronautics*, Vol. 30, No. 6, 2017, pp. 1885–1894. https://doi.org/10.1016/j.cja.2017.09.007.

24. Li, Z., and Xia, P., “Aeroelastic Modelling and Stability Analysis of Tiltrotor Aircraft in Conversion Flight,” *The Aeronautical Journal*, Vol. 122, No. 1256, 2018, pp. 1606–1629. https://doi.org/10.1017/aer.2018.93.

25. Mattaboni, M., Masarati, P., Quaranta, G., and Mantegazza, P., “Multibody Simulation of Integrated Tiltrotor Flight Mechanics, Aeroelasticity, and Control,” *Journal of Guidance, Control, and Dynamics*, Vol. 35, No. 5, 2012, pp. 1391–1405. https://doi.org/10.2514/1.57309.

26. Juhasz, O., Celi, R., and Tischler, M. B., “Flight Dynamics Simulation Modeling of a Large Flexible Tiltrotor Aircraft,” *Journal of the American Helicopter Society*, Vol. 67, Article 022003, 2022. https://doi.org/10.4050/JAHS.67.022003.

27. Cocco, A., Savino, A., and Masarati, P., “Flexible Multibody Model of a Complete Tiltrotor for Aeroservoelastic Analysis,” *ASME 2022 International Design Engineering Technical Conferences and Computers and Information in Engineering Conference*, St. Louis, MO, 2022, Paper DETC2022-89734, V009T09A026. https://doi.org/10.1115/DETC2022-89734.

28. Hoover, C. B., and Shen, J., “Whirl Flutter Analysis of a Free-Flying Electric-Driven Propeller Aircraft,” *Journal of Aircraft*, Vol. 56, No. 2, 2019, pp. 831–836. https://doi.org/10.2514/1.C035263.

29. Bath, P. E., Gaitonde, A. L., and Jones, D. P., “Reduced Order Aerodynamics and Flight Mechanics Coupling for Tiltrotor Aircraft Stability Analysis,” *AIAA SciTech 2022 Forum*, San Diego, CA, 2022, AIAA Paper 2022-0284. https://doi.org/10.2514/6.2022-0284.

30. Cocco, A., Savino, A., Zanoni, A., and Masarati, P., “Comprehensive Simulation of a Complete Tiltrotor with Pilot-in-the-Loop for Whirl-Flutter Stability Analysis,” *48th European Rotorcraft Forum*, Winterthur, Switzerland, 2022, Paper ERF-2022-078. https://hdl.handle.net/20.500.11881/4383.

31. Savino, A., Cocco, A., and Muscarello, V., “Evaluation of Dynamic Loads and Performance Indexes in Tiltrotors Using a Mid-Fidelity Aeroservoelastic Tool,” *Aerospace Science and Technology*, Vol. 146, Article 108929, 2024. https://doi.org/10.1016/j.ast.2024.108929.

32. Tuzcu, I., and Nguyen, N., “Flutter of Maneuvering Aircraft,” *Journal of Aerospace Engineering*, Vol. 28, No. 4, Article 04014094, 2015. https://doi.org/10.1061/(ASCE)AS.1943-5525.0000415.

33. Federal Aviation Administration, “Integration of Powered-Lift: Pilot Certification and Operations; Miscellaneous Amendments Related to Rotorcraft and Airplanes,” *Federal Register*, Vol. 89, No. 225, Nov. 21, 2024, pp. 92296–92522. https://www.govinfo.gov/content/pkg/FR-2024-11-21/pdf/2024-24886.pdf.

34. Federal Aviation Administration, *Type Certification—Powered-Lift*, Advisory Circular AC 21.17-4, July 18, 2025, particularly PL.2241 and PL.2245. https://www.faa.gov/documentLibrary/media/Advisory_Circular/AC_21.17-4.pdf.

35. European Union Aviation Safety Agency, *Special Condition for Small-Category VTOL-Capable Aircraft*, Issue 2, SC-VTOL-02, June 10, 2024, particularly VTOL.2245, “Aeroelasticity.” https://www.easa.europa.eu/en/downloads/139946/en.

36. Zhang, S., Li, L., Gong, Y., Yao, T., Wang, H., and Jiang, C., “Modeling and Analysis of Tiltrotor Whirl Flutter Based on Lagrange Equations of the Second Kind and Transient Perturbation Method,” *51st European Rotorcraft Forum*, Venice, Italy, 2025, Paper 098.

37. Akinwale, A., and Datta, A., “Understanding High-Speed Aeroelastic Stability of a Gimballed Proprotor,” *Journal of Aircraft*, Vol. 62, No. 2, 2025, pp. 313–327. https://doi.org/10.2514/1.C037994.

38. Ferguson, S. W., *A Mathematical Model for Real Time Flight Simulation of a Generic Tilt-Rotor Aircraft*, NASA CR-166536, Rev. A, National Aeronautics and Space Administration, Sept. 1988.

39. Naftaly, D., and Raveh, D. E., “Flight-Dynamics and Aeroelastic Coupling in Flexible Maneuvering Aircraft,” *International Forum on Aeroelasticity and Structural Dynamics (IFASD 2024)*, The Hague, The Netherlands, June 17–21, 2024, Paper IFASD-2024-000.
