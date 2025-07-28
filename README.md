# Introduction
As a tutorial demo, this repository introduces an application of Model Predictive Control (MPC) in industrial process, which involves multivariable and complex parameters couping situation. To illustarte it, two opposite simulation examples are exhibited as follows. The main purpose of the simulation task is to eliminate value bias of y1. It's interesting that in the tight coupling sample the x2 is willing to reach out for this goal at the beginning, even sacrificing its own bias which is y2. On the contrary, in the weak coupling sample these x_i act individually, only devote to their own y_i bias. 

The formation, items and coefficients of the cost function influcence the simulation result directly. The penalty term of terminal state is excluded here, which can be extended simply by adding terms according to your specific control demand.

<br>  <!-- 这是空行间隔 -->

<div align="center">
  <img src="images/TightCouplingResult.png" alt="TightCouplingResult" style="width: 800px; height: auto;"/>

  *Figure1: tight coupling sample*
</div>

<br>  <!-- 这是空行间隔 -->

<div align="center">
  <img src="images/WeakCouplingResult.png" alt="WeakCouplingResult" style="width: 800px; height: auto;"/>

  *Figure2: weak coupling sample*
</div>

<br>  <!-- 这是空行间隔 -->
# Advance research
Adaptive MPC / Nonlinear system identification (data-driven, approximate linearazation, etc) / Acceleration of sparse matrix solution.
