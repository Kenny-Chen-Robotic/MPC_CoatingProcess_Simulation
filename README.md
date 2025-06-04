# Introduction
This project introduces an application demo of Model predict control (MPC) in coating process, which involves multivariable and complex couping situation. To emphasize it, two opposite simulation examples are exhibited as follows. The main purpose of simulation task is to eliminate bias of y1 value, it's interesting to see that the x2 is willing to act for this goal, even sacrificing its own bias y2 at the beginning in tight coupling sample. By contrast, these xi act as just being concentrated to their own yi bias in weak coupling sample, thus they act almost independently. 

The penalty term of terminal state is not yet concluded, which is simple to extend by adding term behind of cost function.

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
