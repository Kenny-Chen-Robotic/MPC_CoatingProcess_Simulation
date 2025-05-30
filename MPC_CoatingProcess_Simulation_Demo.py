# -*- coding: utf-8 -*-
"""
Created on Tue May 27 10:16:44 2025

@author: zugang.chen

Triple order MPC with trajectory plotting
"""

import pandas as pd
import math
import numpy as np
import matplotlib.pyplot as plt
import os
import scipy
import cvxpy as cp
import time
import random

plt.rcParams['font.sans-serif'] = ['SimHei']  # 设置中文字体
plt.rcParams['axes.unicode_minus'] = False  # 解决负号显示问题

start = time.time()
iteration_steps = 90
Y_ref_list = [10, 40, -15] # 时变的目标值
update_circle = int(iteration_steps/len(Y_ref_list))

U_opt, xk = 0, 0
# 初始化历史记录
x_history = [np.array([[50.0], [-30.0], [-20.0]])]  # 初始状态

# 模型参数；输入输出变量默认3个, xk+1 = Axk + Buk, yk = Cxk，x为千分尺绝对行程，u为千分尺变化量, y为根据绝对行程推测的面密度
Np = 3
Nc = 3
# A = np.diag([1, 1, 1])
# B = np.diag([1, 1, 1])
# C = -1 * np.array([[0.4, 0.4, 0.2], [0.2, 0.6, 0.2], [0, 0.2, 0.6]]) # 强耦合
# # C = -1 * np.array([[1, 0.0, 0.0], [0.0, 0.6, 0.2], [0, 0.2, 0.6]]) # 弱耦合
# y_history = [C @ x_history[0]]  # 初始输出


# 另一种定义，这种定义带了实际值y观测值，更科学
# x为分区面密度（实测值，在预测出y之后可加白噪声得到下一个x模拟），u为各千分尺变化量，B为面密度变化量与千分尺变化量的关系，y=k=面密度输出
A = np.diag([1, 1, 1])
B = -1 * np.array([[0.5, 0.4, 0.2], [0.2, 0.7, 0.1], [0, 0.1, 0.7]]) # 强耦合
# B = -1 * np.array([[1, 0.0, 0.0], [0.0, 0.6, 0.2], [0, 0.2, 0.6]]) # 弱耦合
C = np.diag([1, 1, 1])
y_history = [C @ x_history[0]]  # 初始输出


# 主循环
for j in range(iteration_steps):
    # 获取当前状态
    xk = x_history[-1]
    
    # 参考轨迹更新
    if j % update_circle == 0 and (len(Y_ref_list)-1) * update_circle >= j:
        print(j)
        Y_ref = np.full((3 * Np, 1), Y_ref_list[int(j // update_circle)])
    
    # 构建预测矩阵, psi初始状态矩阵，phi输入矩阵
    psi = 0
    psi_temp = []
    Zeros = np.zeros((3, 3))
    
    for i in range(Np):
        if i == 0:
            psi_temp.append(C @ A)
            psi = psi_temp[-1]
        else:
            psi_temp.append(psi_temp[-1] @ A)
            psi = np.vstack([psi, psi_temp[-1]])
    
    phi1 = np.hstack([C @ B, Zeros, Zeros])
    phi2 = np.hstack([C @ A @ B, C @ B, Zeros])
    phi3 = np.hstack([C @ A @ A @ B, C @ A @ B, C @ B])
    phi = np.vstack([phi1, phi2, phi3])
    
    # 权重矩阵，大q鼓励尽快消除误差，小r鼓励大动作
    q = np.diag([4, 1, 1]) # 鼓励消除边缘的误差
    # q = np.diag([1, 1, 1]) # 各个误差重视度均等
    r = np.diag([1, 1, 1])
    Q_total = scipy.linalg.block_diag(q, q, q)
    R_total = scipy.linalg.block_diag(r, r, r)
    
    # 优化问题
    H = 2 * (phi.T @ Q_total @ phi + R_total)
    f = 2 * phi.T @ Q_total @ (psi @ xk - Y_ref)
    
    # 约束
    g = np.array([[1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1]])
    G_total = scipy.linalg.block_diag(g, g, g)
    h = np.array([[30, 30]*3*Nc]).T  # 简化约束
    
    # 求解QP
    m = 3
    U = cp.Variable(m * Nc)
    objective = cp.Minimize(0.5 * cp.quad_form(U, H) + f.T @ U)
    constraints = [G_total @ U <= h]
    prob = cp.Problem(objective, constraints)
    prob.solve(solver=cp.OSQP, verbose=False)
    
    # 更新状态并记录
    if prob.status == cp.OPTIMAL:
        U_opt = U.value
        xk_new = A @ xk + B @ U_opt[:3].reshape(-1, 1) + random.uniform(-2.0, 2.0) # 加入噪声，强调xk为实际观测值，具有结果反馈
        y_new = C @ xk_new 
        x_history.append(xk_new)
        y_history.append(y_new)
        
        print(f'Step {j}: 控制量 {U_opt[:3].round(2)}')
    else:
        raise ValueError("Optimization failed")


# 轨迹可视化
plt.figure(figsize=(10, 5))
plt.subplot(2, 1, 1)
for i in range(3):
    if i == 0:
        plt.plot([x[i][0] for x in x_history], '--', linewidth=4, label=f'x{i+1}')
    else:
        plt.plot([x[i][0] for x in x_history], linewidth=3, label=f'x{i+1}')
        
plt.title('状态变量变化')
plt.legend()

plt.subplot(2, 1, 2)
ref_traj = []
for t in range(len(y_history)):
    if t < update_circle: ref = Y_ref_list[0]
    elif t < update_circle*2: ref = Y_ref_list[1]
    else: ref = Y_ref_list[2]
    ref_traj.append(ref)
    
for i in range(3):
    if i == 0:
        plt.plot([y[i][0] for y in y_history], '--', linewidth=4, label=f'y{i+1}')
    else:
        plt.plot([y[i][0] for y in y_history], linewidth=3, label=f'y{i+1}')
        
plt.step(range(len(y_history)), ref_traj, 'k--', label='参考')
plt.title('输出跟踪效果')
plt.legend()
plt.ylim(-40, 60)
plt.tight_layout()
print('总耗时:', round(time.time()-start, 2), '秒')
plt.show()
