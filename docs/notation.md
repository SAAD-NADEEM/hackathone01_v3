---
title: Notation
description: Mathematical and technical notation used throughout the Physical AI and Humanoid Robotics educational resource
keywords: [notation, mathematical symbols, robotics, AI, equations]
sidebar_position: 2
---

# Notation

This document defines the mathematical and technical notation used consistently throughout the Physical AI and Humanoid Robotics educational resource.

## Mathematical Operators

### Scalars, Vectors, and Matrices
- s - Scalar quantity
- **v** - Vector quantity (bold lowercase)
- **M** - Matrix quantity (bold uppercase)
- ℝ - Set of real numbers
- ℝ^n - n-dimensional real vector space
- I_n - Identity matrix of size n×n

### Calculus
- d/dt - Ordinary derivative with respect to time
- ∂/∂x - Partial derivative with respect to x
- ∇ - Gradient operator
- ∫ - Integral operator
- ∑ - Summation operator

### Set Theory
- ∈ - "is an element of"
- ⊂ - Subset
- ∪ - Union
- ∩ - Intersection
- ∅ - Empty set
- | or : - "such that"

### Logic
- ∧ - Logical AND
- ∨ - Logical OR
- ¬ or ∼ - Logical NOT
- ⇒ - Logical implication
- ⇔ - Logical equivalence

## Robotics-Specific Notation

### Transformations
- ᴮT_A - Transformation matrix from frame A to frame B
- ᴮR_A - Rotation matrix from frame A to frame B
- ᴮp - Position vector of point p expressed in frame A

### Kinematics
- θᵢ - Joint angle for joint i
- **q** - Vector of joint variables [θ₁ θ₂ ... θₙ]ᵀ
- **J** - Jacobian matrix
- **x** - Task space position/orientation vector
- **q̇** - Vector of joint velocities
- **ẋ** - Vector of task space velocities

### Dynamics
- **M(q)** - Inertia matrix
- **C(q, q̇)** - Coriolis and centrifugal force matrix
- **g(q)** - Gravity vector
- **τ** - Vector of joint torques
- **F** - Vector of external forces

### Control Theory
- G(s) - Transfer function in Laplace domain
- ζ - Damping ratio
- ωₙ - Natural frequency
- Kₚ, Kᵢ, Kd - Proportional, integral, and derivative gains
- e(t) - Error signal
- u(t) - Control input signal

## Probability and Statistics
- P(A) - Probability of event A
- p(x) - Probability density function of x
- E[X] - Expected value of random variable X
- Var(X) - Variance of random variable X
- x ~ N(μ, σ²) - x follows a normal distribution with mean μ and variance σ²
- σ - Standard deviation

## AI and Machine Learning
- D - Dataset
- x, y - Input and output variables
- f_θ(x) - Function parameterized by θ
- L - Loss function
- ∇_θ L - Gradient of loss with respect to parameters
- α - Learning rate
- **W** - Weight matrix
- σ(·) - Activation function (often sigmoid)
- ReLU(x) - Rectified Linear Unit activation function max(0, x)

## Computer Vision
- I(x, y) - Intensity value at pixel location (x, y)
- **K** - Camera intrinsic matrix
- **R**, **t** - Camera rotation and translation
- fₓ, fᵧ - Focal lengths in pixels
- (cₓ, cᵧ) - Principal point coordinates

## Time and Discrete Systems
- t - Continuous time
- k - Discrete time step
- xₖ - State at discrete time step k
- z⁻¹ - Unit delay operator in z-transform domain

## Common Abbreviations
- T - Transpose of a matrix or vector
- ⁻¹ - Matrix inverse
- † - Pseudoinverse
- ||·|| - Norm
- ||·||₂ - Euclidean norm
- ||·||_F - Frobenius norm