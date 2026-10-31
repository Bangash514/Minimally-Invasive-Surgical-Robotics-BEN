# Chapter 6 Notes — The Jacobian

**Date:** May 11, 2026

## Key Formula
ẋ = J(q) · q̇

## Physical meaning
J maps joint velocities → end-effector velocity.
Each column = how that joint contributes to tip motion.

## Three uses
1. Velocity control: q̇ = J⁻¹ · ẋ
2. Force mapping: τ = Jᵀ · F  (virtual work principle)
3. Singularity detection: det(J) → 0

## 2-Link Jacobian
J = [−L₁s₁ − L₂s₁₂    −L₂s₁₂]
    [ L₁c₁ + L₂c₁₂     L₂c₁₂]

det(J) = L₁L₂sin(θ₂)
Singular at θ₂ = 0° or 180°

## Damped Least Squares (near singularity)
q̇ = Jᵀ(JJᵀ + λ²I)⁻¹ · ẋ

## Self-Check: 3/3 ✓
## Practice Problems: 3/3 ✓
P6.1: θ₁=45°, θ₂=90° → det(J)=1, NOT singular ✓
P6.2: θ₂=0° → arm straight, singular, loses lateral DOF ✓
P6.3: q̇ = [0, -0.7071] rad/s ✓
