# Chapter 3 Notes — Rotations

**Date studied:** May 6, 2026
**Author:** BEN (班班)

## Chapter Summary (My Own Words)

A rotation has only 3 degrees of freedom but can be represented three different ways:

### Three Representations

| Method | Numbers | Best Use |
|---|---|---|
| Rotation Matrix | 9 (3x3) | Computing transforms, multiplying |
| Euler Angles | 3 (roll, pitch, yaw) | Human input, surgeon UI |
| Quaternion | 4 (w, x, y, z) | Smooth interpolation, no singularities |

### Key Concepts

- **Rotation matrix:** R^T · R = I and det(R) = +1
- **Composition:** R_total = R_z(yaw) · R_y(pitch) · R_x(roll)
- **Quaternion formula:** q = (cos(theta/2), axis * sin(theta/2))
- **Always use libraries** for conversion (scipy.spatial.transform)

### Gimbal Lock (Important Warning)

- Happens at pitch = 90 degrees
- Roll and yaw become the same axis
- Loses one degree of freedom
- Solution: Don't store rotations as Euler angles internally
- Use quaternions or rotation matrices in code

### Convention Warnings

- scipy quaternion order: [x, y, z, w]
- Eigen (C++) quaternion order: [w, x, y, z]
- Always verify which convention before using!

## Self-Check Answers

**Q15.** Why does a rotation matrix have only 3 DOF despite 9 numbers?
> Because R^T · R = I imposes 6 constraints (orthonormality of columns),
> reducing 9 numbers down to 3 free parameters.

**Q16.** What is gimbal lock and when does it happen?
> When pitch = 90 degrees, roll and yaw axes align and merge into one,
> losing one rotation degree of freedom.

**Q17.** Why are quaternions preferred for interpolation?
> They have no singularities (no gimbal lock), allow smooth SLERP between
> orientations, and stay numerically stable.

**Q18.** What's the danger of Euler angles in code?
> Multiple conventions cause bugs (XYZ vs ZYX), gimbal lock causes
> sudden jumps, and they cannot be smoothly interpolated.

## Practice Problems Solved

**Problem 3.1:** "Rotate 30 deg about Y, then 45 deg about new Z"
- Intrinsic rotation -> post-multiply
- R = R_y(30) * R_z(45)

**Problem 3.2:** Quaternion q = (0.7071, 0, 0.7071, 0)
- cos(theta/2) = 0.7071 -> theta = 90 deg
- Axis = (0, 1, 0) -> Y-axis
- Answer: 90 deg about Y-axis

## Lab Status
- [ ] lab04_rotation_basics.py (to be tested)
