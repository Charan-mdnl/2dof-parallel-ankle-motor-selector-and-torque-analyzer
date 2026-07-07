#!/usr/bin/env python3
"""
2-DOF Ankle Joint Torque Solver (Python) — Orthogonal Layout Version
====================================================================
Extends the verified Mech class to support orthogonal actuator layout.
In this layout, Motor 1 is tilted by 90° in yaw (sweeping in the YZ plane),
while Motor 2 remains in the standard sagittal-facing plane (sweeping in XZ).
"""
import math

DEG = math.pi / 180.0
RAD = 180.0 / math.pi

# ═══════════════════════════════════════════════════
# 3D MATH
# ═══════════════════════════════════════════════════
class Vec3:
    __slots__ = ('x','y','z')
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x, self.y, self.z = float(x), float(y), float(z)
    def __add__(s, o): return Vec3(s.x+o.x, s.y+o.y, s.z+o.z)
    def __sub__(s, o): return Vec3(s.x-o.x, s.y-o.y, s.z-o.z)
    def __mul__(s, f): return Vec3(s.x*f, s.y*f, s.z*f)
    def dot(s, o): return s.x*o.x + s.y*o.y + s.z*o.z
    def lsq(s): return s.dot(s)
    def __repr__(s): return f"({s.x:.4f}, {s.y:.4f}, {s.z:.4f})"

def Rx(a):
    c, s = math.cos(a), math.sin(a)
    return [[1,0,0],[0,c,-s],[0,s,c]]

def Ry(a):
    c, s = math.cos(a), math.sin(a)
    return [[c,0,s],[0,1,0],[-s,0,c]]

def mm(A, B):
    R = [[0]*3 for _ in range(3)]
    for i in range(3):
        for j in range(3):
            for k in range(3):
                R[i][j] += A[i][k] * B[k][j]
    return R

def mv(M, v):
    return Vec3(
        M[0][0]*v.x + M[0][1]*v.y + M[0][2]*v.z,
        M[1][0]*v.x + M[1][1]*v.y + M[1][2]*v.z,
        M[2][0]*v.x + M[2][1]*v.y + M[2][2]*v.z
    )

# ═══════════════════════════════════════════════════
# MECHANISM MODEL WITH ORTHOGONAL CONFIGURATION SUPPORT
# ═══════════════════════════════════════════════════
class Mech:
    def __init__(self, p):
        self.set(p)

    def set(self, p):
        self.m1z = p['m1z']; self.m2z = p['m2z']
        self.r1 = p['r1'];   self.r2 = p['r2']
        self.L1 = p['L1'];   self.L2 = p['L2']
        self.E = p['E'];     self.ujZ = p['ujZ']; self.ah = p['ah']
        self.oy = p.get('oy', 0.0)
        self.orthoYaw = p.get('orthoYaw', False)

        # Set attachment positions based on orthogonal layout toggle
        if self.orthoYaw:
            self.a1b = Vec3(-self.oy, p['E']/2, p['ah'])   # Orthogonal: rotated 90° in yaw
        else:
            self.a1b = Vec3(p['E']/2, self.oy, p['ah'])    # Standard parallel
        
        self.a2b = Vec3(-p['E']/2, self.oy, p['ah'])       # Left is always standard
        self.ac = Vec3(0, 0, p['ujZ'])

    def rot(self, p, r):
        return mm(Rx(p), Ry(r))

    def ankW(self, bp, p, r):
        return mv(self.rot(p, r), bp) + self.ac

    def ct1(self, th):
        if self.orthoYaw:
            # Sweeps in YZ plane at X = -oy
            return Vec3(-self.oy, self.r1 * math.sin(th), self.m1z + self.r1 * math.cos(th))
        else:
            # Sweeps in XZ plane at Y = oy
            return Vec3(self.r1 * math.sin(th), self.oy, self.m1z + self.r1 * math.cos(th))

    def ct2(self, th):
        # Sweeps in XZ plane at Y = oy
        return Vec3(-self.r2 * math.sin(th), self.oy, self.m2z + self.r2 * math.cos(th))

    def _solve(self, mz, r, L, aw, xDir, branch=1):
        dx, dy, dz = -aw.x, self.oy - aw.y, mz - aw.z
        a = 2 * xDir * r * dx
        b = 2 * r * dz
        c = L*L - r*r - dx*dx - dy*dy - dz*dz
        R = math.sqrt(a*a + b*b)
        if R < 1e-12: return 0.0, False
        ratio = c / R
        if abs(ratio) > 1: return 0.0, False
        psi = math.atan2(a, b)
        delta = math.acos(max(-1.0, min(1.0, ratio)))
        th = (psi - delta) if branch == 0 else (psi + delta)
        return th, True

    def ik(self, pitch, roll, b1=1, b2=1):
        a1 = self.ankW(self.a1b, pitch, roll)
        a2 = self.ankW(self.a2b, pitch, roll)

        if self.orthoYaw:
            # Solve Motor 1 in the YZ plane (sweeps along Y, constant along X)
            dx = -self.oy - a1.x
            dy = -a1.y
            dz = self.m1z - a1.z
            a = -2.0 * self.r1 * a1.y
            b = 2.0 * self.r1 * dz
            c = self.L1*self.L1 - self.r1*self.r1 - dy*dy - dz*dz - dx*dx
            
            R = math.sqrt(a*a + b*b)
            if R < 1e-12:
                t1, ok1 = 0.0, False
            else:
                ratio = c / R
                if abs(ratio) > 1:
                    t1, ok1 = 0.0, False
                else:
                    psi = math.atan2(a, b)
                    delta = math.acos(max(-1.0, min(1.0, ratio)))
                    t1 = (psi - delta) if b1 == 0 else (psi + delta)
                    ok1 = True
        else:
            t1, ok1 = self._solve(self.m1z, self.r1, self.L1, a1, +1, b1)

        t2, ok2 = self._solve(self.m2z, self.r2, self.L2, a2, -1, b2)
        return t1, t2, ok1 and ok2

    def jacobian(self, t1, t2, p, r):
        cp, sp = math.cos(p), math.sin(p)
        cr, sr = math.cos(r), math.sin(r)

        tip1, tip2 = self.ct1(t1), self.ct2(t2)
        a1, a2 = self.ankW(self.a1b, p, r), self.ankW(self.a2b, p, r)

        if self.orthoYaw:
            # Velocity tangent sweeps in the YZ plane
            dc1_dt1 = Vec3(0.0, self.r1 * math.cos(t1), -self.r1 * math.sin(t1))
        else:
            dc1_dt1 = Vec3(self.r1 * math.cos(t1), 0.0, -self.r1 * math.sin(t1))
            
        dc2_dt2 = Vec3(-self.r2 * math.cos(t2), 0.0, -self.r2 * math.sin(t2))

        den1 = (tip1 - a1).dot(dc1_dt1)
        den2 = (tip2 - a2).dot(dc2_dt2)

        def get_da_dp(bp):
            return Vec3(
                0.0,
                (cp*sr)*bp.x + (-sp)*bp.y + (-cp*cr)*bp.z,
                (sp*sr)*bp.x + (cp)*bp.y + (-sp*cr)*bp.z
            )

        def get_da_dr(bp):
            return Vec3(
                -sr*bp.x + cr*bp.z,
                (sp*cr)*bp.x + (sp*sr)*bp.z,
                (-cp*cr)*bp.x + (-cp*sr)*bp.z
            )

        da1_dp = get_da_dp(self.a1b)
        da1_dr = get_da_dr(self.a1b)
        da2_dp = get_da_dp(self.a2b)
        da2_dr = get_da_dr(self.a2b)

        J = [[0.0, 0.0], [0.0, 0.0]]
        det = 0.0
        cn = float('inf')

        if abs(den1) > 1e-12 and abs(den2) > 1e-12:
            J[0][0] = (tip1 - a1).dot(da1_dp) / den1
            J[0][1] = (tip1 - a1).dot(da1_dr) / den1
            J[1][0] = (tip2 - a2).dot(da2_dp) / den2
            J[1][1] = (tip2 - a2).dot(da2_dr) / den2

            det = J[0][0]*J[1][1] - J[0][1]*J[1][0]
            nJ = math.sqrt(sum(J[i][j]**2 for i in range(2) for j in range(2)))
            if abs(det) > 1e-12:
                iJ = [[J[1][1]/det, -J[0][1]/det], [-J[1][0]/det, J[0][0]/det]]
                cn = nJ * math.sqrt(sum(iJ[i][j]**2 for i in range(2) for j in range(2)))

        return J, det, cn

    def solve_torques(self, pitch, roll, tp, tr):
        t1, t2, ok = self.ik(pitch, roll)
        if not ok:
            return None
        J, det, cn = self.jacobian(t1, t2, pitch, roll)
        if abs(det) < 1e-10:
            return None
        # τ_motor = (Jᵀ)⁻¹ · τ_ankle
        m1 = ( J[1][1]*tp - J[1][0]*tr) / det
        m2 = (-J[0][1]*tp + J[0][0]*tr) / det
        return {
            'm1': m1, 'm2': m2,
            't1': t1, 't2': t2,
            'J': J, 'det': det, 'cn': cn
        }


# ═══════════════════════════════════════════════════
# ROBSTRIDE MOTOR DATABASE
# ═══════════════════════════════════════════════════
MOTORS = {
    'RS00': {'rated': 5,  'peak': 14},
    'RS02': {'rated': 6,  'peak': 17},
    'RS03': {'rated': 20, 'peak': 60},
    'RS04': {'rated': 40, 'peak': 120},
}


if __name__ == '__main__':
    # Test standard
    params = {'m1z':23.5, 'm2z':16.0, 'r1':2.5, 'r2':2.5, 'L1':20.7, 'L2':12.7, 'E':2.3, 'ujZ':3.6, 'ah': 0.0, 'oy':3.8, 'orthoYaw': False}
    mech = Mech(params)
    t1, t2, ok = mech.ik(0, 0)
    print(f"Standard Neutral: ok={ok}, t1={t1*RAD:.1f}°, t2={t2*RAD:.1f}°")

    # Test orthogonal
    params_ortho = {'m1z':23.5, 'm2z':16.0, 'r1':2.5, 'r2':2.5, 'L1':20.7, 'L2':12.7, 'E':2.3, 'ujZ':3.6, 'ah': 0.0, 'oy':3.8, 'orthoYaw': True}
    mech_ortho = Mech(params_ortho)
    t1_o, t2_o, ok_o = mech_ortho.ik(0, 0)
    print(f"Orthogonal Neutral: ok={ok_o}, t1={t1_o*RAD:.1f}°, t2={t2_o*RAD:.1f}°")
