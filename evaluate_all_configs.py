#!/usr/bin/env python3
import math
import os
from ankle_orthogonal_solver import Mech, Vec3, MOTORS

DEG = math.pi / 180.0
RAD = 180.0 / math.pi

def evaluate_motor(motor_name, base_params, E_values, L_values, h_values, layouts):
    motor_spec = MOTORS[motor_name]
    results = {}
    
    # We want to check Rated and Peak limits
    for limit_type in ['rated', 'peak']:
        limit_val = motor_spec[limit_type]
        results[limit_type] = {}
        
        for layout_name, layout_val in layouts:
            results[limit_type][layout_name] = []
            
            for E in E_values:
                for L in L_values:
                    for h in h_values:
                        p = base_params.copy()
                        p['E'] = E
                        p['oy'] = L
                        p['ah'] = h
                        p['orthoYaw'] = layout_val
                        
                        mech = Mech(p)
                        
                        # 1. Pitch sweep (at Roll = 0)
                        min_pitch_cap = float('inf')
                        crit_pitch_ang = 0.0
                        pitch_samples = {}
                        
                        # We sweep in 0.5 deg steps to find the absolute minimum
                        for i in range(101):
                            ang = -25.0 + i * 0.5
                            ik_res = mech.ik(ang * DEG, 0.0)
                            if ik_res[2]:
                                J, det, cn = mech.jacobian(ik_res[0], ik_res[1], ang * DEG, 0.0)
                                if abs(det) > 1e-12 and cn < 250:
                                    cap = (abs(J[0][0]) + abs(J[1][0])) * limit_val
                                    if cap < min_pitch_cap:
                                        min_pitch_cap = cap
                                        crit_pitch_ang = ang
                                        
                        # Record sample points
                        for ang in [-25, -20, -15, -10, -5, 0, 5, 10, 15, 20, 25]:
                            ik_res = mech.ik(ang * DEG, 0.0)
                            cap = 0.0
                            if ik_res[2]:
                                J, det, cn = mech.jacobian(ik_res[0], ik_res[1], ang * DEG, 0.0)
                                if abs(det) > 1e-12 and cn < 250:
                                    cap = (abs(J[0][0]) + abs(J[1][0])) * limit_val
                            pitch_samples[ang] = cap
                            
                        # 2. Roll sweep (at Pitch = 0)
                        min_roll_cap = float('inf')
                        crit_roll_ang = 0.0
                        roll_samples = {}
                        
                        for i in range(161):
                            ang = -40.0 + i * 0.5
                            ik_res = mech.ik(0.0, ang * DEG)
                            if ik_res[2]:
                                J, det, cn = mech.jacobian(ik_res[0], ik_res[1], 0.0, ang * DEG)
                                if abs(det) > 1e-12 and cn < 250:
                                    cap = (abs(J[0][1]) + abs(J[1][1])) * limit_val
                                    if cap < min_roll_cap:
                                        min_roll_cap = cap
                                        crit_roll_ang = ang
                                        
                        # Record sample points
                        for ang in [-40, -30, -20, -10, 0, 10, 20, 30, 40]:
                            ik_res = mech.ik(0.0, ang * DEG)
                            cap = 0.0
                            if ik_res[2]:
                                J, det, cn = mech.jacobian(ik_res[0], ik_res[1], 0.0, ang * DEG)
                                if abs(det) > 1e-12 and cn < 250:
                                    cap = (abs(J[0][1]) + abs(J[1][1])) * limit_val
                            roll_samples[ang] = cap
                            
                        results[limit_type][layout_name].append({
                            'E': E, 'L': L, 'h': h,
                            'min_pitch': min_pitch_cap, 'crit_pitch': crit_pitch_ang,
                            'pitch_samples': pitch_samples,
                            'min_roll': min_roll_cap, 'crit_roll': crit_roll_ang,
                            'roll_samples': roll_samples
                        })
    return results

def main():
    layouts = [
        ('Standard Layout (0° Yaw)', False),
        ('Orthogonal Layout (90° Yaw)', True)
    ]
    
    # 1. RS02: Current Robot dimensions
    rs02_params = {
        'm1z': 27.1, 'm2z': 19.6,
        'r1': 2.5,   'r2': 2.5,
        'L1': 24.3,  'L2': 16.3,
        'ujZ': 3.6,  'ah': 0.0,
        'oy': 3.8
    }
    rs02_E = [2.3, 5.0]
    rs02_L = [3.2, 3.8, 4.4]
    rs02_h = [-1.0, 0.0, 1.0]
    
    # 2. RS03 & RS04: Screenshot dimensions
    sc_params = {
        'm1z': 25.8, # G + H = 3.6 + 22.2 = 25.8
        'm2z': 14.8, # G + D = 3.6 + 11.2 = 14.8
        'r1': 2.5,   'r2': 2.5,
        'L1': 22.2,  'L2': 11.2,
        'ujZ': 3.6,  'ah': 0.0,
        'oy': 4.2
    }
    sc_E = [2.3, 5.0]
    sc_L = [3.6, 4.2, 4.8]
    sc_h = [-1.0, 0.0, 1.0]

    print("Running evaluations for RS02...")
    rs02_res = evaluate_motor('RS02', rs02_params, rs02_E, rs02_L, rs02_h, layouts)
    
    print("Running evaluations for RS03...")
    rs03_res = evaluate_motor('RS03', sc_params, sc_E, sc_L, sc_h, layouts)
    
    print("Running evaluations for RS04...")
    rs04_res = evaluate_motor('RS04', sc_params, sc_E, sc_L, sc_h, layouts)

    # Now write the comprehensive report in markdown
    report = []
    report.append("# Comprehensive Robotic Ankle Sizing and Parameter Sweep Report")
    report.append("\nThis report presents a thorough evaluation of the 2-RSS/U joint actuator torque capability under relative sweeps of **ankle spread (E)**, **motor offset depth (L)**, and **attachment offset (h)**.\n")
    
    # ═══════════════════════════════════════════════════
    # ACTUATOR SECTION: RS02
    # ═══════════════════════════════════════════════════
    report.append("## 1. Actuator Sizing: Robstride RS02 (Current Robot Dimensions)")
    report.append("Evaluation dimensions: $A = B = 2.5\\text{ cm}$, $C = 24.3\\text{ cm}$, $G = 3.6\\text{ cm}$, $H = 23.5\\text{ cm}$, $I = 16.3\\text{ cm}$.")
    
    for limit in ['rated', 'peak']:
        report.append(f"\n### 1.1 RS02 {limit.upper()} Sizing (Limit: {MOTORS['RS02'][limit]:.1f} N·m)")
        
        for lay_name, _ in layouts:
            report.append(f"\n#### 1.1.1 {lay_name}")
            
            # Print sweep table
            report.append("| Spread E (cm) | Y-Offset L (cm) | Attach h (cm) | Min Pitch (N·m) | Crit Pitch | Min Roll (N·m) | Crit Roll |")
            report.append("| --- | --- | --- | --- | --- | --- | --- |")
            
            for item in rs02_res[limit][lay_name]:
                p_str = f"{item['min_pitch']:.2f}" if item['min_pitch'] != float('inf') else "Singular"
                r_str = f"{item['min_roll']:.2f}" if item['min_roll'] != float('inf') else "Singular"
                report.append(f"| {item['E']} | {item['L']} | {item['h']} | {p_str} | {item['crit_pitch']:+.1f}° | {r_str} | {item['crit_roll']:+.1f}° |")
                
            # Print a discrete sample position table for nominal case: L=3.8, h=0.0
            report.append("\n**Discrete Angle Torque Matrix (L=3.8 cm, h=0.0 cm):**")
            
            # Pitch samples
            pitch_nom = next(x for x in rs02_res[limit][lay_name] if abs(x['L']-3.8)<1e-3 and abs(x['h']-0.0)<1e-3 and x['E'] == 2.3)
            pitch_nom_5 = next(x for x in rs02_res[limit][lay_name] if abs(x['L']-3.8)<1e-3 and abs(x['h']-0.0)<1e-3 and x['E'] == 5.0)
            
            report.append("| Pitch Angle (Roll = 0°) | E=2.3 cm Torque (N·m) | E=5.0 cm Torque (N·m) |")
            report.append("| --- | --- | --- |")
            for ang in [-25, -20, -15, -10, -5, 0, 5, 10, 15, 20, 25]:
                cap_2 = f"{pitch_nom['pitch_samples'][ang]:.2f}" if pitch_nom['pitch_samples'][ang] > 0 else "—"
                cap_5 = f"{pitch_nom_5['pitch_samples'][ang]:.2f}" if pitch_nom_5['pitch_samples'][ang] > 0 else "—"
                report.append(f"| {ang:+.1f}° | {cap_2} | {cap_5} |")

            # Roll samples
            report.append("\n| Roll Angle (Pitch = 0°) | E=2.3 cm Torque (N·m) | E=5.0 cm Torque (N·m) |")
            report.append("| --- | --- | --- |")
            for ang in [-40, -30, -20, -10, 0, 10, 20, 30, 40]:
                cap_2 = f"{pitch_nom['roll_samples'][ang]:.2f}" if pitch_nom['roll_samples'][ang] > 0 else "—"
                cap_5 = f"{pitch_nom_5['roll_samples'][ang]:.2f}" if pitch_nom_5['roll_samples'][ang] > 0 else "—"
                report.append(f"| {ang:+.1f}° | {cap_2} | {cap_5} |")
                
    # ═══════════════════════════════════════════════════
    # ACTUATOR SECTION: RS03
    # ═══════════════════════════════════════════════════
    report.append("\n## 2. Actuator Sizing: Robstride RS03 (Screenshot Dimensions)")
    report.append("Evaluation dimensions: $A = B = 2.5\\text{ cm}$, $C = 22.2\\text{ cm}$, $G = 3.6\\text{ cm}$, $H = 22.2\\text{ cm}$, $I = 11.2\\text{ cm}$.")
    
    for limit in ['rated', 'peak']:
        report.append(f"\n### 2.1 RS03 {limit.upper()} Sizing (Limit: {MOTORS['RS03'][limit]:.1f} N·m)")
        
        for lay_name, _ in layouts:
            report.append(f"\n#### 2.1.1 {lay_name}")
            
            report.append("| Spread E (cm) | Y-Offset L (cm) | Attach h (cm) | Min Pitch (N·m) | Crit Pitch | Min Roll (N·m) | Crit Roll |")
            report.append("| --- | --- | --- | --- | --- | --- | --- |")
            
            for item in rs03_res[limit][lay_name]:
                p_str = f"{item['min_pitch']:.2f}" if item['min_pitch'] != float('inf') else "Singular"
                r_str = f"{item['min_roll']:.2f}" if item['min_roll'] != float('inf') else "Singular"
                report.append(f"| {item['E']} | {item['L']} | {item['h']} | {p_str} | {item['crit_pitch']:+.1f}° | {r_str} | {item['crit_roll']:+.1f}° |")
                
            report.append("\n**Discrete Angle Torque Matrix (L=4.2 cm, h=0.0 cm):**")
            
            # Pitch samples
            pitch_nom = next(x for x in rs03_res[limit][lay_name] if abs(x['L']-4.2)<1e-3 and abs(x['h']-0.0)<1e-3 and x['E'] == 2.3)
            pitch_nom_5 = next(x for x in rs03_res[limit][lay_name] if abs(x['L']-4.2)<1e-3 and abs(x['h']-0.0)<1e-3 and x['E'] == 5.0)
            
            report.append("| Pitch Angle (Roll = 0°) | E=2.3 cm Torque (N·m) | E=5.0 cm Torque (N·m) |")
            report.append("| --- | --- | --- |")
            for ang in [-25, -20, -15, -10, -5, 0, 5, 10, 15, 20, 25]:
                cap_2 = f"{pitch_nom['pitch_samples'][ang]:.2f}" if pitch_nom['pitch_samples'][ang] > 0 else "—"
                cap_5 = f"{pitch_nom_5['pitch_samples'][ang]:.2f}" if pitch_nom_5['pitch_samples'][ang] > 0 else "—"
                report.append(f"| {ang:+.1f}° | {cap_2} | {cap_5} |")

            # Roll samples
            report.append("\n| Roll Angle (Pitch = 0°) | E=2.3 cm Torque (N·m) | E=5.0 cm Torque (N·m) |")
            report.append("| --- | --- | --- |")
            for ang in [-40, -30, -20, -10, 0, 10, 20, 30, 40]:
                cap_2 = f"{pitch_nom['roll_samples'][ang]:.2f}" if pitch_nom['roll_samples'][ang] > 0 else "—"
                cap_5 = f"{pitch_nom_5['roll_samples'][ang]:.2f}" if pitch_nom_5['roll_samples'][ang] > 0 else "—"
                report.append(f"| {ang:+.1f}° | {cap_2} | {cap_5} |")

    # ═══════════════════════════════════════════════════
    # ACTUATOR SECTION: RS04
    # ═══════════════════════════════════════════════════
    report.append("\n## 3. Actuator Sizing: Robstride RS04 (Screenshot Dimensions)")
    report.append("Evaluation dimensions: $A = B = 2.5\\text{ cm}$, $C = 22.2\\text{ cm}$, $G = 3.6\\text{ cm}$, $H = 22.2\\text{ cm}$, $I = 11.2\\text{ cm}$.")
    
    for limit in ['rated', 'peak']:
        report.append(f"\n### 3.1 RS04 {limit.upper()} Sizing (Limit: {MOTORS['RS04'][limit]:.1f} N·m)")
        
        for lay_name, _ in layouts:
            report.append(f"\n#### 3.1.1 {lay_name}")
            
            report.append("| Spread E (cm) | Y-Offset L (cm) | Attach h (cm) | Min Pitch (N·m) | Crit Pitch | Min Roll (N·m) | Crit Roll |")
            report.append("| --- | --- | --- | --- | --- | --- | --- |")
            
            for item in rs04_res[limit][lay_name]:
                p_str = f"{item['min_pitch']:.2f}" if item['min_pitch'] != float('inf') else "Singular"
                r_str = f"{item['min_roll']:.2f}" if item['min_roll'] != float('inf') else "Singular"
                report.append(f"| {item['E']} | {item['L']} | {item['h']} | {p_str} | {item['crit_pitch']:+.1f}° | {r_str} | {item['crit_roll']:+.1f}° |")
                
            report.append("\n**Discrete Angle Torque Matrix (L=4.2 cm, h=0.0 cm):**")
            
            # Pitch samples
            pitch_nom = next(x for x in rs04_res[limit][lay_name] if abs(x['L']-4.2)<1e-3 and abs(x['h']-0.0)<1e-3 and x['E'] == 2.3)
            pitch_nom_5 = next(x for x in rs04_res[limit][lay_name] if abs(x['L']-4.2)<1e-3 and abs(x['h']-0.0)<1e-3 and x['E'] == 5.0)
            
            report.append("| Pitch Angle (Roll = 0°) | E=2.3 cm Torque (N·m) | E=5.0 cm Torque (N·m) |")
            report.append("| --- | --- | --- |")
            for ang in [-25, -20, -15, -10, -5, 0, 5, 10, 15, 20, 25]:
                cap_2 = f"{pitch_nom['pitch_samples'][ang]:.2f}" if pitch_nom['pitch_samples'][ang] > 0 else "—"
                cap_5 = f"{pitch_nom_5['pitch_samples'][ang]:.2f}" if pitch_nom_5['pitch_samples'][ang] > 0 else "—"
                report.append(f"| {ang:+.1f}° | {cap_2} | {cap_5} |")

            # Roll samples
            report.append("\n| Roll Angle (Pitch = 0°) | E=2.3 cm Torque (N·m) | E=5.0 cm Torque (N·m) |")
            report.append("| --- | --- | --- |")
            for ang in [-40, -30, -20, -10, 0, 10, 20, 30, 40]:
                cap_2 = f"{pitch_nom['roll_samples'][ang]:.2f}" if pitch_nom['roll_samples'][ang] > 0 else "—"
                cap_5 = f"{pitch_nom_5['roll_samples'][ang]:.2f}" if pitch_nom_5['roll_samples'][ang] > 0 else "—"
                report.append(f"| {ang:+.1f}° | {cap_2} | {cap_5} |")

    # Save Markdown report
    with open("motor_evaluation_report_comprehensive.md", "w") as f:
        f.write("\n".join(report))
        
    print("Comprehensive report generated successfully!")

if __name__ == '__main__':
    main()
