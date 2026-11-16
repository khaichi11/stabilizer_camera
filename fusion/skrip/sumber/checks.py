

# ---------------------------------------------------------------- assembly poses + checks
def at_pose(body, theta, phi=0.0):
    b = cp(body)
    if abs(phi) > 1e-9:
        xf(b, mrot((0, 1, 0), phi))
    return xf(b, mrot((1, 0, 0), -theta), mtr(0, P['Yg'], P['Zg']))


def build_parts():
    K = derived()
    S = {}
    S['G_Base'] = build_base(K)
    S['G_PitchFrame'] = build_pitch_frame(K)
    S['G_CamTray'] = build_tray(K)
    S['G_ThumbScrew'] = build_thumbscrew(K)
    S['G_ClampScrew'] = build_clamp_screw(K)
    S['G_ClampJaw'] = build_clamp_jaw(K)
    S['G_CableScrew'] = build_cable_screw(K)
    S['G_PivotCap'] = build_pivot_cap(K)
    S['G_ImuLid'] = build_imu_lid(K)
    S['G_BeltScrew'] = build_belt_screw(K, P['bs_x'][1])
    S['G_BeltPad'] = build_belt_pad(K, 0.0)          # print copy (2x); placed copies are belt_pad_R / _L
    S['G_Nut_M6'] = build_clamp_nut(K)
    S['G_EspCover'] = build_esp_cover(K)
    S['G_UsbGate'] = build_usb_gate(K)
    Dm = {}
    Dm['clamp_nut'] = build_clamp_nut(K)
    Dm['esp32c3'] = build_esp_dummy(K)
    Dm['capacitor'], Dm['power_plug'], _ = build_power_dummies(K)
    Dm['belt_screw_L'] = build_belt_screw(K, P['bs_x'][0])
    Dm['belt_pad_L'] = build_belt_pad(K, P['bs_x'][0])
    Dm['belt_pad_R'] = build_belt_pad(K, P['bs_x'][1])
    Dm['wires_out'] = build_wires_out(K)
    cov = None
    for dz in (3.0, 6.0, 9.0, 12.0, 15.0, 18.0, 21.0, 24.0, 27.0, 30.0):
        b = xf(cp(S['G_EspCover']), mtr(0, 0, dz))
        cov = b if cov is None else U(cov, b)
    Dm['cover_slide'] = cov
    cap = None
    for dx in (1.0, 2.0, 3.0, 4.0, 5.0, 6.0):
        b = xf(cp(S['G_PivotCap']), mtr(dx, 0, 0))
        cap = b if cap is None else U(cap, b)
    Dm['cap_slide'] = cap
    Dm['pitch_servo'] = pitch_servo_B(K)
    Dm['roll_servo'] = roll_servo_L(K)
    Dm['pitch_horn'] = horn_pitch_L(K)
    Dm['roll_horn'] = horn_roll_L(K)
    Dm['camera'], Dm['usb_plug'] = build_camera(K)
    Dm['imu'] = build_imu(K)
    Dm['fov'] = build_fov(K)
    tcx = P['tie_x'][1]                                    # a cable tie (1.3 thick) lying in the right tie channel
    ts = ring(K['Rf'] + 0.1, K['Rf'] + 1.4, P['esp_bot'] - 0.5, P['esp_top'] + 0.5, K)
    I(ts, box(tcx - 2.4, tcx + 2.4, -80, 40, P['esp_bot'] - 0.5, P['esp_top'] + 0.5))
    Dm['tie_strip_R'] = ts
    th0 = P['tilt_nom']
    pfs = U(at_pose(S['G_PitchFrame'], th0), at_pose(Dm['pitch_horn'], th0), at_pose(Dm['roll_servo'], th0))
    xf(pfs, mtr(3.5, 0, 0))
    drop = cp(pfs)
    for dz in (4.0, 8.0, 12.0, 16.0, 20.0, 24.0, 28.0, 32.0, 36.0, 40.0):
        U(drop, xf(cp(pfs), mtr(0, 0, dz)))
    Dm['pf_drop'] = drop
    return K, S, Dm


STATIC = ('G_Base', 'pitch_servo', 'esp32c3', 'G_CableScrew', 'capacitor', 'power_plug', 'G_EspCover', 'G_UsbGate', 'G_PivotCap',
          'G_BeltScrew', 'belt_pad_R', 'belt_screw_L', 'belt_pad_L')
PITCHING = ('G_PitchFrame', 'roll_servo', 'pitch_horn')
ROLLING = ('G_CamTray', 'G_ThumbScrew', 'G_ClampScrew', 'G_ClampJaw', 'clamp_nut', 'camera', 'usb_plug', 'imu', 'roll_horn', 'G_ImuLid')
INTENDED = {frozenset(('pitch_servo', 'pitch_horn')), frozenset(('roll_servo', 'roll_horn')), frozenset(('G_CableScrew', 'G_Base')),
            frozenset(('G_BeltScrew', 'G_Base')), frozenset(('belt_screw_L', 'G_Base')), frozenset(('G_ClampScrew', 'clamp_nut'))}


def run_checks(log):
    t0 = time.time()
    K, S, Dm = build_parts()
    A = dict(S)
    A.update(Dm)
    log.append('BUILD ok ({:.1f} s); volumes: '.format(time.time() - t0) +
               ', '.join('{} {:.1f} cm3'.format(n, vol(S[n]) / 1000) for n in S))

    def fmt(v):
        return 'CLEAR' if v < 0.05 else '{:.1f} mm3'.format(v)

    # ---- fits in the local frames (nothing moves)
    log.append('FITS (parts in place, nominal):')
    fits = [('base vs pitch servo', 'G_Base', 'pitch_servo'),
            ('pitch frame vs roll servo', 'G_PitchFrame', 'roll_servo'),
            ('pitch frame vs pitch horn', 'G_PitchFrame', 'pitch_horn'),
            ('tray vs roll horn', 'G_CamTray', 'roll_horn'),
            ('tray vs camera', 'G_CamTray', 'camera'),
            ('tray vs thumbscrew', 'G_CamTray', 'G_ThumbScrew'),
            ('camera vs thumbscrew (thread in tripod hole)', 'camera', 'G_ThumbScrew'),
            ('tray vs IMU board', 'G_CamTray', 'imu'),
            ('roll servo vs tray', 'roll_servo', 'G_CamTray'),
            ('roll servo vs camera', 'roll_servo', 'camera'),
            ('pitch frame vs tray', 'G_PitchFrame', 'G_CamTray'),
            ('pitch frame vs camera', 'G_PitchFrame', 'camera'),
            ('tray vs clamp screw', 'G_CamTray', 'G_ClampScrew'),
            ('tray vs clamp nut', 'G_CamTray', 'clamp_nut'),
            ('camera vs clamp screw', 'camera', 'G_ClampScrew'),
            ('tray vs clamp jaw', 'G_CamTray', 'G_ClampJaw'),
            ('camera vs clamp jaw (0.2 mm before tightening)', 'camera', 'G_ClampJaw'),
            ('clamp screw vs clamp jaw', 'G_ClampScrew', 'G_ClampJaw'),
            ('USB-C plug vs tray (right end stop)', 'usb_plug', 'G_CamTray'),
            ('base vs ESP32-C3 board', 'G_Base', 'esp32c3'),
            ('base vs capacitor 470uF (6.3 x 11)', 'G_Base', 'capacitor'),
            ('ESP32 vs capacitor', 'esp32c3', 'capacitor'),
            ('base vs power plug head (7 x 12)', 'G_Base', 'power_plug'),
            ('ESP32 vs power plug head', 'esp32c3', 'power_plug'),
            ('plug-lock screw vs plug head (0.2 before tightening)', 'G_CableScrew', 'power_plug'),
            ('base vs ESP front cover + hood', 'G_Base', 'G_EspCover'),
            ('ESP32 vs front cover', 'esp32c3', 'G_EspCover'),
            ('capacitor vs front cover', 'capacitor', 'G_EspCover'),
            ('front cover vs wedge gate', 'G_EspCover', 'G_UsbGate'),
            ('base vs wedge gate', 'G_Base', 'G_UsbGate'),
            ('base vs pivot cap (dovetail)', 'G_Base', 'G_PivotCap'),
            ('pivot cap vs ESP cover hood', 'G_PivotCap', 'G_EspCover'),
            ('tray vs IMU lid', 'G_CamTray', 'G_ImuLid'),
            ('IMU board vs lid', 'imu', 'G_ImuLid'),
            ('IMU board vs tray (pegs in the M3 holes)', 'imu', 'G_CamTray'),
            ('base vs belt pad R (in its recess)', 'G_Base', 'belt_pad_R'),
            ('base vs belt pad L', 'G_Base', 'belt_pad_L'),
            ('belt screw R vs pad', 'G_BeltScrew', 'belt_pad_R'),
            ('belt screw L vs pad', 'belt_screw_L', 'belt_pad_L'),
            ('belt screw R thread in the base (intended)', 'G_BeltScrew', 'G_Base'),
            ('belt screw L thread in the base (intended)', 'belt_screw_L', 'G_Base'),
            ('belt screw R knob vs ESP cover hood', 'G_BeltScrew', 'G_EspCover'),
            ('belt screw R knob vs pivot cap', 'G_BeltScrew', 'G_PivotCap'),
            ('ASSEMBLY: ESP cover slid down 3..30 mm vs wires from the pivot', 'cover_slide', 'wires_out'),
            ('ASSEMBLY: pivot cap slid in 1..6 mm vs base', 'cap_slide', 'G_Base'),
            ('ASSEMBLY: pivot cap slid in 1..6 mm vs wires', 'cap_slide', 'wires_out'),
            ('ASSEMBLY: pitch frame lowered from above (3.5 mm right) vs base', 'pf_drop', 'G_Base'),
            ('ASSEMBLY: same vs pitch servo', 'pf_drop', 'pitch_servo'),
            ('plug-lock screw thread in the base (overlap = thread, intended)', 'G_CableScrew', 'G_Base'),
            ('right cable tie in its channel vs ESP32', 'tie_strip_R', 'esp32c3'),
            ('right cable tie vs plug head', 'tie_strip_R', 'power_plug'),
            ('right cable tie vs base (channel must be free)', 'tie_strip_R', 'G_Base'),]
    for lab, a, b in fits:
        log.append('  {:<46s}: {}'.format(lab, fmt(ivol(A[a], A[b]))))

    # ---- motion: pitch x roll
    th_list = tuple(float(t) for t in range(int(P['tilt_min']), int(P['tilt_max']) + 1, 5))
    ph_list = (-P['roll_max'], -P['roll_max'] / 2, 0.0, P['roll_max'] / 2, P['roll_max'])
    worst = {}
    log.append('MOTION (pitch {} deg, roll {} deg):'.format(th_list, ph_list))
    base_st = {n: A[n] for n in STATIC}
    for th in th_list:
        pit = {n: at_pose(A[n], th) for n in PITCHING}
        for ph in ph_list:
            rol = {n: at_pose(A[n], th, ph) for n in ROLLING}
            fov = at_pose(A['fov'], th, ph)
            pairs = []
            for a in base_st:
                for b in list(pit) + list(rol):
                    if ph == 0.0 or b in rol:
                        pairs.append((a, base_st[a], b, pit.get(b) or rol.get(b)))
            if th == P['tilt_nom']:
                for a in pit:
                    for b in rol:
                        pairs.append((a, pit[a], b, rol[b]))
            for a, ba, b, bb in pairs:
                if frozenset((a, b)) in INTENDED:
                    continue
                if a in ('pitch_servo',) and b in ('pitch_servo',):
                    continue
                v = ivol(ba, bb)
                key = a + ' / ' + b
                if v >= 0.05:
                    x = cp(ba)
                    TB.booleanOperation(x, cp(bb), INTER)
                    q = x.boundingBox
                    worst.setdefault(key, []).append((th, ph, v, q.minPoint.x * 10, q.maxPoint.x * 10, q.minPoint.y * 10,
                                                      q.maxPoint.y * 10, q.minPoint.z * 10, q.maxPoint.z * 10))
            for nm, bd in list(base_st.items()) + list(pit.items()) + list(rol.items()):
                if nm == 'camera':
                    continue
                v = ivol(fov, bd)
                if v >= 0.05:
                    worst.setdefault('FOV / ' + nm, []).append((th, ph, v, 0, 0, 0, 0, 0, 0))
    if not worst:
        log.append('  all pairs CLEAR at every pose, depth FOV (87x58 deg) clear at every pose')
    for k, lst in sorted(worst.items()):
        log.append('  {:<40s}: '.format(k) + '; '.join(
            'p{:.0f}/r{:.0f}: {:.1f} mm3 @X[{:.0f},{:.0f}] Y[{:.0f},{:.0f}] Z[{:.0f},{:.0f}]'.format(*t) for t in lst))

    # ---- cable paths (nominal pose): every cable must run free of every part, except its own ends / clamp
    th = P['tilt_nom']
    parts = {n: A[n] for n in ('G_Base', 'G_EspCover', 'G_PivotCap', 'G_CableScrew', 'G_BeltScrew', 'belt_screw_L',
                               'belt_pad_R', 'belt_pad_L', 'pitch_servo', 'esp32c3', 'capacitor', 'power_plug', 'G_UsbGate')}
    for n in ('G_PitchFrame', 'roll_servo', 'pitch_horn'):
        parts[n] = at_pose(A[n], th)
    for n in ('G_CamTray', 'imu', 'G_ImuLid', 'camera', 'usb_plug', 'roll_horn', 'G_ThumbScrew', 'G_ClampScrew', 'G_ClampJaw', 'clamp_nut'):
        parts[n] = at_pose(A[n], th, 0.0)
    ends = {('Kabel servo nunduk', 'pitch_servo'), ('Kabel servo nunduk', 'esp32c3'), ('Kabel servo kiri-kanan', 'roll_servo'),
            ('Kabel IMU', 'imu'), ('Kabel USB kamera', 'usb_plug'), ('Kabel USB kamera', 'G_UsbGate'),
            ('Kabel power (power bank)', 'power_plug')}
    log.append('CABLE PATHS (nominal pose, every part):')
    for cn, (cb, col) in build_cable_dummies(K, th).items():
        hits = ['{} {:.1f} mm3'.format(pn, v) for pn, pb in parts.items()
                for v in [ivol(cb, pb)] if v > 0.3 and (cn, pn) not in ends]
        log.append('  {:<40s}: {}'.format(cn, 'CLEAR' if not hits else 'BLOCKED by ' + ', '.join(hits)))

    # ---- protrusion / sizes
    nom = [at_pose(A[n], P['tilt_nom']) for n in ('camera', 'G_CamTray', 'G_PitchFrame')]
    ymax = max(b.boundingBox.maxPoint.y for b in nom) * 10
    zmin = min(b.boundingBox.minPoint.z for b in nom) * 10
    bb = A['G_Base'].boundingBox
    log.append('SIZE: front of camera {:.1f} mm in front of the plate face ({:.1f} mm from the body); '
               'base {:.0f} x {:.0f} mm; lowest point Z {:.0f} (plate bottom {:.0f})'.format(
                   ymax, ymax + K['Tp'], (bb.maxPoint.x - bb.minPoint.x) * 10, (bb.maxPoint.z - bb.minPoint.z) * 10,
                   zmin, P['plate_z0']))
    log.append('SIZE: unit height Z {:.0f} .. {:.0f} mm'.format(zmin, P['plate_z1']))
    cg = clamp_geom(K)
    log.append('CLEARANCES (design values, mm): camera back 0.3 | jaw 0.2 before tightening | right stop 0.2, only 2.5 high | '
               'front of camera free | tripod slot +-2.0 front/back, +-0.3 sideways | thumbscrew {:.1f} into the camera thread | '
               'servo pocket {:.1f} x {:.1f} for MG90S max 12.4 x 22.8 | IMU pocket {:.1f} x {:.1f} for GY-521 ~21 x 16 | '
               'ESP32 box 18.5 x 25.3 for 18 x 22.5, plug opening 14 x 8, box front 7 mm in front of the plate'.format(
                   P['screw_engage'], 2 * (P['s_hw'] + P['sc']), P['s_xp'] - P['s_xn'] + 2 * P['sc'],
                   P['imu_x1'] - P['imu_x0'], P['imu_u1'] - P['imu_u0']))
    log.append('CHECK TIME {:.1f} s'.format(time.time() - t0))
    return K, S, Dm


def run(_context):
    log = []
    run_checks(log)
    print('\n'.join(log))
