"""
D435 2-AXIS SERVO GIMBAL FOR THE BELT  (Autodesk Fusion script, adsk.core / adsk.fusion only)

Parts (all 3D printed):
  G_Base        belt plate (belt goes THROUGH the sleeve) + left block for the pitch servo + right pivot arm
  G_PitchFrame  pitch frame: holds the roll servo, left side driven by the pitch servo horn, right side on the pivot bolt
  G_CamTray     camera tray: back plate on the roll servo horn, floor with 1/4"-20 thumbscrew, IMU pocket underneath
  G_ThumbScrew  printed 1/4"-20 thumbscrew (tripod thread, centred under the camera)
  G_PivotBolt   printed M6 shoulder bolt for the right pitch pivot
  G_PivotNut    printed M6 nut (sits in the hex pocket inside the pitch frame)
  G_Spacer      C-spacer that fills the assembly gap on the right pivot
  E_Box         electronics box: 1x18650 holder, ESP32 DevKit (USB-C), charge/boost module, switch; belt sleeve
  E_Lid         sliding lid with end cap (snaps in)
Bought parts: 2x MG90S (with horns + screws), MPU6050 (GY-521), ESP32 DevKit USB-C, 1x18650 holder,
5 V >= 2 A boost/charge module, switch, wires.

Frames (mm):
  B (body/static): X = wearer right (USB side), Y = forward (away from the body, plate front face Y = 0 at X = 0), Z = up
  L (moving, built axis-aligned): x = X, y = u = optical axis (forward), z = v = camera up, origin = gimbal centre G
     L -> B :  rotate about x by (-pitch), translate (0, Yg, Zg).  Roll = rotation about u (L y-axis).
"""
import adsk.core, adsk.fusion, math, time, os, traceback

TB = adsk.fusion.TemporaryBRepManager.get()
BT = adsk.fusion.BooleanTypes
UNION, DIFF, INTER = BT.UnionBooleanType, BT.DifferenceBooleanType, BT.IntersectionBooleanType

P = dict(
    cam_w=90.0, cam_h=25.0, cam_d=25.0, tripod_back=12.5, screw_engage=4.5,
    tilt_nom=35.0, tilt_min=10.0, tilt_max=60.0, roll_max=25.0,
    Yg=19.0, Zg=3.0,
    # MG90S (servo frame S: shaft = +z, body bottom z = 0, length along x, shaft at x = 0)
    s_xn=-6.0, s_xp=16.8, s_hw=6.2, s_h=22.7, tab_xn=-10.85, tab_xp=21.65, tab_z0=15.9, tab_z1=18.5,   # MG90S max: 22.8 x 12.4, tabs 32.5
    tab_hole_dx=13.9, s_mid=5.4, boss_r=5.8, boss_top=26.5, spline_r=2.4, spline_top=28.6,
    horn_face=27.7, horn_top=29.5, hub_r=3.6, hub_bot=26.8, horn_len=19.0, horn_w=5.4,
    sc=0.3,                       # servo pocket clearance
    hp_depth=2.0, hp_hub_r=4.2, hp_slot_w=6.6, hp_slot_len=20.5, hp_hole_r=2.6, hp_pilot_r=0.75, hp_pilots=(10.0, 14.0),
    tab_pilot_r=0.8,
    # belt plate
    waist_r=180.0, strap_t=3.0, belt_ch=5.5, front_t=4.5, plate_hw=55.0, plate_z0=-27.0, plate_z1=27.0, plate_rc=4.0,
    tie_x=(-51.0, 66.5), bs_x=(-21.8, 42.6), bs_z=18.0, bs_travel=4.5, bs_boss=3.5, tie_w=5.0, tie_d=1.2, shims=(1.5, 2.5, 3.5),
    esp_x0=47.5, esp_x1=70.0, esp_y0=-9.0, esp_y1=7.8, esp_floor=-8.5, esp_cf=-11.0, esp_roof=14.3, esp_top=16.1, esp_bot=-20.0, plug_lock_z=-15.5,
    esp_cy0=-7.6, esp_cy1=4.8, door_y0=5.2, door_y1=6.8, usb_x=58.75, usb_y=-3.4, plate_xr=72.0,
    bb_hw=47.0, bb_y1=25.5, bb_lid=2.5, bb_post=(38.5, 19.5),
    belt_top=20.5, barb_h=1.8,                      # clip: belt top edge at Z 20, barb below the belt bottom edge (Z -20)
    box_y1=30.0, box_z0=29.0, box_floor=2.5, box_wall=2.5, box_z1=54.0, lid_t=2.5, box_bolt_x=45.5,
    batt_l=79.5, batt_w=22.5,
    clamp_gap=0.2,
    # camera tray
    tray_t=4.0, tray_gap=0.3, d_cam=21.0, floor_t=4.5, tripod_hole_r=3.5,
    imu_x0=15.0, imu_x1=37.0, imu_u0=25.0, imu_u1=42.0, imu_depth=2.8,
    head_r=9.0, head_t=5.0,
    # pivot
    shoulder_r=6.8, arm_hole_r=7.05, arm_r=12.5, brg_r=8.25, trun_r=8.0, trun_bore=5.0, slot_w=4.4, trun_x1=41.4,
    dv_y=1.5, imu_peg_u=39.0, pf_boss_r=9.5, bush_thr_r=6.0, bush_minor_r=5.188,
    bush_bore_r=3.0, bush_head_r=9.0, bolt_r=3.0, nut_af=10.0, nut_t=5.0, nut_pocket_af=10.2, head_af=10.0, head_t2=4.0,
    pivot_gap=4.0,
)


# ---------------------------------------------------------------- primitive helpers (mm in, cm inside)
def Pt(x, y, z):
    return adsk.core.Point3D.create(x / 10.0, y / 10.0, z / 10.0)


def Vc(x, y, z):
    return adsk.core.Vector3D.create(x, y, z)


def box(x0, x1, y0, y1, z0, z1):
    x0, x1 = min(x0, x1), max(x0, x1)
    y0, y1 = min(y0, y1), max(y0, y1)
    z0, z1 = min(z0, z1), max(z0, z1)
    obb = adsk.core.OrientedBoundingBox3D.create(Pt((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), Vc(1, 0, 0), Vc(0, 1, 0),
                                                (x1 - x0) / 10.0, (y1 - y0) / 10.0, (z1 - z0) / 10.0)
    return TB.createBox(obb)


def obox(c, d1, d2, l1, l2, l3):
    obb = adsk.core.OrientedBoundingBox3D.create(Pt(*c), Vc(*d1), Vc(*d2), l1 / 10.0, l2 / 10.0, l3 / 10.0)
    return TB.createBox(obb)


def cyl(a, b, r):
    return TB.createCylinderOrCone(Pt(*a), r / 10.0, Pt(*b), r / 10.0)


def cylx(x0, x1, y, z, r):
    return cyl((x0, y, z), (x1, y, z), r)


def cyly(y0, y1, x, z, r):
    return cyl((x, y0, z), (x, y1, z), r)


def cylz(z0, z1, x, y, r):
    return cyl((x, y, z0), (x, y, z1), r)


def U(a, *bs):
    for b in bs:
        if b is not None:
            if not TB.booleanOperation(a, b, UNION):
                raise RuntimeError('union failed')
    return a


def D(a, *bs):
    for b in bs:
        if b is not None:
            TB.booleanOperation(a, b, DIFF)
    return a


def I(a, b):
    TB.booleanOperation(a, b, INTER)
    return a


def cp(b):
    return TB.copy(b)


def _norm(v):
    l = math.sqrt(sum(c * c for c in v))
    return [c / l for c in v]


def _cross(a, b):
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]


def halfspace(p0, n, size=800.0):
    """big box filling {p : n.(p - p0) > 0}"""
    n = _norm(n)
    a = (1, 0, 0) if abs(n[0]) < 0.9 else (0, 1, 0)
    t1 = _norm(_cross(n, a))
    c = [p0[i] + n[i] * size / 2 for i in range(3)]
    return obox(c, n, t1, size, size, size)


def rrect(axis, a0, a1, b0, b1, c0, c1, r):
    """rounded-rectangle prism. axis = 'x'|'y'|'z' = extrusion axis (c0..c1);
       (a, b) = the other two axes in order x,y,z without the axis."""
    r = min(r, (a1 - a0) / 2 - 0.01, (b1 - b0) / 2 - 0.01)

    def mk(A0, A1, B0, B1):
        if axis == 'x':
            return box(c0, c1, A0, A1, B0, B1)
        if axis == 'y':
            return box(A0, A1, c0, c1, B0, B1)
        return box(A0, A1, B0, B1, c0, c1)

    def cc(A, B):
        if axis == 'x':
            return cylx(c0, c1, A, B, r)
        if axis == 'y':
            return cyly(c0, c1, A, B, r)
        return cylz(c0, c1, A, B, r)
    body = mk(a0 + r, a1 - r, b0, b1)
    U(body, mk(a0, a1, b0 + r, b1 - r))
    for A in (a0 + r, a1 - r):
        for B in (b0 + r, b1 - r):
            U(body, cc(A, B))
    return body


def hexx(x0, x1, y, z, af, a0=0.0):
    """hex prism along x, flats normal to directions a0, a0+60, a0+120 (deg, in the y-z plane, from +y)"""
    xm = (x0 + x1) / 2
    body = None
    for k in range(3):
        a = math.radians(a0 + 60 * k)
        d = (0, math.cos(a), math.sin(a))
        e = (0, -math.sin(a), math.cos(a))
        b = obox((xm, y, z), (1, 0, 0), d, abs(x1 - x0), af, af * 2)
        body = b if body is None else I(body, b)
    return body


def mrot(axis, deg, origin=(0, 0, 0)):
    m = adsk.core.Matrix3D.create()
    m.setToRotation(math.radians(deg), Vc(*axis), Pt(*origin))
    return m


def mtr(dx, dy, dz):
    m = adsk.core.Matrix3D.create()
    m.translation = Vc(dx / 10.0, dy / 10.0, dz / 10.0)
    return m


def xf(body, *mats):
    for m in mats:
        TB.transform(body, m)
    return body


def vol(b):
    try:
        return b.volume * 1000.0
    except Exception:
        return 0.0


def ivol(a, b):
    if not a.boundingBox.intersects(b.boundingBox):
        return 0.0
    x = cp(a)
    ok = TB.booleanOperation(x, cp(b), INTER)
    if not ok:
        return 0.0
    return vol(x)


# ---------------------------------------------------------------- derived geometry
def derived():
    K = {}
    K['u_hf'] = P['d_cam'] - P['tray_gap'] - P['tray_t']          # roll horn face = tray back face (16.7)
    K['u_rb'] = K['u_hf'] - P['horn_face']                         # roll servo body bottom (-11.0)
    K['u_cb'] = P['d_cam']                                         # camera back face (21)
    K['u_cf'] = P['d_cam'] + P['cam_d']                            # camera front face (46)
    K['u_tp'] = K['u_cf'] - P['tripod_back']                       # tripod axis (33.5)
    K['v_cb'] = -P['cam_h'] / 2                                    # camera bottom (-12.5)
    K['tab_u0'] = K['u_rb'] + P['tab_z0']                          # roll servo tabs (4.9 .. 7.4)
    # pitch servo (static): shaft along +X, horn face at the pitch frame left plate outer face
    K['pf_lx0'] = -17.0                                            # pitch frame left plate outer face
    K['pf_lx1'] = -12.0
    K['ps_x0'] = K['pf_lx0'] - P['horn_face']                      # pitch servo bottom X (-44.7)
    K['pf_rx0'], K['pf_rx1'] = 22.5, 31.0                          # pitch frame right plate
    K['arm_x0'] = K['pf_rx1'] + P['pivot_gap']                     # base right arm (35 .. 41)
    K['arm_x1'] = K['arm_x0'] + 6.0
    K['Rb'] = P['waist_r']
    K['Tp'] = P['strap_t'] + P['belt_ch'] + P['front_t']
    K['Rf'] = K['Rb'] + K['Tp']
    K['Rs'] = K['Rb'] + P['strap_t'] + P['belt_ch']                # belt slot front face
    K['Cy'] = -K['Rf']
    return K


def front_y(K, x):
    return K['Cy'] + math.sqrt(K['Rf'] ** 2 - x * x)


# ---------------------------------------------------------------- servo + horn dummies
def servo_S():
    s = box(P['s_xn'], P['s_xp'], -P['s_hw'], P['s_hw'], 0, P['s_h'])
    U(s, box(P['tab_xn'], P['tab_xp'], -P['s_hw'], P['s_hw'], P['tab_z0'], P['tab_z1']))
    U(s, cylz(P['s_h'] - 0.01, P['boss_top'], 0, 0, P['boss_r']))
    U(s, cylz(P['boss_top'] - 0.01, P['spline_top'], 0, 0, P['spline_r']))
    return s


def pitch_servo_B(K):
    s = servo_S()
    return xf(s, mrot((1, 0, 1), 180.0), mtr(K['ps_x0'], P['Yg'], P['Zg']))


def roll_servo_L(K):
    s = servo_S()
    return xf(s, mrot((1, 0, 0), -90.0), mtr(0, K['u_rb'], 0))


def horn_pitch_L(K):
    x0 = K['pf_lx0']
    h = cylx(x0 - (P['horn_face'] - P['hub_bot']), x0 + (P['horn_top'] - P['horn_face']), 0, 0, P['hub_r'])
    U(h, box(x0, x0 + (P['horn_top'] - P['horn_face']), -P['horn_w'] / 2, P['horn_w'] / 2, 0, P['horn_len']))
    return h


def horn_roll_L(K):
    u0 = K['u_hf']
    h = cyly(u0 - (P['horn_face'] - P['hub_bot']), u0 + (P['horn_top'] - P['horn_face']), 0, 0, P['hub_r'])
    U(h, box(-P['horn_w'] / 2, P['horn_w'] / 2, u0, u0 + (P['horn_top'] - P['horn_face']), -P['horn_len'], 0))
    return h


def dovetail(x0, x1, yc, ztop, cl):
    """dovetail prism along x, narrow at ztop (4.0) and wide 2 mm below (5.6); cl = clearance for the tongue"""
    hw_t, hw_b, dep = 2.0 - cl, 2.8 - cl, 2.0 - cl
    b = box(x0, x1, yc - hw_b, yc + hw_b, ztop - dep, ztop + (0.01 if cl > 0 else 1.0))
    sl = (hw_b - hw_t) / dep
    D(b, halfspace((0, yc - hw_t, ztop), (0, -1, sl)))
    D(b, halfspace((0, yc + hw_t, ztop), (0, 1, sl)))
    return b


def horn_pocket_x(x_face, sign, direction):
    """pocket for a single-arm horn in a face normal to x. sign=+1: material is at x > x_face.
       direction = (dy, dz) unit vector of the arm in the face plane."""
    d = P['hp_depth']
    xa, xb = (x_face - 0.1, x_face + d) if sign > 0 else (x_face - d, x_face + 0.1)
    p = cylx(xa, xb, 0, 0, P['hp_hub_r'])
    w, L = P['hp_slot_w'] / 2, P['hp_slot_len']
    dy, dz = direction
    ny, nz = -dz, dy
    c = (0.5 * (xa + xb), dy * L / 2, dz * L / 2)
    U(p, obox(c, (1, 0, 0), (0, dy, dz), xb - xa, L, 2 * w))
    U(p, cylx(xa, xb, dy * L, dz * L, w))
    hole = cylx(x_face - 20 * sign, x_face + 20 * sign, 0, 0, P['hp_hole_r'])
    pil = None
    for r in P['hp_pilots']:
        c2 = cylx(x_face - 0.1 * sign, x_face + 7.0 * sign, dy * r, dz * r, P['hp_pilot_r'])
        pil = c2 if pil is None else U(pil, c2)
    return p, hole, pil


def horn_pocket_y(y_face, sign, direction):
    """same, face normal to y (u). direction = (dx, dz)."""
    d = P['hp_depth']
    ya, yb = (y_face - 0.1, y_face + d) if sign > 0 else (y_face - d, y_face + 0.1)
    p = cyly(ya, yb, 0, 0, P['hp_hub_r'])
    w, L = P['hp_slot_w'] / 2, P['hp_slot_len']
    dx, dz = direction
    c = (dx * L / 2, 0.5 * (ya + yb), dz * L / 2)
    U(p, obox(c, (0, 1, 0), (dx, 0, dz), yb - ya, L, 2 * w))
    U(p, cyly(ya, yb, dx * L, dz * L, w))
    hole = cyly(y_face - 20 * sign, y_face + 20 * sign, 0, 0, P['hp_hole_r'])
    pil = None
    for r in P['hp_pilots']:
        c2 = cyly(y_face - 0.1 * sign, y_face + 3.7 * sign, dx * r, dz * r, P['hp_pilot_r'])   # blind: M2x4 only
        pil = c2 if pil is None else U(pil, c2)
    return p, hole, pil


# ---------------------------------------------------------------- printed parts
def tie_anchor_on_plate(K, x0, z0):
    xs = (x0, x0 + 8.0)
    y_lo = min(front_y(K, xs[0]), front_y(K, xs[1]))
    y_c = front_y(K, x0 + 4.0)
    a = box(x0, x0 + 8.0, y_lo - 3.0, y_c + 3.2, z0, z0 + 6.0)
    D(a, box(x0 + 2.0, x0 + 6.0, y_c - 0.4, y_c + 1.8, z0 - 1, z0 + 7))
    return a


def ring(r0, r1, z0, z1, K):
    b = cylz(z0, z1, 0, K['Cy'], r1)
    D(b, cylz(z0 - 1, z1 + 1, 0, K['Cy'], r0))
    return b


def plate_outline(K, y0=-60.0, y1=40.0):
    return rrect('y', -P['plate_hw'], P['plate_hw'], P['plate_z0'], P['plate_z1'], y0, y1, P['plate_rc'])


def box_bolt_pos(K):
    """bolt axis (x, y) for the power box: centred in the plate band thickness"""
    out = []
    for sx in (-1, 1):
        x = sx * P['box_bolt_x']
        yb = K['Cy'] + math.sqrt(K['Rb'] ** 2 - x * x)
        yf = K['Cy'] + math.sqrt(K['Rf'] ** 2 - x * x)
        out.append((x, 0.5 * (yb + yf)))
    return out


def build_base(K):
    Yg, Zg = P['Yg'], P['Zg']
    Rb, Rf, Cy = K['Rb'], K['Rf'], K['Cy']
    Rt, Rs = Rb + P['strap_t'], Rb + P['strap_t'] + P['belt_ch']
    z0, z1 = P['plate_z0'], P['plate_z1']
    slab = ring(Rb, Rf, z0 - 1, z1 + 1, K)
    I(slab, U(plate_outline(K), rrect('y', 45.0, P['plate_xr'], z0, z1, -60.0, 40.0, P['plate_rc'])))
    # left block: pitch servo (long side up), tabs seat on the +X face
    xa, xb = K['ps_x0'] - 2.5, K['ps_x0'] + P['tab_z0']           # -47.2 .. -28.8
    ytop, zb, zt = Yg + P['s_hw'] + 3.0, Zg + P['tab_xn'] - 3.0, Zg + P['tab_xp'] + 2.5
    lb = box(xa, xb, -7.0, ytop, zb, zt)
    w = box(xa, xb, -7.0, ytop, z0, zb + 0.01)
    D(w, halfspace((0, ytop - 5.0, zb), (0, 1, -1)))      # 45 deg support, 5 mm short overhang at the front edge
    hol = box(xa + 2.0, xb - 2.0, front_y(K, xb) + 2.0, ytop, z0 - 1, zb - 2.0)        # hollow inside (lighter)
    D(hol, halfspace((0, ytop - 5.0 - 2.0 * math.sqrt(2), zb), (0, 1, -1)))
    D(w, hol)
    U(lb, w)
    D(lb, box(xa + 2.0, xb - 2.0, front_y(K, xb) + 2.0, Yg - P['s_hw'] - P['sc'] - 2.0, zb - 2.1, 21.0))
    c = P['sc']
    D(lb, box(K['ps_x0'] - c, xb + 1, Yg - P['s_hw'] - c, Yg + P['s_hw'] + c, Zg + P['s_xn'] - c, Zg + P['s_xp'] + c))
    for dz in (P['s_mid'] - P['tab_hole_dx'], P['s_mid'] + P['tab_hole_dx']):
        D(lb, cylx(xb + 0.1, xb - 8.0, Yg, Zg + dz, P['tab_pilot_r']))
    for za, zc in ((Zg + P['s_xp'], Zg + P['s_xp'] + 3.8), (Zg + P['s_xn'] - 3.8, Zg + P['s_xn'])):
        D(lb, box(K['ps_x0'] - c, K['ps_x0'] + 6.7, Yg - 2.5, ytop + 1, za, zc))
    # right arm: open-top half bearing for the pitch-frame trunnion (wires are laid in from above),
    # closed by G_PivotCap that slides in along a dovetail
    xa2, xb2 = K['arm_x0'], K['arm_x1']
    ar, zt2 = P['arm_r'], Zg + 0.5
    ra = box(xa2, xb2, -7.0, Yg, Zg - ar, zt2)
    cl = cylx(xa2, xb2, Yg, Zg, ar)
    I(cl, box(xa2 - 1, xb2 + 1, Yg - ar - 1, Yg + ar + 1, Zg - ar - 1, zt2))
    U(ra, cl)
    t = (0, Yg + 8.0 * math.cos(math.radians(45)), Zg - 8.0 * math.sin(math.radians(45)))
    w2 = box(xa2, xb2, -7.0, Yg + 8.0, z0, Zg)
    D(w2, halfspace(t, (0, 1, -1)))
    U(ra, w2)
    D(ra, cylx(xa2 - 1, xb2 + 1, Yg, Zg, P['brg_r']))
    D(ra, dovetail(xa2 + 0.5, xb2 + 1.0, P['dv_y'], zt2, 0.0))
    U(slab, lb, ra)
    # belt CLIP: slot open at the bottom, closed at the top (belt top edge stops here), barb under the belt
    bt = P['belt_top']
    slot = ring(Rt, Rs, z0 - 2, bt - 1.2, K)
    co = TB.createCylinderOrCone(Pt(0, Cy, bt - 1.2), Rs / 10.0, Pt(0, Cy, bt), (Rs - 1.2) / 10.0)
    D(co, TB.createCylinderOrCone(Pt(0, Cy, bt - 1.2), Rt / 10.0, Pt(0, Cy, bt), (Rt + 1.2) / 10.0))
    U(slot, co)
    I(slot, box(-100, 100, -80, 40, z0 - 2, bt))
    D(slab, slot)                                  # top corners of the belt slot filled 1.2 x 45 deg (stronger waist)
    barb = ring(Rt - 0.05, Rt + P['barb_h'], -bt - 1.0, -bt, K)
    cone = TB.createCylinderOrCone(Pt(0, Cy, -bt - 1.0 - P['barb_h']), Rt / 10.0, Pt(0, Cy, -bt - 1.0), (Rt + P['barb_h']) / 10.0)
    D(cone, cylz(-bt - 5, -bt + 1, 0, Cy, Rt - 0.05))
    U(barb, cone)
    I(barb, plate_outline(K))
    I(barb, box(-100, 100, -80, 40, -bt - 1.0 - P['barb_h'] - 0.01, -bt))
    U(slab, barb)
    # ESP32-C3 Super Mini box: fused 3 mm deep into the front wall of the clip over its whole width.
    # Board stands upright parallel to the plate (thin box), its own USB-C port facing DOWN.
    ex0, ex1, ey1 = P['esp_x0'], P['esp_x1'], P['esp_y1']
    cf, roof0, ztop = P['esp_cf'], P['esp_roof'], P['esp_top']
    cx0, cx1, cy0, cy1 = ex0 + 2.0, ex1 - 2.0, P['esp_cy0'], P['esp_cy1']
    eb = P['esp_bot']
    ec = rrect('z', ex0, ex1, -30.0, ey1, eb, ztop, 2.0)
    D(ec, cylz(eb - 1, ztop + 1, 0, Cy, Rf - 3.0))
    D(ec, box(cx0, cx1, cy0, cy1, cf, roof0))                                   # cavity
    U(ec, box(cx0, cx0 + 1.2, cy0, cy0 + 3.0, cf + 22.8, roof0 + 0.01))         # stops over the board's top corners
    U(ec, box(cx1 - 1.2, cx1, cy0, cy0 + 3.0, cf + 22.8, roof0 + 0.01))
    D(ec, box(cx0, cx1, cy1 - 0.01, ey1 + 1, cf, ztop + 1))                     # front: board goes in here
    D(ec, box(cx0 - 0.9, cx1 + 0.9, P['door_y0'], P['door_y1'], cf, ztop + 1))   # grooves for the sliding front cover
    U(slab, ec)
    D(slab, box(cx0, cx1, cy0, cy1, cf, cf + 22.79))                             # cavity reaches into the curved front wall
    px, py = P['usb_x'], P['usb_y']
    D(slab, box(px - 7.0, px + 7.0, py - 4.0, py + 4.0, eb - 1, cf + 0.01))          # plug head enters here (14 x 8)
    D(slab, cylx(px + 7.0 - 0.1, ex1 + 0.1, py, P['plug_lock_z'], 4.92 / 2))        # M6 hole: screw presses the plug head
    D(slab, box(ex0 - 0.1, cx0 + 0.1, 1.5, ey1 + 0.1, 0.8, 5.0))                     # wires from the pivot enter here (lay-in)
    D(slab, box(cx0, px + 6.0, py + 5.4, ey1 - 1.6, eb - 1, cf - 1.6))                # lighter: pocket under the box front
    tie_slots(slab, K, z0, z1)
    tcx = P['tie_x'][1]
    tch = ring(Rf - 0.05, Rf + 1.6, P['esp_bot'] - 1, P['esp_top'] + 1, K)          # right tie runs inside the ESP box
    I(tch, box(tcx - 2.7, tcx + 2.7, -80, 40, P['esp_bot'] - 1, P['esp_top'] + 1))
    D(slab, tch)
    # belt clamp: 2 printed thumb screws, turned from the FRONT, push a pad against the belt
    for xs in P['bs_x']:
        yf_ = front_y(K, xs)
        U(slab, cyly(yf_ - 2.0, yf_ + P['bs_boss'], xs, P['bs_z'], 4.75))
        rc = ring(K['Rs'] - 0.1, K['Rs'] + 1.35, 6.0, P['belt_top'] + 0.1, K)
        I(rc, box(xs - 6.7, xs + 6.7, -80, 40, 5.0, P['belt_top'] + 0.2))
        D(slab, rc)
        yr = K['Cy'] + math.sqrt((K['Rs'] + 1.35) ** 2 - xs ** 2)
        D(slab, cyly(yr - 2.0, yr + 1.3, xs, P['bs_z'], 3.3))                  # clean start of the thread
        D(slab, cyly(yr + 1.2, yf_ + P['bs_boss'] + 0.1, xs, P['bs_z'], 4.92 / 2))
    # cable channel on top of the plate (follows the waist arc) + feeder channel over the pitch-servo block
    g0, g1, gd = K['Rb'] + 3.5, K['Rb'] + 8.5, 2.0
    gx0, gx1 = -44.0, 38.0
    ch = ring(g0, g1, z1 - gd, z1 + 1, K)
    I(ch, U(box(gx0, -11.0, -80, 40, z1 - gd, z1 + 1), box(11.0, gx1, -80, 40, z1 - gd, z1 + 1)))
    D(slab, ch)
    chm = ring(g0, g1, z1 - 2.0, z1 + 1, K)                                   # shallow over the tie slots
    I(chm, box(-11.5, 11.5, -80, 40, z1 - 2.0, z1 + 1))
    D(slab, chm)
    pw0, pw1 = P['esp_x0'] + 4.0, P['esp_x0'] + 7.0                     # 51.5 .. 54.5: pitch-servo wire drops here
    ch2 = ring(g0, g1, z1 - 2.0, z1 + 1, K)
    I(ch2, box(gx1 - 1.0, pw1, -80, 40, z1 - 2.0, z1 + 1))
    D(slab, ch2)
    lip = ring(g0, g1, z1 - 0.8, z1, K)
    D(lip, ring(g0 + 0.7, g1 - 0.7, z1 - 2, z1 + 1, K))
    I(lip, box(gx0 + 2.0, pw0 - 0.5, -80, 40, z1 - 1, z1 + 1))
    U(slab, lip)
    nt = ring(g0, Rf + 1.0, z1 - 2.0, z1 + 1, K)
    I(nt, box(pw0, pw1, -80, 40, z1 - 2.0, z1 + 1))
    D(slab, nt)
    gw = box(pw0, pw1, -40.0, P['esp_cy0'] + 0.01, P['esp_roof'] - 3.3, z1 + 1)
    D(gw, cylz(z0 - 1, z1 + 2, 0, Cy, Rf - 2.5))                             # 1.5 mm wall stays in front of the belt
    D(slab, gw)
    D(slab, box(pw0, pw1, P['esp_cy0'] - 0.1, P['esp_cy0'] + 3.2, P['esp_roof'] - 3.3, P['esp_roof']))   # over the board into the box
    xf0 = K['ps_x0'] + 3.2                                   # -41.5, above the upper cable slot of the servo block
    yfa = K['Cy'] + math.sqrt(g0 ** 2 - xf0 ** 2)
    ytop = P['Yg'] + P['s_hw'] + 3.0
    D(slab, box(xf0 - 2.5, xf0 + 2.5, yfa, ytop + 1.0, z1 - gd - 0.6, z1 + 1))
    # over the servo block the feeder goes down until it meets the block's upper cable slot (open path, no wall)
    D(slab, box(xf0 - 2.5, xf0 + 2.5, P['Yg'] - 4.0, ytop + 1.0, P['Zg'] + P['s_xp'] + 3.8 - 0.8, z1 + 1))
    return slab


def build_cable_screw(K):
    cy, cz, x0 = P['usb_y'], P['plug_lock_z'], P['esp_x1'] + 0.5
    hd = cylx(x0, x0 + 4.0, cy, cz, 6.0)
    for i in range(10):
        a = 2 * math.pi * i / 10
        D(hd, cylx(x0 - 1.0, x0 + 5.0, cy + 6.0 * math.cos(a), cz + 6.0 * math.sin(a), 1.1))
    U(hd, cylx(P['usb_x'] + 6.2, x0 + 0.01, cy, cz, 3.0))
    return hd


def tie_slots(body, K, z0, z1):
    """zip-tie slots straight through the clip, above and below the belt (tie wraps tongue + belt + front wall)"""
    w = P['tie_w'] / 2
    for xg in P['tie_x']:
        for za, zb in ((P['belt_top'] + 0.9, P['belt_top'] + 3.1), (z0 + 2.6, z0 + 4.8)):
            sl = ring(K['Rb'] - 2, K['Rf'] + 2, za, zb, K)
            I(sl, box(xg - w, xg + w, -80, 40, za, zb))
            D(body, sl)


def tie_grooves(body, K, hw, z0, z1):
    """grooves for an optional zip tie around the whole clip (squeezes the tongue onto a thin belt)"""
    w, d = P['tie_w'] / 2, P['tie_d']
    for xg in P['tie_x']:
        if abs(xg) > hw:
            continue
        f = ring(K['Rf'] - d, K['Rf'] + 3, z0 - 2, z1 + 2, K)
        I(f, box(xg - w, xg + w, -80, 40, z0 - 2, z1 + 2))
        t = ring(K['Rb'] - 3, K['Rf'] + 3, z1 - d, z1 + 2, K)
        I(t, box(xg - w, xg + w, -80, 40, z1 - d, z1 + 2))
        b = ring(K['Rb'] - 3, K['Rf'] + 3, z0 - 2, z0 + d, K)
        I(b, box(xg - w, xg + w, -80, 40, z0 - 2, z0 + d))
        D(body, f, t, b)


def build_shim(K, t):
    """spacer for thin belts: slides into the clip slot in front of the belt, hook under the front wall"""
    Rs = K['Rb'] + P['strap_t'] + P['belt_ch']
    s = ring(Rs - t, Rs - 0.1, P['plate_z0'] - 0.01, P['belt_top'] - 1.0, K)
    U(s, ring(Rs - t, K['Rf'] - 0.5, P['plate_z0'] - 1.6, P['plate_z0'], K))
    I(s, box(-46.0, 40.5, -80, 40, P['plate_z0'] - 2, P['belt_top']))       # stops short of the right tie slot
    return s


def build_belt_pad(K, xs):
    """pressure pad: sits in the recess of the front wall, the thumb screw pushes it onto the belt"""
    p = ring(K['Rs'] + 0.05, K['Rs'] + 1.25, 6.2, P['belt_top'] - 0.1, K)
    I(p, box(-6.4, 6.4, -80, 40, 5.0, P['belt_top']))
    a = math.degrees(math.asin(xs / (K['Rs'] + 0.65)))
    return xf(p, mrot((0, 0, 1), -a, (0, K['Cy'], 0)))


def build_belt_screw(K, xs):
    """thumb screw (shown loosened: tip flush with the recess, knob 4.5 mm off the boss)"""
    yf_ = front_y(K, xs)
    yr = K['Cy'] + math.sqrt((K['Rs'] + 1.35) ** 2 - xs ** 2)                  # recess bottom on the screw axis
    yk = yf_ + P['bs_boss'] + P['bs_travel']
    k = cyly(yk, yk + 4.0, xs, P['bs_z'], 4.5)
    for i in range(10):
        a = 2 * math.pi * i / 10
        D(k, cyly(yk - 1, yk + 5, xs + 4.5 * math.cos(a), P['bs_z'] + 4.5 * math.sin(a), 0.9))
    U(k, cyly(yr + 1.0, yk + 0.01, xs, P['bs_z'], 3.0))
    return k


def build_pitch_frame(K):
    c = P['sc']
    lx0, lx1, rx0, rx1 = K['pf_lx0'], K['pf_lx1'], K['pf_rx0'], K['pf_rx1']
    ub = K['u_rb'] - 2.5                                    # rear (-13.5)
    tab_u0 = K['tab_u0']                                    # 4.9
    vw0, vw1 = P['s_hw'] + c, P['s_hw'] + c + 3.1           # sleeve walls 6.4 .. 9.5
    pf = rrect('x', -11.0, 9.0, -12.0, 23.0, lx0, lx1, 5.0)               # left plate (horn up)
    U(pf, rrect('x', ub, tab_u0, -vw1, vw1, rx0, rx1, 3.0), cylx(rx0, rx1, 0, 0, P['pf_boss_r']))   # right plate + boss
    U(pf, box(lx1 - 0.01, rx0 + 0.01, ub, tab_u0, vw0, vw1), box(lx1 - 0.01, rx0 + 0.01, ub, tab_u0, -vw1, -vw0))
    tp = box(lx1 - 0.01, rx0 + 0.01, tab_u0 - 3.0, tab_u0, -vw1, vw1)
    D(tp, box(P['s_xn'] - c, P['s_xp'] + c, tab_u0 - 4, tab_u0 + 1, -vw0, vw0))
    U(pf, tp)
    U(pf, box(-3.0, 14.0, ub, K['u_rb'] - 0.3, -vw1, vw1))                 # rear wall
    # pitch horn pocket on the left plate outer face, arm pointing +v
    p, hole, pil = horn_pocket_x(lx0, +1, (0.0, 1.0))
    D(pf, p, hole, pil)
    # roll servo tab pilot holes
    for dx in (P['s_mid'] - P['tab_hole_dx'], P['s_mid'] + P['tab_hole_dx']):
        D(pf, cyly(tab_u0 + 0.1, tab_u0 - 6.0, dx, 0, P['tab_pilot_r']))
    D(pf, box(17.6, rx0 + 0.1, -7.0, 2.0, -vw1 - 1.0, -vw0 + 0.1))       # IMU wire enters from below
    # trunnion (turns in the open-top bearing of the base arm); hollow, with a slot that faces UP at the
    # nominal tilt, so the roll-servo and IMU wires are simply laid in from above
    tx1 = P['trun_x1']
    U(pf, cylx(rx1 - 0.01, tx1, 0, 0, P['trun_r']))
    D(pf, cylx(rx0 - 1.0, tx1 + 1, 0, 0, P['trun_bore']))
    a = math.radians(P['tilt_nom'])
    du, dv, Ls = -math.sin(a), math.cos(a), 15.0
    D(pf, obox((0.5 * (rx0 + tx1 + 1), du * Ls / 2, dv * Ls / 2), (1, 0, 0), (0, du, dv), tx1 + 1 - rx0, Ls, P['slot_w']))
    return pf


def clamp_geom(K):
    g = P['clamp_gap']
    xc = -P['cam_w'] / 2 - g                       # jaw face -45.2
    return dict(jaw=(xc - 3.8, xc), post=(xc - 14.3, xc - 5.3), u=K['u_tp'], v=K['v_cb'] + 5.0)


def build_tray(K):
    u_hf, u_cb, u_cf, v_cb = K['u_hf'], K['u_cb'], K['u_cf'], K['v_cb']
    vf = v_cb - P['floor_t']
    cg = clamp_geom(K)
    xr = P['cam_w'] / 2 + P['clamp_gap']
    p0, p1 = cg['post']
    j0, j1 = cg['jaw']
    tr = rrect('y', -12.0, 12.0, -21.5, 12.0, u_hf, u_cb - P['tray_gap'], 4.0)          # back plate
    U(tr, rrect('z', -14.0, xr + 3.0, u_hf, u_cf - 1.0, vf, v_cb, 4.0))                  # floor (centre + right)
    U(tr, rrect('z', p0, -13.0, cg['u'] - 7.5, cg['u'] + 7.5, vf, v_cb, 2.0))            # strip to the clamp
    # (no front lip: the front of the camera stays completely free)
    # right end stop: only 2.5 mm high, so any USB-C plug fits above it
    U(tr, box(xr, xr + 3.0, u_cb + 1.0, u_cf - 1.0, v_cb - 0.01, v_cb + 2.5))
    # guide rails for the sliding jaw (jaw is a separate part)
    for ua, ub in ((cg['u'] - 7.5, cg['u'] - 6.0), (cg['u'] + 6.0, cg['u'] + 7.5)):
        U(tr, box(p1, -P['cam_w'] / 2 - 0.6, ua, ub, v_cb - 0.01, v_cb + 3.5))
    # screw post with captive M6 nut (drops in from the top)
    U(tr, rrect('y', p0, p1, vf, v_cb + 9.5, cg['u'] - 5.5, cg['u'] + 5.5, 1.5))
    D(tr, cylx(p0 - 1, p1 + 1, cg['u'], cg['v'], 3.2))
    nx0 = 0.5 * (p0 + p1) - 2.6
    D(tr, hexx(nx0, nx0 + 5.2, cg['u'], cg['v'], 10.2, 0.0))
    D(tr, box(nx0, nx0 + 5.2, cg['u'] - 5.1, cg['u'] + 5.1, cg['v'], v_cb + 10.0))
    # snap clip for the USB-C cable (cable diameter 4-5 mm), under the right end
    ccx, ccv = xr + 5.5, vf + 2.5
    U(tr, cyly(24.0, 32.0, ccx, ccv, 4.0))
    D(tr, cyly(23.0, 33.0, ccx, ccv, 2.4), box(ccx, ccx + 6.0, 23.0, 33.0, ccv - 1.6, ccv + 1.6))
    p, hole, pil = horn_pocket_y(u_hf, +1, (0.0, -1.0))                                # roll horn, arm down
    D(tr, p, hole, pil)
    # tripod screw slot (+-2 mm front/back): the screw always finds the camera thread
    r = P['tripod_hole_r']
    D(tr, cylz(vf - 1, v_cb + 1, 0, K['u_tp'] - 2.0, r), cylz(vf - 1, v_cb + 1, 0, K['u_tp'] + 2.0, r),
      box(-r, r, K['u_tp'] - 2.0, K['u_tp'] + 2.0, vf - 1, v_cb + 1))
    D(tr, box(P['imu_x0'], P['imu_x1'], P['imu_u0'], P['imu_u1'], vf - 0.1, vf + P['imu_depth']))
    D(tr, box(6.0, 35.0, u_hf - 1, P['imu_u0'] + 0.1, vf - 0.1, vf + 1.9))             # wire channel (exits near the roll axis)
    # IMU fixing: 2 pegs through the GY-521 mounting holes (M3, 15.5 mm apart) + sliding lid underneath
    ixc = 0.5 * (P['imu_x0'] + P['imu_x1'])
    for xh in (ixc - 7.75, ixc + 7.75):
        U(tr, cylz(vf + 0.1, vf + P['imu_depth'] + 0.01, xh, P['imu_peg_u'], 1.2))
    for xr0, xr1, g0, g1 in ((P['imu_x0'] - 2.0, P['imu_x0'], P['imu_x0'] - 1.0, P['imu_x0'] + 0.01),
                             (P['imu_x1'], P['imu_x1'] + 2.0, P['imu_x1'] - 0.01, P['imu_x1'] + 1.0)):
        U(tr, box(xr0, xr1, 27.5, u_cf - 1.0, vf - 2.2, vf + 0.01))
        D(tr, box(g0, g1, 28.5, u_cf, vf - 1.2, vf + 0.02))
    D(tr, cylz(vf - 0.01, vf + 0.4, ixc, 43.5, 0.8))                                     # detent for the lid
    return tr


def build_clamp_jaw(K):
    """separate sliding jaw: the screw tip sits in the hole at its back, the flat face presses the camera end"""
    cg = clamp_geom(K)
    j0, j1 = cg['jaw']
    v_cb = K['v_cb']
    j = rrect('x', cg['u'] - 5.5, cg['u'] + 5.5, v_cb + 0.1, v_cb + 8.5, j0, j1, 1.0)
    D(j, cylx(j0 - 0.1, j0 + 3.0, cg['u'], cg['v'], 3.3))
    D(j, box(j1 - 0.8, j1 + 0.1, cg['u'] - 4.0, cg['u'] + 4.0, v_cb + 1.5, v_cb + 7.0))   # recess for a 1 mm rubber/felt pad
    return j


def build_clamp_screw(K):
    cg = clamp_geom(K)
    tip = cg['jaw'][0] + 2.8
    h1 = cg['post'][0] - 1.0
    hd = cylx(h1 - 5.0, h1, cg['u'], cg['v'], 7.0)
    for i in range(12):
        a = 2 * math.pi * i / 12
        D(hd, cylx(h1 - 6, h1 + 1, cg['u'] + 7.0 * math.cos(a), cg['v'] + 7.0 * math.sin(a), 1.2))
    U(hd, cylx(h1 - 0.01, tip, cg['u'], cg['v'], 3.0))
    return hd


def build_thumbscrew(K, thread=True):
    vf = K['v_cb'] - P['floor_t']
    u = K['u_tp']
    hd = cylz(vf - P['head_t'], vf, 0, u, P['head_r'])
    for i in range(16):
        a = 2 * math.pi * i / 16
        D(hd, cylz(vf - P['head_t'] - 1, vf + 1, P['head_r'] * math.cos(a), u + P['head_r'] * math.sin(a), 1.3))
    U(hd, cylz(vf - 0.01, K['v_cb'] + P['screw_engage'], 0, u, 6.35 / 2))
    return hd


def build_pivot_bushing(K):
    """hollow pivot: shoulder turns in the base arm, M12x1.5 end screws into the pitch frame, wires run through the bore"""
    rx0, rx1, ax1 = K['pf_rx0'], K['pf_rx1'], K['arm_x1']
    xt, hx0 = rx0 + 2.2, ax1 + 0.4
    hx1, hr, br = hx0 + 4.0, P['bush_head_r'], P['bush_bore_r']
    b = cylx(hx0, hx1, 0, 0, hr)
    for i in range(12):
        a = 2 * math.pi * i / 12
        D(b, cylx(hx0 - 1, hx1 + 1, hr * math.cos(a), hr * math.sin(a), 1.2))
    U(b, cylx(rx1, hx0 + 0.01, 0, 0, P['shoulder_r']))
    U(b, cylx(xt, rx1 + 0.01, 0, 0, P['bush_thr_r']))
    D(b, cylx(xt - 1, hx1 + 1, 0, 0, br))
    D(b, TB.createCylinderOrCone(Pt(hx1 - 1.2, 0, 0), br / 10.0, Pt(hx1 + 0.01, 0, 0), (br + 1.2) / 10.0))
    D(b, TB.createCylinderOrCone(Pt(xt - 0.01, 0, 0), (br + 1.0) / 10.0, Pt(xt + 1.0, 0, 0), br / 10.0))
    return b


def nut_x(x0, y, z):
    n = hexx(x0, x0 + P['nut_t'] - 0.2, y, z, P['nut_af'], 0.0)
    D(n, cylx(x0 - 1, x0 + P['nut_t'] + 1, y, z, 4.92 / 2))
    return n


def build_pivot_nut(K):
    return nut_x(K['pf_rx0'] + 0.1, 0, 0)


def build_clamp_nut(K):
    cg = clamp_geom(K)
    nx0 = 0.5 * (cg['post'][0] + cg['post'][1]) - 2.6
    return nut_x(nx0 + 0.1, cg['u'], cg['v'])


def build_spacer(K):
    x0, x1 = K['pf_rx1'] + 0.2, K['arm_x0'] - 0.2
    s = cylx(x0, x1, 0, 0, 9.0)
    D(s, cylx(x0 - 1, x1 + 1, 0, 0, P['shoulder_r'] + 0.15))
    D(s, box(x0 - 1, x1 + 1, -6.2, 6.2, -10.0, 0.0))     # C opening 12.4 (snaps over the 13.6 mm shoulder)
    return s


def build_camera(K):
    cam = box(-P['cam_w'] / 2, P['cam_w'] / 2, K['u_cb'], K['u_cf'], K['v_cb'], -K['v_cb'])
    D(cam, cylz(K['v_cb'] - 0.1, K['v_cb'] + 6.0, 0, K['u_tp'], 3.2))
    plug = box(45.0, 47.5, K['u_tp'] - 4.2, K['u_tp'] + 4.2, -1.3, 1.3)
    U(plug, box(47.5, 66.0, K['u_tp'] - 6.0, K['u_tp'] + 6.0, -3.5, 3.5), cylx(65.9, 80.0, K['u_tp'], 0, 2.0))
    return cam, plug


def build_imu(K):
    vf = K['v_cb'] - P['floor_t']
    ixc = 0.5 * (P['imu_x0'] + P['imu_x1'])
    b = box(ixc - 10.5, ixc + 10.5, P['imu_u0'] + 0.5, P['imu_u1'] - 0.5, vf + 0.0, vf + 1.6)
    for xh in (ixc - 7.75, ixc + 7.75):
        D(b, cylz(vf - 1, vf + 3, xh, P['imu_peg_u'], 1.5))
    U(b, box(24.0, 28.0, 31.0, 35.0, vf + 1.6, vf + 2.5))
    return b


def build_imu_lid(K):
    vf = K['v_cb'] - P['floor_t']
    ixc = 0.5 * (P['imu_x0'] + P['imu_x1'])
    l = box(P['imu_x0'] - 0.9, P['imu_x1'] + 0.9, 28.6, K['u_cf'] - 1.0, vf - 1.0, vf)
    U(l, cylz(vf - 0.01, vf + 0.3, ixc, 43.5, 0.6))                                      # detent bump
    D(l, cylz(vf - 2, vf + 1, ixc + 7.0, K['u_cf'] - 1.0, 2.5))                          # finger notch
    return l


def build_pivot_cap(K):
    """closes the open-top bearing; slides in along x on a dovetail, held in place by the ESP cover hood"""
    Yg, Zg = P['Yg'], P['Zg']
    xa2, x1 = K['arm_x0'], P['trun_x1']
    ar, zt2 = P['arm_r'], Zg + 0.5
    c = cylx(xa2, x1, Yg, Zg, ar)
    I(c, box(xa2 - 1, x1 + 1, Yg - ar - 1, Yg + ar + 1, zt2, Zg + ar + 1))
    U(c, rrect('x', -1.5, Yg, zt2, 12.0, xa2, x1, 2.5))
    D(c, cylx(xa2 - 1, x1 + 1, Yg, Zg, P['brg_r']))
    U(c, dovetail(xa2 + 0.6, x1, P['dv_y'], zt2, 0.2))
    return c


def build_fov(K, hdeg=87.0, vdeg=58.0, ax=40.0, av=5.0, L=200.0):
    u0 = K['u_cf'] + 0.05
    c = box(-600, 600, u0, u0 + L, -600, 600)
    h, v = math.radians(hdeg / 2), math.radians(vdeg / 2)
    D(c, halfspace((0, u0, av), (0, -math.sin(v), math.cos(v))))
    D(c, halfspace((0, u0, -av), (0, -math.sin(v), -math.cos(v))))
    D(c, halfspace((ax, u0, 0), (math.cos(h), -math.sin(h), 0)))
    D(c, halfspace((-ax, u0, 0), (-math.cos(h), -math.sin(h), 0)))
    return c


# ---------------------------------------------------------------- battery box (separate, own belt clip; frame E = B)
def clip_plate(K, hw, z0, z1):
    Rb, Rf, Cy = K['Rb'], K['Rf'], K['Cy']
    Rt, Rs = Rb + P['strap_t'], Rb + P['strap_t'] + P['belt_ch']
    slab = ring(Rb, Rf, z0 - 1, z1 + 1, K)
    I(slab, rrect('y', -hw, hw, z0, z1, -60.0, 40.0, P['plate_rc']))
    bt = P['belt_top']
    slot = ring(Rt, Rs, z0 - 2, bt, K)
    I(slot, box(-100, 100, -80, 40, z0 - 2, bt))
    D(slab, slot)
    barb = ring(Rt - 0.05, Rt + P['barb_h'], -bt - 1.0, -bt, K)
    cone = TB.createCylinderOrCone(Pt(0, Cy, -bt - 1.0 - P['barb_h']), Rt / 10.0, Pt(0, Cy, -bt - 1.0), (Rt + P['barb_h']) / 10.0)
    D(cone, cylz(-bt - 5, -bt + 1, 0, Cy, Rt - 0.05))
    U(barb, cone)
    I(barb, rrect('y', -hw, hw, z0, z1, -60.0, 40.0, P['plate_rc']))
    I(barb, box(-100, 100, -80, 40, -bt - 1.0 - P['barb_h'] - 0.01, -bt))
    U(slab, barb)
    return slab


def build_batt_box(K):
    hw, z0, z1, y1 = P['bb_hw'], P['plate_z0'], P['plate_z1'], P['bb_y1']
    w = 2.5
    b = clip_plate(K, hw, z0, z1)
    shell = rrect('z', -hw, hw, -60.0, y1, z0, z1, 3.0)
    D(shell, cylz(z0 - 1, z1 + 1, 0, K['Cy'], K['Rf'] - 0.5))
    D(shell, box(-hw + w, hw - w, -60.0, y1 + 1, z0 + w, z1 - w))
    U(b, shell)
    # lid bolt posts (threaded M6 from the front)
    for sx in (-1, 1):
        px, pz = sx * P['bb_post'][0], P['bb_post'][1]
        post = cyly(-10.0, y1, px, pz, 5.5)
        D(post, cylz(z0 - 1, z1 + 1, 0, K['Cy'], K['Rf'] - 0.5))
        U(b, post)
        D(b, cyly(y1 - 12.0, y1 + 1, px, pz, 4.92 / 2))
    # holder locators on the floor (holder 76-78 x 21-22 x 18-22 mm, lies along X at the bottom)
    zf = z0 + w
    L = P['batt_l']
    U(b, box(-L / 2 - 1.2, -L / 2, 1.0, 17.0, zf - 0.01, zf + 3.0), box(L / 2, L / 2 + 1.2, 1.0, 17.0, zf - 0.01, zf + 3.0))
    # top wall: charger USB-C slot, switch hole, output cable hole + tie anchor
    D(b, box(-8.0, 8.0, 4.0, 14.0, z1 - w - 0.1, z1 + 1))
    D(b, box(-29.0, -15.8, 6.0, 14.6, z1 - w - 0.1, z1 + 1))
    D(b, box(17.0, 25.0, 4.0, 11.0, z1 - w - 0.1, z1 + 1))
    a = box(14.0, 28.0, 14.0, 20.0, z1 - 0.01, z1 + 3.0)
    D(a, box(18.5, 23.5, 13.9, 20.1, z1 - 0.1, z1 + 1.6))
    U(b, a)
    return b


def build_batt_lid(K):
    hw, z0, z1, y1, t = P['bb_hw'], P['plate_z0'], P['plate_z1'], P['bb_y1'], P['bb_lid']
    lid = rrect('y', -hw, hw, z0, z1, y1, y1 + t, 3.0)
    for sx in (-1, 1):
        D(lid, cyly(y1 - 1, y1 + t + 1, sx * P['bb_post'][0], P['bb_post'][1], 3.3))
    # ribs that press the battery holder against the back wall
    holder_front = 0.3 + P['batt_w']
    for x0 in (-30.0, 20.0):
        U(lid, box(x0, x0 + 10.0, holder_front + 0.2, y1 + 0.01, z0 + 6.0, z0 + 18.0))
    return lid


def build_batt_bolt(K, sx=1):
    y0 = P['bb_y1'] + P['bb_lid']
    px, pz = sx * P['bb_post'][0], P['bb_post'][1]
    hd = cyly(y0, y0 + 5.0, px, pz, 7.0)
    for i in range(12):
        a = 2 * math.pi * i / 12
        D(hd, cyly(y0 - 1, y0 + 6, px + 7.0 * math.cos(a), pz + 7.0 * math.sin(a), 1.2))
    U(hd, cyly(y0 - 13.0, y0 + 0.01, px, pz, 3.0))
    return hd


def build_esp_dummy(K):
    cf, px, cy0 = P['esp_cf'], P['usb_x'], P['esp_cy0']
    b = box(px - 9.0, px + 9.0, cy0 + 1.0, cy0 + 2.6, cf, cf + 22.5)                # PCB 18 x 22.5, 1 mm off the back
    U(b, box(px - 4.5, px + 4.5, cy0 + 2.6, cy0 + 5.8, cf, cf + 7.5))               # USB-C receptacle, mouth on the floor
    return b


LOCK = dict(x=58.5, y=11.6, r=2.6, z0=-9.0, z1=4.0, gy0=14.4, gy1=16.0)


def build_esp_cover(K):
    """front cover of the ESP box + hood over the wires from the pivot + camera-cable lock.
       Slides DOWN into the grooves of the box. The hood is open at the bottom, so it slides over the wires."""
    cx0, cx1, cf, zt = P['esp_x0'] + 2.0, P['esp_x1'] - 2.0, P['esp_cf'], P['esp_top']
    yd0, yd1, yf = P['door_y0'] + 0.1, P['door_y1'] - 0.1, P['esp_y1']
    c = box(cx0 - 0.8, cx1 + 0.8, yd0, yd1, cf + 0.2, zt)                          # plate in the grooves
    U(c, box(cx0 + 0.15, cx1 - 0.15, yd1 - 0.05, yf, cf + 0.2, zt))                 # face, flush with the box
    U(c, box(cx0 - 0.8, cx1 + 0.8, yd0, yf, zt, zt + 1.0))                          # grip, rests on the box
    hx0, hx1, hz0, hz1, hy1 = 41.8, 48.6, -3.5, 9.5, 21.8                         # slim chute over the wires
    ch = rrect('z', hx0, hx1, P['esp_y1'] + 0.1, hy1, hz0, hz1, 1.5)
    U(ch, rrect('z', hx0, P['esp_x0'] - 0.1, 0.5, P['esp_y1'] + 0.6, hz0, hz1, 1.5))     # beside the box
    U(ch, box(47.3, 50.5, 7.9, 9.4, hz0, hz1), box(49.7, 50.5, 7.0, 9.4, hz0, hz1))  # joins the cover face
    D(ch, box(hx0 + 1.5, 47.1, 2.0, hy1 - 1.5, hz0 - 1, hz1 - 1.5))                  # hollow, open at the bottom
    D(ch, box(46.5, 49.0, 1.5, 7.9, hz0 - 1, hz1 - 1.5))                             # opens into the side notch of the box
    U(c, ch)
    Yg, Zg = P['Yg'], P['Zg']
    D(c, cylx(hx0 - 1, hx0 + 2, Yg, Zg, 3.6))                                       # wires come in from the trunnion
    D(c, box(hx0 - 1, hx0 + 2, Yg - 2.6, Yg + 2.6, hz0 - 1, Zg))                     # open to the bottom
    L = LOCK
    lk = rrect('z', L['x'] - 5.0, L['x'] + 5.0, yf - 0.1, L['gy1'] + 1.0, L['z0'], L['z1'], 1.5)
    D(lk, cylz(L['z0'] - 1, L['z1'] + 1, L['x'], L['y'], L['r']))
    D(lk, box(L['x'] - L['r'], L['x'] + L['r'], L['y'], L['gy1'] + 2, L['z0'] - 1, L['z1'] + 1))
    D(lk, box(L['x'] - 3.6, L['x'] + 3.6, L['gy0'], L['gy1'], L['z0'] + 1.2, L['z1'] + 1))
    U(c, lk)
    return c


def build_usb_gate(K):
    """wedge gate: the lower it slides, the harder it presses the camera cable (fits 4-5 mm cables)"""
    L = LOCK
    zb, zt = L['z0'] + 1.3, L['z1'] + 2.5
    g = box(L['x'] - 3.5, L['x'] + 3.5, L['gy0'] + 0.1, L['gy1'] - 0.1, zb, zt)
    U(g, box(L['x'] - 3.5, L['x'] + 3.5, L['gy0'] + 0.1, L['gy1'] + 1.0, L['z1'], zt))         # grip above the lock
    w = box(L['x'] - 2.4, L['x'] + 2.4, L['gy0'] - 2.0, L['gy0'] + 0.2, zb, L['z1'] - 0.2)
    D(w, halfspace((0, L['gy0'] + 0.1, zb), (0, -(L['z1'] - 0.2 - zb), -1.9)))     # pad 0 mm at the bottom -> 1.9 mm at the top
    U(g, w)
    return g


def build_power_dummies(K):
    """470 uF capacitor (6.3 x 11), head of the power USB-C plug (12 x 7, typical), pivot-bushing insertion path"""
    cap = cylz(P['esp_cf'] + 8.0, P['esp_cf'] + 19.0, 64.5, P['esp_cy1'] - 3.25, 3.15)
    px, py = P['usb_x'], P['usb_y']
    plug = box(px - 6.0, px + 6.0, py - 3.5, py + 3.5, P['esp_cf'] - 18.0, P['esp_cf'] - 0.01)
    path = cylx(K['arm_x1'] + 0.6, K['arm_x1'] + 0.6 + 26.0, P['Yg'], P['Zg'], P['bush_head_r'] + 0.6)
    return cap, plug, path


def cable(points, r):
    c = None
    for i in range(len(points) - 1):
        seg = cyl(points[i], points[i + 1], r)
        c = seg if c is None else U(c, seg)
    for p in points[1:-1]:
        U(c, TB.createSphere(Pt(*p), r / 10.0))
    return c


def LB(x, u, v, th):
    t = math.radians(th)
    return (x, P['Yg'] + u * math.cos(t) + v * math.sin(t), P['Zg'] - u * math.sin(t) + v * math.cos(t))


def wire_tail(dy):
    """wires from the trunnion end, inside the hood, through the side notch into the ESP box (B frame)"""
    return [(44.5, 18.4 + dy, 0.0), (45.2, 12.0 + dy, 0.6), (45.6, 6.8 + 0.3 * dy, 1.4), (47.0, 5.6 + 0.3 * dy, 2.2),
            (47.9, 4.7, 2.3 + 0.5 * dy), (48.8, 3.8, 2.4 + 0.5 * dy), (50.5, 3.2, 2.6 + dy), (52.5, 0.0, 1.0 + dy),
            (52.0, -2.6, -4.0 + dy)]


def build_wires_out(K):
    return U(cable(wire_tail(-0.5), 0.9), cable(wire_tail(0.9), 0.7))


def build_cable_dummies(K, th):
    def ya(x, R=186.0):
        return K['Cy'] + math.sqrt(R * R - x * x)
    zg, zs = P['plate_z1'] - 1.1, P['plate_z1'] - 1.1
    pwc = P['esp_x0'] + 5.5
    cy0 = P['esp_cy0']
    out = {}
    # 1 pitch servo: block slot -> feeder -> top channel -> groove down the front face -> into the ESP box over the board
    xf0 = K['ps_x0'] + 3.2
    p1 = [(xf0, 20.0, 21.0), (xf0, 20.0, zg), (xf0, ya(xf0), zg)] + \
         [(x, ya(x), zg if abs(x) > 12.0 else zs) for x in (-30.0, -15.0, -11.5, 0.0, 11.5, 15.0, 30.0)] + \
         [(37.0, ya(37.0), zs), (pwc - 1.0, ya(pwc - 1.0), zs), (pwc, ya(pwc, 191.8), zs), (pwc, ya(pwc, 191.8), 17.5),
          (pwc, cy0 - 0.4, 13.0), (pwc + 2.0, cy0 + 2.0, 12.9), (55.0, -2.0, 10.0), (51.5, -4.4, 2.0)]
    out['Kabel servo nunduk'] = (cable(p1, 0.9), ('plastic', 'glossy', 'red'))
    # 2 roll servo / 3 IMU: through the hollow pivot -> inside the hood -> into the ESP box
    tail = wire_tail
    p2 = [LB(17.5, -9.0, -3.0, th), LB(20.0, -7.0, -1.0, th), LB(21.8, -2.5, -1.5, th), LB(23.5, 1.2, -3.3, th),
          LB(P['trun_x1'] + 0.2, 1.2, -3.3, th)] + tail(-0.5)
    out['Kabel servo kiri-kanan'] = (cable(p2, 0.9), ('plastic', 'glossy', 'blue'))
    p3 = [LB(26.0, 22.0, -16.2, th), LB(20.0, 18.0, -16.2, th), LB(9.0, 17.5, -16.2, th), LB(8.0, 14.0, -14.5, th),
          LB(13.0, 8.0, -12.5, th), LB(19.0, 1.0, -11.0, th), LB(20.0, -2.0, -8.5, th), LB(20.5, -1.5, -4.5, th),
          LB(22.5, 1.5, -3.0, th), LB(24.5, 2.8, -2.4, th), LB(P['trun_x1'] + 0.2, 2.8, -2.4, th)] + tail(0.9)
    out['Kabel IMU'] = (cable(p3, 0.7), ('plastic', 'glossy', 'green'))
    # 4 camera USB: plug -> clip under the tray -> slack loop -> up through the lock on the hood -> away to the right
    L = LOCK
    p4 = [LB(66.0, 33.5, 0.0, th), LB(71.0, 33.5, -3.0, th), LB(71.0, 35.0, -14.5, th), LB(52.0, 35.0, -14.5, th),
          LB(50.7, 32.5, -14.5, th), LB(50.7, 20.0, -14.5, th), LB(50.7, 14.0, -16.0, th), (52.0, 23.0, -6.0), (55.5, 19.0, 8.0),
          (L['x'], L['y'] + 0.4, L['z1'] + 3.0), (L['x'], L['y'] + 0.4, L['z0'] - 1.0), (60.0, 13.5, -18.0),
          (61.0, 15.0, -55.0)]
    out['Kabel USB kamera'] = (cable(p4, 2.25), ('plastic', 'glossy', 'black'))
    # 5 power: below the plug head (power bank OR battery box)
    out['Kabel power (power bank)'] = (cylz(-62.0, P['esp_cf'] - 18.0, P['usb_x'], P['usb_y'], 2.0), ('plastic', 'glossy', 'yellow'))
    return out


def build_screw_dummies(K):
    """metal screws (preview only): (static B-frame, on the pitch frame L-frame, on the tray L-frame)"""
    Yg, Zg, xb = P['Yg'], P['Zg'], K['ps_x0'] + P['tab_z0']
    tz1 = K['ps_x0'] + P['tab_z1']
    st, pf, tr = [], [], []
    for dz in (P['s_mid'] - P['tab_hole_dx'], P['s_mid'] + P['tab_hole_dx']):          # pitch servo ears
        sc = cylx(xb - 5.0, tz1, Yg, Zg + dz, 0.95)
        U(sc, cylx(tz1, tz1 + 1.4, Yg, Zg + dz, 1.9))
        st.append(sc)
        x = dz                                                                          # roll servo ears
        sc = cyly(K['tab_u0'] + 2.6 - 5.0, K['tab_u0'] + 2.6, x, 0, 0.95)
        U(sc, cyly(K['tab_u0'] + 2.6, K['tab_u0'] + 4.0, x, 0, 1.9))
        pf.append(sc)
    lx0 = K['pf_lx0']                                                                   # pitch horn: centre + 2 arm screws
    sc = cylx(lx0 - 1.7, lx0 + 1.8, 0, 0, 1.0)
    U(sc, cylx(lx0 + 1.8, lx0 + 3.2, 0, 0, 1.9))
    pf.append(sc)
    for r in P['hp_pilots']:
        sc = cylx(lx0, lx0 + 6.0, 0, r, 0.9)
        U(sc, cylx(lx0 - 1.4, lx0, 0, r, 1.9))
        pf.append(sc)
    uh = K['u_hf']                                                                      # roll horn: centre + 2 arm screws (M2x4)
    sc = cyly(uh - 1.7, uh + 1.8, 0, 0, 1.0)
    U(sc, cyly(uh + 1.8, uh + 3.2, 0, 0, 1.9))
    tr.append(sc)
    for r in P['hp_pilots']:
        sc = cyly(uh, uh + 3.9, 0, -r, 0.9)
        U(sc, cyly(uh - 1.4, uh, 0, -r, 1.9))
        tr.append(sc)
    return st, pf, tr
