

# ---------------------------------------------------------------- document: print layout, threads, STL, preview
VI = adsk.core.ValueInput
PRINT_ORIENT = {'G_Base': None, 'G_PitchFrame': ('y', -90), 'G_CamTray': ('x', 90), 'G_ThumbScrew': None,
                'G_ClampScrew': ('y', -90), 'G_ClampJaw': ('y', -90), 'G_CableScrew': ('y', 90), 'G_PivotCap': ('y', 90), 'G_ImuLid': None, 'G_BeltScrew': ('x', -90), 'G_BeltPad': ('x', 90), 'G_Nut_M6': ('y', -90), 'G_EspCover': ('x', 90), 'G_UsbGate': ('x', -90),
}
PRINT_ROWS = (('G_Base', 'G_PitchFrame', 'G_ClampScrew', 'G_ThumbScrew'),
              ('G_CamTray', 'G_ImuLid', 'G_EspCover'),
              ('G_PivotCap', 'G_BeltScrew#1', 'G_BeltScrew#2', 'G_BeltPad#1', 'G_BeltPad#2'),
              ('G_Nut_M6', 'G_ClampJaw', 'G_CableScrew', 'G_UsbGate'))
PRINT_QTY = {'G_BeltScrew': 2, 'G_BeltPad': 2}
THREADS = {'G_ThumbScrew': (6.35 / 2, False, 'UNC14'), 'G_BeltScrew': (3.0, False, 'M6'), 'G_Nut_M6': (4.92 / 2, True, 'M6'),
           'G_ClampScrew': (3.0, False, 'M6'),
           'G_CableScrew': (3.0, False, 'M6'), 'G_Base': (4.92 / 2, True, 'M6')}
THREAD_CLEAR = 0.15


def add_thread(comp, body, radius_mm, internal, kind, log, label, clear=0.0):
    try:
        faces = []
        for f in body.faces:
            gm = f.geometry
            if gm.surfaceType == adsk.core.SurfaceTypes.CylinderSurfaceType and abs(gm.radius * 10.0 - radius_mm) < 0.01:
                faces.append(f)
        if not faces:
            raise RuntimeError('cylinder r={} not found'.format(radius_mm))
        tf = comp.features.threadFeatures
        q = tf.threadDataQuery
        if kind == 'UNC14':
            ttype = [t for t in q.allThreadTypes if 'ANSI Unified' in t][0]
            size = [z for z in q.allSizes(ttype) if z.strip() in ('1/4', '0.25')][0]
            desig = [d for d in q.allDesignations(ttype, size) if '-20' in d][0]
            pref = '2B' if internal else '2A'
        else:
            ttype = [t for t in q.allThreadTypes if t == 'ISO Metric profile'][0]
            sz, dn = {'M6': ('6', ('M6x1', 'M6x1.0')), 'M12': ('12', ('M12x1.5',))}[kind]
            size = [z for z in q.allSizes(ttype) if z.strip() in (sz, sz + '.0')][0]
            desig = [d for d in q.allDesignations(ttype, size) if d.replace(' ', '') in dn][0]
            pref = '6H' if internal else '6g'
        cls = list(q.allClasses(internal, ttype, desig))
        info = tf.createThreadInfo(internal, ttype, desig, pref if pref in cls else cls[0])
        coll = adsk.core.ObjectCollection.create()
        for f in faces:
            coll.add(f)
        ti = tf.createInput(coll, info)
        ti.isModeled = True
        ti.isFullLength = True
        feat = tf.add(ti)
        msg = '  {}: modeled thread {} on {} face(s)'.format(label, desig, len(faces))
        if clear > 0:
            try:
                faces = [f for f in feat.faces]
                oi = comp.features.offsetFacesFeatures.createInput(faces, VI.createByReal(-clear / 10.0))
                comp.features.offsetFacesFeatures.add(oi)
                msg += ', print clearance {:.2f} mm'.format(clear)
            except Exception as e:
                msg += ', clearance NOT applied ({})'.format(e)
        log.append(msg)
    except Exception as e:
        log.append('  {}: thread NOT modeled ({})'.format(label, e))


def add_bodies(parent, name, bodies):
    o = parent.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    c = o.component
    c.name = name
    bf = c.features.baseFeatures.add()
    bf.startEdit()
    for nm, tb in bodies:
        b = c.bRepBodies.add(tb, bf)
        b.name = nm
    bf.finishEdit()
    return o


def find_appearance(app, keys):
    for i in range(app.materialLibraries.count):
        lib = app.materialLibraries.item(i)
        try:
            for j in range(lib.appearances.count):
                a = lib.appearances.item(j)
                if all(k.lower() in a.name.lower() for k in keys):
                    return a
        except Exception:
            pass
    return None


def paint(design, app, comp, body_names, keys):
    try:
        a = find_appearance(app, keys)
        if a is None:
            return
        a2 = design.appearances.itemByName(a.name) or design.appearances.addByCopy(a, a.name)
        for b in comp.bRepBodies:
            if b.name in body_names:
                b.appearance = a2
    except Exception:
        pass


def print_layout(design, root, S, log, folder, gap=8.0):
    # every printed copy is on the plate and has its own STL (G_BeltScrew_1.stl, G_BeltScrew_2.stl, ...)
    entries = [n for row in PRINT_ROWS for n in row]
    for src in PRINT_ORIENT:
        n_on_plate = sum(1 for n in entries if n.split('#')[0] == src)
        if n_on_plate != PRINT_QTY.get(src, 1):
            raise RuntimeError('print plate has {}x {} but {}x is needed'.format(n_on_plate, src, PRINT_QTY.get(src, 1)))
    # cross-check: every printed piece used in the assembly is on the plate, same count
    PRINTED_COPIES = {'belt_screw_L': 'G_BeltScrew', 'belt_pad_L': 'G_BeltPad', 'belt_pad_R': 'G_BeltPad', 'clamp_nut': 'G_Nut_M6'}
    used = {}
    for n in STATIC + PITCHING + ROLLING:
        src = n if n.startswith('G_') else PRINTED_COPIES.get(n)
        if src:
            used[src] = used.get(src, 0) + 1
    plate = {}
    for n in entries:
        plate[n.split('#')[0]] = plate.get(n.split('#')[0], 0) + 1
    if used != plate:
        raise RuntimeError('assembly uses {} but the plate has {}'.format(used, plate))
    log.append('PRINT LIST = ASSEMBLY: {} pieces, {} STL files ({})'.format(
        len(entries), len(entries), ', '.join('{} x{}'.format(k, v) for k, v in sorted(plate.items()))))
    parts = {}
    for nm in entries:
        src = nm.split('#')[0]
        o = PRINT_ORIENT[src]
        tb = cp(S[src])
        if o:
            ax = (1, 0, 0) if o[0] == 'x' else (0, 1, 0)
            xf(tb, mrot(ax, o[1]))
        parts[nm] = tb
    dims = {}
    for nm, tb in parts.items():
        bb = tb.boundingBox
        dims[nm] = ((bb.maxPoint.x - bb.minPoint.x) * 10, (bb.maxPoint.y - bb.minPoint.y) * 10, (bb.maxPoint.z - bb.minPoint.z) * 10)
    row_w = [sum(dims[n][0] for n in row) + gap * (len(row) - 1) for row in PRINT_ROWS]
    row_d = [max(dims[n][1] for n in row) for row in PRINT_ROWS]
    total_d = sum(row_d) + gap * (len(PRINT_ROWS) - 1)
    lay = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    lay.component.name = 'GIMBAL_PRINT_LAYOUT'
    y_top = total_d / 2
    for ri, row in enumerate(PRINT_ROWS):
        x = -row_w[ri] / 2
        yc = y_top - row_d[ri] / 2
        for nm in row:
            tb = parts[nm]
            bb = tb.boundingBox
            m = adsk.core.Matrix3D.create()
            m.translation = adsk.core.Vector3D.create(x / 10 - bb.minPoint.x, yc / 10 - (bb.minPoint.y + bb.maxPoint.y) / 2, -bb.minPoint.z)
            TB.transform(tb, m)
            src = nm.split('#')[0]
            fn = nm.replace('#', '_')
            o = add_bodies(lay.component, 'PRINT_' + fn, [(fn, tb)])
            c = o.component
            body = c.bRepBodies.item(0)
            if src in THREADS:
                r, internal, kind = THREADS[src]
                add_thread(c, body, r, internal, kind, log, fn, THREAD_CLEAR)
                body = c.bRepBodies.item(0)
            log.append('  PRINT_{:<15s} {:5.1f} x {:5.1f} mm footprint, {:5.1f} mm high'.format(fn, *dims[nm]))
            if folder:
                opts = design.exportManager.createSTLExportOptions(body, os.path.join(folder, fn + '.stl'))
                opts.isBinaryFormat = True
                opts.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementHigh
                design.exportManager.execute(opts)
            x += dims[nm][0] + gap
        y_top -= row_d[ri] + gap
    log.append('  build plate area used: {:.0f} x {:.0f} mm (centred on the origin)'.format(max(row_w), total_d))
    return lay


def preview(design, app, root, K, S, Dm, dx=-320.0):
    th = P['tilt_nom']
    sh = mtr(dx, 0, 0)
    bodies = []
    for n, lab in (('G_Base', 'G_Base'), ('pitch_servo', 'MG90S_pitch (bought)'), ('esp32c3', 'ESP32-C3 SuperMini (bought)'),
                   ('capacitor', 'Kapasitor 470uF (bought)'), ('power_plug', 'Kepala colokan USB-C power (contoh)'),
                   ('G_CableScrew', 'G_CableScrew'), ('G_EspCover', 'G_EspCover'), ('G_UsbGate', 'G_UsbGate'),
                   ('G_PivotCap', 'G_PivotCap'), ('G_BeltScrew', 'G_BeltScrew'), ('belt_pad_R', 'G_BeltPad'),
                   ('belt_screw_L', 'G_BeltScrew 2'), ('belt_pad_L', 'G_BeltPad 2')):
        src = S.get(n) or Dm.get(n)
        bodies.append((lab, xf(cp(src), sh)))
    for n, lab in (('G_PitchFrame', 'G_PitchFrame'), ('roll_servo', 'MG90S_roll (bought)'), ('pitch_horn', 'horn_pitch (bought)')):
        src = S.get(n) or Dm.get(n)
        bodies.append((lab, xf(at_pose(src, th), sh)))
    for n, lab in (('G_CamTray', 'G_CamTray'), ('G_ThumbScrew', 'G_ThumbScrew'), ('G_ClampScrew', 'G_ClampScrew'),
                   ('G_ClampJaw', 'G_ClampJaw'),
                   ('clamp_nut', 'G_Nut_M6 (clamp)'), ('camera', 'D435 (dummy)'), ('usb_plug', 'USB-C plug (dummy)'),
                   ('imu', 'MPU6050 GY-521 (bought)'), ('roll_horn', 'horn_roll (bought)'), ('G_ImuLid', 'G_ImuLid')):
        src = S.get(n) or Dm.get(n)
        bodies.append((lab, xf(at_pose(src, th, 0.0), sh)))
    cables = build_cable_dummies(K, th)
    for nm, (b, col) in cables.items():
        bodies.append((nm, xf(b, sh)))
    st, pfs, trs = build_screw_dummies(K)
    scr = []
    for i, b in enumerate(st):
        scr.append(('sekrup M2 kuping servo nunduk %d' % (i + 1), xf(b, sh)))
    for i, b in enumerate(pfs):
        scr.append(('sekrup M2 rangka %d' % (i + 1), xf(at_pose(b, th), sh)))
    for i, b in enumerate(trs):
        scr.append(('sekrup M2 horn kiri-kanan %d' % (i + 1), xf(at_pose(b, th, 0.0), sh)))
    bodies += scr
    o = add_bodies(root, 'GIMBAL_ASSEMBLY_PREVIEW', bodies)
    c = o.component
    paint(design, app, c, [b[0] for b in bodies if b[0].startswith('G_') or b[0].startswith('E_')], ('plastic', 'matte', 'yellow'))
    for nm, (b, col) in cables.items():
        paint(design, app, c, (nm,), col)
    paint(design, app, c, ('D435 (dummy)', 'USB-C plug (dummy)'), ('plastic', 'matte', 'gray'))
    paint(design, app, c, ('MG90S_pitch (bought)', 'MG90S_roll (bought)', 'MPU6050 GY-521 (bought)',
                           'ESP32-C3 SuperMini (bought)'), ('plastic', 'glossy', 'blue'))
    paint(design, app, c, ('Kapasitor 470uF (bought)', 'Kepala colokan USB-C power (contoh)'), ('plastic', 'glossy', 'black'))
    paint(design, app, c, [n for n, b in scr], ('plastic', 'glossy', 'white'))
    paint(design, app, c, ('horn_pitch (bought)', 'horn_roll (bought)'), ('plastic', 'glossy', 'white'))
    return o


def run(_context):
    app = adsk.core.Application.get()
    base = os.path.join(os.path.expanduser('~'), 'D435_mount_STL', 'gimbal')
    folder = os.path.join(base, 'print_ready')
    os.makedirs(folder, exist_ok=True)
    log = []
    try:
        K, S, Dm = run_checks(log)
        design = None
        mine = ('GIMBAL_PRINT_LAYOUT', 'GIMBAL_ASSEMBLY_PREVIEW')
        for i in range(app.documents.count):
            d = app.documents.item(i)
            des = adsk.fusion.Design.cast(d.products.itemByProductType('DesignProductType'))
            if des and any(o.component.name in mine for o in des.rootComponent.occurrences):
                design = des
                d.activate()
        if design is None:
            doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
            design = adsk.fusion.Design.cast(doc.products.itemByProductType('DesignProductType'))
        design.designType = adsk.fusion.DesignTypes.ParametricDesignType
        root = design.rootComponent
        for i in range(root.occurrences.count - 1, -1, -1):
            o = root.occurrences.item(i)
            if o.component.name in mine:
                o.deleteMe()
        old = os.path.join(base, 'old_versions')
        os.makedirs(old, exist_ok=True)
        for f in os.listdir(folder):
            if f.endswith('.stl'):
                os.replace(os.path.join(folder, f), os.path.join(old, f))
        log.append('PRINT LAYOUT (each part its own component, print orientation, on Z = 0) -> ' + folder)
        lay = print_layout(design, root, S, log, folder)
        paint(design, app, lay.component, [], ())
        for c in [o.component for o in lay.component.occurrences]:
            paint(design, app, c, [b.name for b in c.bRepBodies], ('plastic', 'matte', 'yellow'))
        preview(design, app, root, K, S, Dm)
        design.exportManager.execute(design.exportManager.createFusionArchiveExportOptions(os.path.join(base, 'D435_gimbal.f3d')))
        log.append('f3d saved: ' + os.path.join(base, 'D435_gimbal.f3d'))
        try:
            app.activeViewport.fit()
        except Exception:
            pass
        log.append('DONE')
    except Exception:
        log.append('FAILED\n' + traceback.format_exc())
    text = '\n'.join(log)
    with open(os.path.join(base, 'build_log.txt'), 'w') as fh:
        fh.write(text)
    print(text)
