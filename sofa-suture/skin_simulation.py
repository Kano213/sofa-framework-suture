import os
import math
import ctypes
import numpy as np
import Sofa
import Sofa.Core
import SofaRuntime

# =============================================================================
# CHARGEMENT DES PLUGINS SOFA REQUIS
# =============================================================================
REQUIRED_PLUGINS = [
    "Sofa.Component.AnimationLoop",
    "Sofa.Component.Collision.Detection.Algorithm",
    "Sofa.Component.Collision.Detection.Intersection",
    "Sofa.Component.Collision.Geometry",
    "Sofa.Component.Collision.Response.Contact",
    "Sofa.Component.Constraint.Lagrangian.Solver",
    "Sofa.Component.Constraint.Lagrangian.Correction",
    "Sofa.Component.Constraint.Lagrangian.Model",
    "Sofa.Component.Constraint.Projective",
    "Sofa.Component.Controller",
    "Sofa.Component.Engine.Select",
    "Sofa.Component.IO.Mesh",
    "Sofa.Component.LinearSolver.Direct",
    "Sofa.Component.LinearSolver.Iterative",
    "Sofa.Component.Mapping.Linear",
    "Sofa.Component.Mapping.NonLinear",
    "Sofa.Component.Mass",
    "Sofa.Component.MechanicalLoad",
    "Sofa.Component.ODESolver.Backward",
    "Sofa.Component.Setting",
    "Sofa.Component.SolidMechanics.FEM.Elastic",
    "Sofa.Component.SolidMechanics.Spring",
    "Sofa.Component.StateContainer",
    "Sofa.Component.Topology.Container.Constant",
    "Sofa.Component.Topology.Container.Dynamic",
    "Sofa.Component.Topology.Mapping",
    "Sofa.Component.Visual",
    "Sofa.GL.Component.Rendering3D",
]

for plugin_name in REQUIRED_PLUGINS:
    try:
        SofaRuntime.importPlugin(plugin_name)
    except Exception:
        pass


# =============================================================================
# UTILITAIRES MATHÉMATIQUES & WINDOWS API (SANS AUCUN CRASH)
# =============================================================================
try:
    user32 = ctypes.windll.user32
except Exception:
    user32 = None


class POINT(ctypes.Structure):
    _fields_ = [("x", ctypes.c_long), ("y", ctypes.c_long)]


def is_key_pressed(vkey):
    """Interroge l'état matériel d'une touche sous Windows via l'API Win32."""
    if user32 is None:
        return False
    try:
        return bool(user32.GetAsyncKeyState(vkey) & 0x8000)
    except Exception:
        return False


def get_mouse_pos():
    """Récupère la position courante du curseur de la souris sous Windows."""
    if user32 is None:
        return 0, 0
    try:
        pt = POINT()
        user32.GetCursorPos(ctypes.byref(pt))
        return pt.x, pt.y
    except Exception:
        return 0, 0


def quat_mult(q1, q2):
    """Multiplication de deux quaternions [x, y, z, w]."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return [
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ]


def quat_from_axis_angle(axis, angle_rad):
    """Génère un quaternion unitaire [x, y, z, w] à partir d'un axe et d'un angle."""
    half = angle_rad * 0.5
    s = math.sin(half)
    return [axis[0] * s, axis[1] * s, axis[2] * s, math.cos(half)]


def rotate_vec_by_quat(v, q):
    """Fait pivoter un vecteur 3D par un quaternion unitaire [x, y, z, w]."""
    x, y, z, w = q
    vx, vy, vz = v
    rx = w * vx + y * vz - z * vy
    ry = w * vy + z * vx - x * vz
    rz = w * vz + x * vy - y * vx
    rw = -x * vx - y * vy - z * vz
    return [
        rx * w - rw * x - ry * z + rz * y,
        ry * w - rw * y - rz * x + rx * z,
        rz * w - rw * z - rx * y + ry * x,
    ]


# =============================================================================
# DIMENSIONS ET GÉOMÉTRIE CHIRURGICALE
# =============================================================================
NEEDLE_RADIUS = 0.95        # Rayon de courbure chirurgical (3/8 de cercle, ~19 mm de corde)
NEEDLE_ANGLE_DEG = 145.0    # Arc de courbure (145 degrés)
MAX_KNOTS = 6               # Nombre maximal de points de suture le long des 4 cm
SKIN_TOP_Y = 1.50           # Hauteur nominale de la surface supérieure de la peau (en cm)
FENTE_FLOOR_Y = 0.75        # Hauteur du fond de la fente d'incision (en cm)


def make_skin_mesh_with_incision(
    ny=3, y_range=(0.0, 1.5), incision_length=4.0, incision_depth=1
):
    """
    Génère un bloc de peau volumique réaliste avec fente centrale de 4 cm (z entre -2.0 et +2.0 cm).
    Maillage raffiné tous les 5 mm le long de l'incision pour permettre une fermeture
    progressive et localisée berge par berge sur jusqu'à 6 points de suture.
    Les berges sont séparées par une ouverture de 2.0 mm (x = -0.10 et x = +0.10 cm).
    """
    xs_left = np.array([-4.0, -2.5, -1.4, -0.6, -0.1])
    xs_right = np.array([0.1, 0.6, 1.4, 2.5, 4.0])
    xs = np.concatenate([xs_left, xs_right])
    ys = np.linspace(y_range[0], y_range[1], ny)
    # Raffinement de la grille Z tous les 0.5 cm dans la zone d'incision [-2.0, +2.0]
    zs = np.array([-4.0, -3.0, -2.0, -1.5, -1.0, -0.5, 0.0, 0.5, 1.0, 1.5, 2.0, 3.0, 4.0])
    nz = len(zs)

    nx_total = len(xs)
    split_col_left = len(xs_left) - 1

    verts = []
    idx_map = {}
    for iy, y in enumerate(ys):
        for iz, z in enumerate(zs):
            for ix, x in enumerate(xs):
                idx_map[(ix, iy, iz)] = len(verts)
                verts.append([float(x), float(y), float(z)])

    tetras = []
    for iy in range(ny - 1):
        for iz in range(nz - 1):
            z_mid = 0.5 * (zs[iz] + zs[iz + 1])
            is_incision_z = abs(z_mid) <= (incision_length / 2.0)
            is_incision_y = iy >= (ny - 1 - incision_depth)

            for ix in range(nx_total - 1):
                if ix == split_col_left and is_incision_z and is_incision_y:
                    continue  # Fente centrale (non reliée)

                p0 = idx_map[(ix, iy, iz)]
                p1 = idx_map[(ix + 1, iy, iz)]
                p2 = idx_map[(ix + 1, iy + 1, iz)]
                p3 = idx_map[(ix, iy + 1, iz)]
                p4 = idx_map[(ix, iy, iz + 1)]
                p5 = idx_map[(ix + 1, iy, iz + 1)]
                p6 = idx_map[(ix + 1, iy + 1, iz + 1)]
                p7 = idx_map[(ix, iy + 1, iz + 1)]

                if (ix + iy + iz) % 2 == 0:
                    tets = [
                        [p0, p1, p3, p4],
                        [p1, p2, p3, p6],
                        [p1, p4, p5, p6],
                        [p3, p4, p6, p7],
                        [p1, p3, p4, p6],
                    ]
                else:
                    tets = [
                        [p0, p1, p2, p5],
                        [p0, p2, p3, p7],
                        [p0, p4, p5, p7],
                        [p2, p5, p6, p7],
                        [p0, p5, p2, p7],
                    ]
                tetras.extend(tets)

    return verts, tetras, idx_map, xs_left, xs_right, zs


def make_puncture_mark_data(r=0.065, n_segs=12):
    """Génère un disque marquant l'orifice de piqûre."""
    verts = [[0.0, 0.0, 0.0]]
    triangles = []
    for i in range(n_segs):
        theta = 2.0 * math.pi * i / float(n_segs)
        verts.append([r * math.cos(theta), 0.0, r * math.sin(theta)])
    for i in range(1, n_segs + 1):
        nxt = 1 if i == n_segs else i + 1
        triangles.append([0, i, nxt])
    return verts, triangles


def make_knot_mesh_data(p_left, p_right):
    """
    Génère la géométrie 3D d'un nœud chirurgical noué à la surface :
    - Pont de suture arqué reliant les deux orifices de berge sans traverser l'épiderme
    - Nœud central à double boucle chirurgicale au-dessus de la ligne d'incision
    - Deux brins coupés (fil résiduel de 4-5 mm) orientés vers le haut
    """
    p_l = np.array(p_left, dtype=float)
    p_r = np.array(p_right, dtype=float)

    # Élévation légère au-dessus de l'épiderme pour éviter tout z-fighting
    p_l[1] += 0.012
    p_r[1] += 0.012

    # 5 points le long du pont de suture avec léger profil arqué
    bridge_pts = []
    n_bridge = 5
    for i in range(n_bridge):
        alpha = i / float(n_bridge - 1)
        pt = (1.0 - alpha) * p_l + alpha * p_r
        pt[1] += 0.025 * math.sin(math.pi * alpha)
        bridge_pts.append([float(pt[0]), float(pt[1]), float(pt[2])])

    p_mid = bridge_pts[n_bridge // 2]

    # Brins coupés (environ 4.5 mm)
    tail1 = [p_mid[0] + 0.11, p_mid[1] + 0.16, p_mid[2] + 0.08]
    tail2 = [p_mid[0] - 0.09, p_mid[1] + 0.15, p_mid[2] - 0.10]

    # Double boucle centrale du nœud plat chirurgical
    loop1 = [p_mid[0] + 0.04, p_mid[1] + 0.035, p_mid[2] + 0.025]
    loop2 = [p_mid[0] - 0.04, p_mid[1] + 0.035, p_mid[2] - 0.025]
    loop3 = [p_mid[0] - 0.03, p_mid[1] + 0.045, p_mid[2] + 0.030]
    loop4 = [p_mid[0] + 0.03, p_mid[1] + 0.045, p_mid[2] - 0.030]

    pts = bridge_pts + [tail1, tail2, loop1, loop2, loop3, loop4]
    mid_idx = n_bridge // 2
    t1_idx = n_bridge
    t2_idx = n_bridge + 1
    l1_idx = n_bridge + 2
    l2_idx = n_bridge + 3
    l3_idx = n_bridge + 4
    l4_idx = n_bridge + 5

    lines = []
    for i in range(n_bridge - 1):
        lines.append([i, i + 1])  # Pont de suture transversal
    lines.extend([
        [mid_idx, t1_idx],
        [mid_idx, t2_idx],        # Deux brins coupés
        [mid_idx, l1_idx],
        [l1_idx, l2_idx],
        [l2_idx, mid_idx],        # Première boucle du nœud
        [mid_idx, l3_idx],
        [l3_idx, l4_idx],
        [l4_idx, mid_idx],        # Seconde boucle croisée du nœud
    ])
    return pts, lines


def ensure_needle_mesh(filepath):
    """Génère le maillage OBJ de l'aiguille courbe chirurgicale si nécessaire."""
    os.makedirs(os.path.dirname(os.path.abspath(filepath)), exist_ok=True)
    if os.path.exists(filepath):
        return

    n_segs = 48
    n_circ = 14
    r_section = 0.032
    max_angle_rad = math.radians(NEEDLE_ANGLE_DEG)
    R = NEEDLE_RADIUS

    curve_pts = []
    normals = []
    binormals = []
    for i in range(n_segs + 1):
        t = i / float(n_segs)
        angle = -t * max_angle_rad
        cx = R * math.cos(angle)
        cy = R * math.sin(angle)
        curve_pts.append([cx, cy, 0.0])
        normals.append([math.cos(angle), math.sin(angle), 0.0])
        binormals.append([0.0, 0.0, 1.0])

    verts = []
    for i in range(n_segs + 1):
        t = i / float(n_segs)
        # Pointe conique effilée sur les premiers 18% de l'arc
        if t <= 0.18:
            scale = max(0.02, (t / 0.18) ** 1.25)
        else:
            scale = 1.0
        r_cur = r_section * scale
        c = curve_pts[i]
        N = normals[i]
        B = binormals[i]
        for j in range(n_circ):
            theta = 2.0 * math.pi * j / float(n_circ)
            cos_t = math.cos(theta)
            sin_t = math.sin(theta)
            vx = c[0] + r_cur * (cos_t * N[0] + sin_t * B[0])
            vy = c[1] + r_cur * (cos_t * N[1] + sin_t * B[1])
            vz = c[2] + r_cur * (cos_t * N[2] + sin_t * B[2])
            verts.append([vx, vy, vz])

    faces = []
    for i in range(n_segs):
        for j in range(n_circ):
            nxt_j = (j + 1) % n_circ
            v0 = i * n_circ + j + 1
            v1 = i * n_circ + nxt_j + 1
            v2 = (i + 1) * n_circ + nxt_j + 1
            v3 = (i + 1) * n_circ + j + 1
            faces.append([v0, v1, v2])
            faces.append([v0, v2, v3])

    with open(filepath, "w") as f:
        f.write("# Aiguille chirurgicale courbe\n")
        for v in verts:
            f.write(f"v {v[0]:.6f} {v[1]:.6f} {v[2]:.6f}\n")
        for face in faces:
            f.write(f"f {face[0]} {face[1]} {face[2]}\n")


# =============================================================================
# CONTRÔLEUR CHIRURGICAL HAUTE FIDÉLITÉ (VERSION FINALE OPTIMISÉE)
# =============================================================================
class SurgicalNeedleController(Sofa.Core.Controller):
    """
    Contrôleur chirurgical complet et optimisé :
    - Cinématique d'arc auto-guidée lors de l'enroulement [E] : garantit que la pointe
      ressort toujours visuellement et physiquement au-dessus de l'épiderme (Y >= 1.50 cm),
      même si l'aiguille a été fortement enfoncée lors de la piqûre initiale.
    - Détection universelle multi-surfaces (1 temps, 2 temps via parois de fente,
      gauche->droite, droite->gauche, ou suture libre n'importe où).
    - Régulation de coaptation en boucle fermée : rapproche les berges de 2.0 mm à 0.0 mm
      sans jamais faire se chevaucher ni s'inverser les lèvres de la plaie.
    - Nœuds chirurgicaux permanents avec marqueurs d'orifices conservés et jusqu'à 6 points.
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.root = kwargs.get("root", None)
        self.idx_map = kwargs.get("idx_map", {})
        self.xs_left = kwargs.get("xs_left", [])
        self.xs_right = kwargs.get("xs_right", [])
        self.zs = kwargs.get("zs", [])

        # Pose initiale ergonomique : pointe inclinée à 35° juste au-dessus de la berge gauche
        self.init_pos = [0.15, 2.12, -1.20]
        self.pos = list(self.init_pos)

        self.init_suture_angle = 35.0
        self.suture_angle = self.init_suture_angle

        # Orientations de la pince porte-aiguille
        self.yaw_angle = 0.0
        self.pitch_angle = 0.0

        # Vitesses de déplacement par pas
        self.speed_trans = 0.03
        self.speed_rot = 1.4

        # Mode souris
        self.mouse_mode = False
        self.prev_tab_down = False
        self.prev_mouse_x = None
        self.prev_mouse_y = None

        # Machine d'état de pénétration chirurgicale pour le point courant
        self.puncture_state = "AIR_OUTSIDE"
        self.holes = []  # [{'pos': [x,y,z], 'bary': weights, 'type': str}]
        self.suture_attached = False
        self.min_ty_since_entry = 10.0
        self.entry_angle_deg = 0.0

        # Gestion des touches bistables
        self.prev_t_down = False
        self.prev_n_down = False
        self.prev_u_down = False

        # OPTION 1 : Tension active du fil (0.0 à 1.0) et régulation de coaptation
        self.suture_tension = 0.0
        self.last_reported_tension = -1.0
        self.max_traction_force = 420.0
        self.stitch_initial_eye_dist = None
        self.row_forces = {iz: 0.0 for iz in range(len(self.zs))}

        # OPTION 2 & 3 : Liste des points de suture noués et verrouillés
        self.completed_knots = []

        # Longueur du fil actif
        self.thread_length = 7.5

        # Coordonnées locales de la pointe et du chas
        self.tip_local = [NEEDLE_RADIUS, 0.0, 0.0]
        self.base_local = [
            NEEDLE_RADIUS * math.cos(math.radians(-NEEDLE_ANGLE_DEG)),
            NEEDLE_RADIUS * math.sin(math.radians(-NEEDLE_ANGLE_DEG)),
            0.0,
        ]

        # Références vers les composants SOFA
        self.needle_dofs = None
        self.needle_sphere_col = None
        self.col_reenable_cooldown = 0
        self.thread_dofs = None
        self.thread_fix = None
        self.skin_dofs = None
        self.traction_cff = None
        self.hole_dofs_list = []
        self.knot_hole_dofs_list = []
        self.knot_topos = []
        self.prev_eye_pos = None
        self._initialized = False

    def init(self):
        if self.root is None:
            try:
                ctx = self.getContext()
                if ctx is not None:
                    self.root = ctx.getRoot()
            except Exception:
                pass
        self._init_controller()

    def _init_controller(self):
        if self._initialized:
            return
        self._initialized = True
        if self.root:
            needle = self.root.getChild("Needle")
            if needle:
                self.needle_dofs = needle.getObject("dofs")
                col = needle.getChild("Collision")
                if col:
                    self.needle_sphere_col = col.getObject("sphereCol")
                    self.needle_sphere_col.active.value = True

            thread = self.root.getChild("Thread")
            if thread:
                self.thread_dofs = thread.getObject("dofs")
                self.thread_fix = thread.getObject("fixPoints")

            skin = self.root.getChild("SkinBlock")
            if skin:
                self.skin_dofs = skin.getObject("dofs")
                self.traction_cff = skin.getObject("traction")

            # Marqueurs des trous du point actif (jusqu'à 4 trous par point)
            self.hole_dofs_list = []
            for i in range(1, 5):
                h = self.root.getChild(f"Hole{i}")
                if h:
                    self.hole_dofs_list.append(h.getObject("dofs"))

            # Marqueurs permanents des orifices des points noués (2 par nœud)
            self.knot_hole_dofs_list = []
            for i in range(1, 2 * MAX_KNOTS + 1):
                kh = self.root.getChild(f"KnotHole{i}")
                if kh:
                    self.knot_hole_dofs_list.append(kh.getObject("dofs"))

            # Topologies des nœuds chirurgicaux (Option 2 & 3)
            self.knot_topos = []
            for i in range(1, MAX_KNOTS + 1):
                k = self.root.getChild(f"Knot{i}")
                if k:
                    self.knot_topos.append(k.getObject("topo"))

        self._sync_needle_and_thread(dt=0.02)
        self.printControls()

    def printControls(self):
        print("=" * 82)
        print("  SIMULATION CHIRURGICALE DE SUTURE - VERSION FINALE HAUTE FIDÉLITÉ")
        print("  * Option 1 : Rapprochement & Coaptation Élastique FEM des Berges (Tension)")
        print("  * Option 2 : Nœud Chirurgical Double Boucle, Brins Coupés & Verrouillage")
        print("  * Option 3 : Suture Multi-Points (jusqu'à 6 points le long des 4 cm)")
        print("=" * 82)
        print("  1. MANIPULATION DE L'AIGUILLE :")
        print("     [Flèches] ou [ZQSD]   : Déplacer l'aiguille librement (X / Z)")
        print("     [R]                   : Monter l'aiguille (Hauteur Y)")
        print("     [F] ou [C]            : Descendre l'aiguille (appuyer fermement pour percer)")
        print("     [E]                   : ENROULER le long de l'arc (traversée & sortie fluide)")
        print("     [A]                   : DÉROULER en arrière")
        print("     [J] / [L]             : Pivoter la pince en lacet (Yaw gauche / droite)")
        print("     [I] / [K]             : Incliner la pince en tangage (Pitch avant / arrière)")
        print("     [U]                   : Réaligner l'orientation de l'aiguille")
        print("     [TAB]                 : Activer / Désactiver le guidage à la souris")
        print("-" * 82)
        print("  2. GESTE DE SUTURE, TENSION & NŒUD :")
        print("     [T]                   : Assistance chirurgicale (Percer / Ressortir sur place)")
        print("     [V] ou [Espace]       : SERRER LA TENSION -> Ferme et coapte les berges (0 mm)")
        print("     [B]                   : DESSERRER LA TENSION")
        print("     [N]                   : NOUER LE POINT DE SUTURE (Nœud + coupe + réarmement)")
        print("     [X]                   : Réinitialiser toute la scène et rouvrir la plaie")
        print("=" * 82)

    def _compute_quaternion(self):
        """Calcule le quaternion d'orientation [qx, qy, qz, qw] de l'aiguille."""
        qz = quat_from_axis_angle([0.0, 0.0, 1.0], math.radians(180.0 + self.suture_angle))
        qx = quat_from_axis_angle([1.0, 0.0, 0.0], math.radians(self.pitch_angle))
        qy = quat_from_axis_angle([0.0, 1.0, 0.0], math.radians(self.yaw_angle))

        q_temp = quat_mult(qx, qz)
        q_final = quat_mult(qy, q_temp)

        norm = math.sqrt(sum(k * k for k in q_final))
        if norm > 1e-8:
            return [k / norm for k in q_final]
        return [0.0, 0.0, 0.0, 1.0]

    def _get_needle_eye_world_pos(self, quat):
        """Calcule la position 3D dans le monde du chas de l'aiguille."""
        rot_base = rotate_vec_by_quat(self.base_local, quat)
        return [
            float(self.pos[0] + rot_base[0]),
            float(self.pos[1] + rot_base[1]),
            float(self.pos[2] + rot_base[2]),
        ]

    def _get_needle_tip_world_pos(self, quat):
        """Calcule la position 3D dans le monde de la pointe de l'aiguille."""
        rot_tip = rotate_vec_by_quat(self.tip_local, quat)
        return [
            float(self.pos[0] + rot_tip[0]),
            float(self.pos[1] + rot_tip[1]),
            float(self.pos[2] + rot_tip[2]),
        ]

    def get_barycentric_weights(self, x, y, z):
        """
        Calcule les poids barycentriques d'un point arbitraire (x, y, z)
        par rapport aux 4 sommets du maillage de peau le plus proche.
        """
        x = max(-3.95, min(3.95, x))
        z = max(-3.95, min(3.95, z))

        if x <= 0:
            col_xs = self.xs_left
            col_offset = 0
        else:
            col_xs = self.xs_right
            col_offset = len(self.xs_left)

        ix = int(np.searchsorted(col_xs, x) - 1)
        ix = max(0, min(len(col_xs) - 2, ix))
        u = max(0.0, min(1.0, (x - col_xs[ix]) / (col_xs[ix + 1] - col_xs[ix])))
        global_ix = col_offset + ix

        iz = int(np.searchsorted(self.zs, z) - 1)
        iz = max(0, min(len(self.zs) - 2, iz))
        v = max(0.0, min(1.0, (z - self.zs[iz]) / (self.zs[iz + 1] - self.zs[iz])))

        iy = 2 if y >= 1.25 else 1

        c00 = self.idx_map[(global_ix, iy, iz)]
        c10 = self.idx_map[(global_ix + 1, iy, iz)]
        c01 = self.idx_map[(global_ix, iy, iz + 1)]
        c11 = self.idx_map[(global_ix + 1, iy, iz + 1)]

        return [
            (c00, (1.0 - u) * (1.0 - v)),
            (c10, u * (1.0 - v)),
            (c01, (1.0 - u) * v),
            (c11, u * v),
        ]

    def compute_point_from_bary(self, skin_pos, bary_weights):
        """Reconstruit la position 3D exacte d'un point sur le maillage déformé."""
        pt = np.zeros(3)
        for idx, w in bary_weights:
            pt += w * np.array(skin_pos[idx])
        return [float(pt[0]), float(pt[1]), float(pt[2])]

    def _get_local_gap_mm(self, z_coord):
        """Mesure l'écartement réel (en mm) entre les deux lèvres de la fente à la coordonnée Z."""
        if self.skin_dofs is None:
            return 2.0
        skin_pos = self.skin_dofs.position.value
        iz = int(np.argmin(np.abs(self.zs - z_coord)))
        vl = self.idx_map.get((4, 2, iz), 0)
        vr = self.idx_map.get((5, 2, iz), 0)
        gap_cm = skin_pos[vr][0] - skin_pos[vl][0]
        return float(max(0.0, gap_cm * 10.0))

    def _sync_needle_and_thread(self, dt=0.02):
        """Synchronise l'aiguille, les trous de piqûre, les nœuds permanents et le fil."""
        quat = self._compute_quaternion()
        if self.needle_dofs is not None:
            self.needle_dofs.position.value = [self.pos + quat]

        skin_pos = self.skin_dofs.position.value if self.skin_dofs is not None else None

        # 1. Mise à jour des nœuds chirurgicaux finalisés et de leurs orifices (Option 2 & 3)
        if skin_pos is not None:
            for knot in self.completed_knots:
                k_idx = knot['knot_idx']
                p_l = self.compute_point_from_bary(skin_pos, knot['bary_left'])
                p_r = self.compute_point_from_bary(skin_pos, knot['bary_right'])
                if k_idx < len(self.knot_topos):
                    pts, lines = make_knot_mesh_data(p_l, p_r)
                    self.knot_topos[k_idx].position.value = pts
                    self.knot_topos[k_idx].lines.value = lines
                # Orifices permanents aux extrémités du point noué
                if 2 * k_idx + 1 < len(self.knot_hole_dofs_list):
                    self.knot_hole_dofs_list[2 * k_idx].position.value = [
                        [p_l[0], p_l[1] + 0.005, p_l[2], 0.0, 0.0, 0.0, 1.0]
                    ]
                    self.knot_hole_dofs_list[2 * k_idx + 1].position.value = [
                        [p_r[0], p_r[1] + 0.005, p_r[2], 0.0, 0.0, 0.0, 1.0]
                    ]

        # 2. Mise à jour des marqueurs visuels des trous de piqûre du point actif
        if skin_pos is not None:
            for i, dof in enumerate(self.hole_dofs_list):
                if i < len(self.holes):
                    hp = self.compute_point_from_bary(skin_pos, self.holes[i]['bary'])
                    y_offset = 0.005 if self.holes[i]['type'] in ['entry_top', 'exit_top'] else 0.0
                    dof.position.value = [[hp[0], hp[1] + y_offset, hp[2], 0.0, 0.0, 0.0, 1.0]]
                else:
                    dof.position.value = [[0.0, -10.0, 0.0, 0.0, 0.0, 0.0, 1.0]]

        # 3. Mise à jour physique et géométrique du fil de suture actif
        eye_pos = self._get_needle_eye_world_pos(quat)
        if self.thread_dofs is not None:
            th_pos = list(self.thread_dofs.position.value)
            th_rest = list(self.thread_dofs.rest_position.value)
            th_vel = list(self.thread_dofs.velocity.value)

            n_nodes = len(th_pos)
            if n_nodes > 0:
                if len(th_rest) < n_nodes:
                    th_rest = list(th_pos)
                if len(th_vel) < n_nodes:
                    th_vel = [[0.0, 0.0, 0.0] for _ in range(n_nodes)]

                th_pos[0] = eye_pos
                th_rest[0] = eye_pos

                n_holes = len(self.holes)
                if n_holes >= 1 and skin_pos is not None:
                    hole_pts = [self.compute_point_from_bary(skin_pos, h['bary']) for h in self.holes]

                    if n_holes == 1:
                        th_pos[14] = hole_pts[0]
                        th_rest[14] = hole_pts[0]

                    elif n_holes == 2:
                        p1 = np.array(hole_pts[0])
                        p2 = np.array(hole_pts[1])
                        th_pos[14] = list(p1)
                        th_rest[14] = list(p1)
                        th_pos[7] = list(p2)
                        th_rest[7] = list(p2)

                        slack = max(0.01, 0.12 * (1.0 - self.suture_tension))
                        for k in range(7, 15):
                            alpha = (k - 7) / 7.0
                            pk = (1.0 - alpha) * p2 + alpha * p1
                            pk[1] -= slack * math.sin(math.pi * alpha)
                            th_pos[k] = list(pk)
                            th_rest[k] = list(pk)
                            if len(th_vel) > k:
                                th_vel[k] = [0.0, 0.0, 0.0]

                    elif n_holes == 3:
                        p1 = np.array(hole_pts[0])
                        p2 = np.array(hole_pts[1])
                        p3 = np.array(hole_pts[2])
                        th_pos[14] = list(p1)
                        th_rest[14] = list(p1)
                        th_pos[11] = list(p2)
                        th_rest[11] = list(p2)
                        th_pos[8] = list(p3)
                        th_rest[8] = list(p3)

                        slack = max(0.01, 0.08 * (1.0 - self.suture_tension))
                        for k in range(11, 15):
                            alpha = (k - 11) / 3.0
                            pk = (1.0 - alpha) * p2 + alpha * p1
                            pk[1] -= slack * math.sin(math.pi * alpha)
                            th_pos[k] = list(pk)
                            th_rest[k] = list(pk)

                        for k in range(8, 12):
                            alpha = (k - 8) / 3.0
                            pk = (1.0 - alpha) * p3 + alpha * p2
                            th_pos[k] = list(pk)
                            th_rest[k] = list(pk)

                    elif n_holes >= 4:
                        p1 = np.array(hole_pts[0])
                        p2 = np.array(hole_pts[1])
                        p3 = np.array(hole_pts[2])
                        p4 = np.array(hole_pts[3])

                        slack = max(0.01, 0.06 * (1.0 - self.suture_tension))
                        for k in range(12, 15):
                            alpha = (k - 12) / 2.0
                            pk = (1.0 - alpha) * p2 + alpha * p1
                            pk[1] -= slack * math.sin(math.pi * alpha)
                            th_pos[k] = list(pk)
                            th_rest[k] = list(pk)

                        for k in range(10, 13):
                            alpha = (k - 10) / 2.0
                            pk = (1.0 - alpha) * p3 + alpha * p2
                            th_pos[k] = list(pk)
                            th_rest[k] = list(pk)

                        for k in range(7, 11):
                            alpha = (k - 7) / 3.0
                            pk = (1.0 - alpha) * p4 + alpha * p3
                            pk[1] -= slack * math.sin(math.pi * alpha)
                            th_pos[k] = list(pk)
                            th_rest[k] = list(pk)

                        for k in range(7, 15):
                            if len(th_vel) > k:
                                th_vel[k] = [0.0, 0.0, 0.0]

                self.thread_dofs.position.value = th_pos
                self.thread_dofs.rest_position.value = th_rest

                if self.prev_eye_pos is not None and dt > 1e-6 and len(th_vel) > 0:
                    th_vel[0] = [(eye_pos[m] - self.prev_eye_pos[m]) / dt for m in range(3)]
                    self.thread_dofs.velocity.value = th_vel

        self.prev_eye_pos = eye_pos

    def _update_all_traction_forces(self):
        """
        Régulation de coaptation dermique multi-couches amortie par tranche Z :
        - Rapproche les berges proportionnellement à la tension (de 2.0 mm à ~0.04 mm).
        - Répartit l'effort sur le derme superficiel (iy=2) et profond (iy=1) ainsi que
          sur la largeur des berges (ix=2,3,4 et ix=5,6,7) pour éviter toute distorsion
          ou inversion de tétraèdre.
        - Amortissement en vitesse intégré pour une fermeture parfaitement stable sans oscillation
          et sans croisement des lèvres (gap >= 0.0 mm), même avec 6 nœuds simultanés.
        """
        if self.traction_cff is None or self.skin_dofs is None:
            return

        skin_pos = self.skin_dofs.position.value
        skin_vel = self.skin_dofs.velocity.value
        target_tension_per_row = {iz: 0.0 for iz in range(len(self.zs))}

        def distribute_stitch_tension(z_val, tens):
            for iz, z_grid in enumerate(self.zs):
                if abs(z_grid) <= 2.05:
                    dist_z = abs(z_val - z_grid)
                    if dist_z <= 0.80:
                        weight = max(0.0, 1.0 - (dist_z / 0.80) * 0.45)
                        target_tension_per_row[iz] = max(target_tension_per_row[iz], tens * weight)

        # 1. Contributions des nœuds chirurgicaux verrouillés
        for knot in self.completed_knots:
            distribute_stitch_tension(knot['z_pos'], knot['tension'])

        # 2. Contribution du point actif sous tension
        if self.suture_attached and len(self.holes) >= 2 and self.suture_tension > 0.001:
            z_active = 0.5 * (self.holes[0]['pos'][2] + self.holes[-1]['pos'][2])
            distribute_stitch_tension(z_active, self.suture_tension)

        all_indices = []
        all_forces = []

        for iz, tens in target_tension_per_row.items():
            if tens <= 0.001:
                self.row_forces[iz] = 0.0
                continue

            vl_lip = self.idx_map.get((4, 2, iz), 0)
            vr_lip = self.idx_map.get((5, 2, iz), 0)
            vl_hole = self.idx_map.get((3, 2, iz), 0)
            vr_hole = self.idx_map.get((6, 2, iz), 0)
            vl_outer = self.idx_map.get((2, 2, iz), 0)
            vr_outer = self.idx_map.get((7, 2, iz), 0)
            vl_deep = self.idx_map.get((3, 1, iz), 0)
            vr_deep = self.idx_map.get((6, 1, iz), 0)

            current_gap_cm = float(skin_pos[vr_lip][0] - skin_pos[vl_lip][0])
            closing_vel = float(skin_vel[vl_lip][0] - skin_vel[vr_lip][0]) if len(skin_vel) > vr_lip else 0.0

            # Écart cible : 0.20 cm au repos (tens=0) -> 0.005 cm (0.05 mm, coaptation franche sans croisement) à tens=1
            target_gap_cm = 0.20 * (1.0 - 0.975 * min(1.0, tens))
            gap_error_cm = current_gap_cm - target_gap_cm

            # Intégrateur doux borné anti-windup + retour proportionnel-dérivé amorti
            F_int = float(np.clip(self.row_forces.get(iz, 0.0) + 22.0 * gap_error_cm, -35.0, 95.0))
            self.row_forces[iz] = F_int

            F_ff = 135.0 * min(1.0, tens)
            F_total = F_ff + F_int + 290.0 * gap_error_cm - 14.0 * closing_vel
            if current_gap_cm < 0.004:
                # Barrière de contact unilatérale empêchant tout chevauchement des lèvres
                F_total = min(F_total, max(0.0, F_ff * 0.35))
            F_total = float(np.clip(F_total, 0.0, 195.0))

            if F_total > 0.01:
                f_lip = 0.28 * F_total
                f_hole = 0.42 * F_total
                f_outer = 0.15 * F_total
                f_deep = 0.15 * F_total
                all_indices.extend([vl_lip, vr_lip, vl_hole, vr_hole, vl_outer, vr_outer, vl_deep, vr_deep])
                all_forces.extend([
                    [f_lip, 0.0, 0.0],
                    [-f_lip, 0.0, 0.0],
                    [f_hole, 0.0, 0.0],
                    [-f_hole, 0.0, 0.0],
                    [f_outer, 0.0, 0.0],
                    [-f_outer, 0.0, 0.0],
                    [f_deep, 0.0, 0.0],
                    [-f_deep, 0.0, 0.0],
                ])

        if len(all_indices) > 0:
            self.traction_cff.indices.value = all_indices
            self.traction_cff.forces.value = all_forces
        else:
            self.traction_cff.indices.value = [0]
            self.traction_cff.forces.value = [[0.0, 0.0, 0.0]]

    def _knot_current_stitch(self):
        """
        Réalise le nœud chirurgical (Option 2) et prépare le point suivant (Option 3).
        - Verrouille la tension de fermeture de manière permanente.
        - Crée le nœud chirurgical à double boucle, brins coupés et orifices permanents.
        - Coupe le fil et réarme l'aiguille pour le point suivant !
        """
        if not self.suture_attached or len(self.holes) < 2:
            print("[INFO] Passez d'abord l'aiguille à travers les berges avant de nouer [N].")
            return

        if len(self.completed_knots) >= MAX_KNOTS:
            print(f"[INFO] Nombre maximal de points noués atteint ({MAX_KNOTS} points).")
            return

        h_first = self.holes[0]
        h_last = self.holes[-1]
        h_left = h_first if h_first['pos'][0] <= h_last['pos'][0] else h_last
        h_right = h_last if h_last['pos'][0] >= h_first['pos'][0] else h_first
        z_stitch = 0.5 * (h_left['pos'][2] + h_right['pos'][2])

        # Tension verrouillée (au moins 95% lors du serrage du nœud)
        knot_tension = max(0.95, self.suture_tension)
        knot_idx = len(self.completed_knots)

        self.completed_knots.append({
            'bary_left': h_left['bary'],
            'bary_right': h_right['bary'],
            'knot_idx': knot_idx,
            'z_pos': z_stitch,
            'tension': knot_tension,
        })

        # Mise à jour immédiate des forces et de la géométrie du nœud
        self._update_all_traction_forces()
        if self.skin_dofs and knot_idx < len(self.knot_topos):
            skin_pos = self.skin_dofs.position.value
            p_l = self.compute_point_from_bary(skin_pos, h_left['bary'])
            p_r = self.compute_point_from_bary(skin_pos, h_right['bary'])
            pts, lines = make_knot_mesh_data(p_l, p_r)
            self.knot_topos[knot_idx].position.value = pts
            self.knot_topos[knot_idx].lines.value = lines

        gap_mm = self._get_local_gap_mm(z_stitch)

        # Réarmement de l'aiguille et du fil pour le point suivant (Option 3)
        self.holes = []
        self.puncture_state = "AIR_OUTSIDE"
        self.suture_attached = False
        self.suture_tension = 0.0
        self.last_reported_tension = -1.0
        self.stitch_initial_eye_dist = None
        self.min_ty_since_entry = 10.0

        # Repositionnement ergonomique pour le point suivant (+0.75 cm le long de l'incision)
        self.pos = [0.15, 2.12, min(1.80, z_stitch + 0.75)]
        self.suture_angle = self.init_suture_angle
        self.yaw_angle = 0.0
        self.pitch_angle = 0.0

        # Délai de 5 frames avant réactivation de la collision pour laisser le RigidMapping
        # propager la nouvelle position en l'air sans choc de téléportation
        if self.needle_sphere_col:
            self.needle_sphere_col.active.value = False
        self.col_reenable_cooldown = 5

        if self.thread_fix:
            self.thread_fix.indices.value = [0]

        # Réinitialisation du fil actif qui pend du chas de l'aiguille
        if self.thread_dofs:
            eye_p = self._get_needle_eye_world_pos(self._compute_quaternion())
            n_pts = len(self.thread_dofs.position.value)
            new_pos = []
            for i in range(n_pts):
                t = i / float(n_pts - 1)
                px = eye_p[0] + 0.15 * t
                py = eye_p[1] - (self.thread_length * 0.90) * t
                pz = eye_p[2] + (self.thread_length * 0.35) * t
                new_pos.append([px, py, pz])
            self.thread_dofs.position.value = new_pos
            self.thread_dofs.rest_position.value = new_pos
            self.thread_dofs.velocity.value = [[0.0, 0.0, 0.0] for _ in range(n_pts)]

        # Masquage des marqueurs temporaires (les marqueurs permanents KnotHole ont pris le relais)
        for dof in self.hole_dofs_list:
            dof.position.value = [[0.0, -10.0, 0.0, 0.0, 0.0, 0.0, 1.0]]

        self._sync_needle_and_thread(dt=0.02)

        total_knots = len(self.completed_knots)
        print("*" * 82)
        print(f"  [NŒUD CHIRURGICAL #{total_knots} VERROUILLÉ] >>> Point noué avec succès à Z = {z_stitch:.2f} cm !")
        print(f"                                          Écart local des berges : {gap_mm:.2f} mm")
        print("  >>> NŒUD PLAT À DOUBLE BOUCLE & BRINS COUPÉS ANCRÉS SUR L'ÉPIDERME.")
        print("  >>> AIGUILLE RÉARMÉE ET AVANCÉE POUR LE POINT SUIVANT !")
        print(f"      (Points posés : {total_knots}/{MAX_KNOTS} - Appuyez sur [F]/[E] pour piquer le point suivant)")
        print("*" * 82)

    def _apply_arc_guidance(self):
        """
        Accompagne le centre de courbure de l'aiguille vers sa hauteur d'arc chirurgicale
        optimale (Y = 2.16 cm, X ~ 0.05 cm) lors de l'enroulement dans les tissus.
        Garantit mathématiquement que la pointe émerge TOUJOURS au-dessus de Y = 1.50 cm !
        """
        if self.puncture_state in ["IN_LEFT_TISSUE", "IN_FENTE_AIR", "IN_RIGHT_TISSUE"]:
            if self.pos[1] < 2.16:
                self.pos[1] = min(2.16, self.pos[1] + max(0.032, (2.16 - self.pos[1]) * 0.14))
            if len(self.holes) >= 1:
                h1_x = self.holes[0]['pos'][0]
                if h1_x < -0.10 and abs(self.pos[0]) < 0.85:
                    target_x = 0.05
                    if abs(self.pos[0] - target_x) > 0.015:
                        step_x = 0.012 if self.pos[0] < target_x else -0.012
                        self.pos[0] += step_x
                elif h1_x > 0.10 and abs(self.pos[0]) < 0.85:
                    target_x = -0.05
                    if abs(self.pos[0] - target_x) > 0.015:
                        step_x = 0.012 if self.pos[0] < target_x else -0.012
                        self.pos[0] += step_x

    def onAnimateBeginEvent(self, event):
        """Boucle temps réel : contrôles, guidage d'arc, percée/sortie, tension et nœuds."""
        if not self._initialized:
            self._init_controller()

        rolling_forward = False
        raising_up = False

        # 1. Mode souris via TAB
        tab_down = is_key_pressed(0x09)
        if tab_down and not self.prev_tab_down:
            self.mouse_mode = not self.mouse_mode
            status = "ACTIVÉ (L'aiguille suit la souris)" if self.mouse_mode else "DÉSACTIVÉ (Caméra libre)"
            print(f"[SOURIS] Mode Guidage Aiguille -> {status}")
        self.prev_tab_down = tab_down

        mx, my = get_mouse_pos()
        if self.prev_mouse_x is None:
            self.prev_mouse_x = mx
            self.prev_mouse_y = my

        dx = mx - self.prev_mouse_x
        dy = my - self.prev_mouse_y
        self.prev_mouse_x = mx
        self.prev_mouse_y = my

        if self.mouse_mode:
            left_click = is_key_pressed(0x01)
            right_click = is_key_pressed(0x02)
            middle_click = is_key_pressed(0x04)

            if right_click:
                if dy != 0:
                    self.pos[1] -= dy * 0.008
                    self.pos[1] = max(0.9, min(4.5, self.pos[1]))
                    if dy < 0:
                        raising_up = True
            else:
                if dx != 0 or dy != 0:
                    self.pos[0] += dx * 0.008
                    self.pos[2] += dy * 0.008
                    self.pos[0] = max(-3.5, min(3.5, self.pos[0]))
                    self.pos[2] = max(-3.5, min(3.5, self.pos[2]))

            if left_click:
                self.suture_angle += self.speed_rot
                rolling_forward = True

            if middle_click:
                self.suture_angle -= self.speed_rot

        # 2. Clavier continu pour l'aiguille
        if is_key_pressed(0x25) or is_key_pressed(0x51):  # Gauche : Flèche Gauche ou Q
            self.pos[0] = max(-3.5, self.pos[0] - self.speed_trans)
        if is_key_pressed(0x27) or is_key_pressed(0x44):  # Droite : Flèche Droite ou D
            self.pos[0] = min(3.5, self.pos[0] + self.speed_trans)
        if is_key_pressed(0x26) or is_key_pressed(0x5A) or is_key_pressed(0x57):  # Avancer : Flèche Haut ou Z / W
            self.pos[2] = max(-3.5, self.pos[2] - self.speed_trans)
        if is_key_pressed(0x28) or is_key_pressed(0x53):  # Reculer : Flèche Bas ou S
            self.pos[2] = min(3.5, self.pos[2] + self.speed_trans)
        if is_key_pressed(0x52):  # Monter : R
            self.pos[1] = min(4.5, self.pos[1] + self.speed_trans)
            raising_up = True
        if is_key_pressed(0x46) or is_key_pressed(0x43):  # Descendre : F ou C
            self.pos[1] = max(0.9, self.pos[1] - self.speed_trans)
        if is_key_pressed(0x45):  # Enrouler le long de l'arc : E
            self.suture_angle += self.speed_rot
            rolling_forward = True
        if is_key_pressed(0x41) or is_key_pressed(0x08):  # Dérouler : A ou Retour
            self.suture_angle -= self.speed_rot
        if is_key_pressed(0x4A):  # Lacet gauche : J
            self.yaw_angle += self.speed_rot * 0.8
        if is_key_pressed(0x4C):  # Lacet droite : L
            self.yaw_angle -= self.speed_rot * 0.8
        if is_key_pressed(0x49):  # Tangage avant : I
            self.pitch_angle += self.speed_rot * 0.6
        if is_key_pressed(0x4B):  # Tangage arrière : K
            self.pitch_angle -= self.speed_rot * 0.6

        # Touche U (0x55) : Réaligner l'orientation de la pince
        u_down = is_key_pressed(0x55)
        if u_down and not self.prev_u_down:
            self.yaw_angle = 0.0
            self.pitch_angle = 0.0
            print("[PINCE] Orientation lacet/tangage réalignée à 0°.")
        self.prev_u_down = u_down

        # GUIDAGE CINÉMATIQUE D'ARC LORS DE L'ENROULEMENT [E] OU DE LA REMONTÉE [R] DANS LES TISSUS :
        if rolling_forward:
            self._apply_arc_guidance()
        elif raising_up and self.puncture_state in ["IN_LEFT_TISSUE", "IN_FENTE_AIR", "IN_RIGHT_TISSUE"]:
            # Si l'utilisateur appuie sur [R] pour faire ressortir la pointe vers le haut,
            # on oriente également la pointe vers la surface pour faciliter sa percée de sortie.
            if self.suture_angle < 142.0:
                self.suture_angle += self.speed_rot * 0.85

        # 3. OPTION 1 : CONTRÔLE DE LA TENSION DU FIL (SERRER / DESSERRER)
        if self.suture_attached:
            # Touche V (0x56) ou Espace (0x20) : Serrer la suture
            if is_key_pressed(0x56) or is_key_pressed(0x20):
                self.suture_tension = min(1.0, self.suture_tension + 0.025)

            # Touche B (0x42) : Desserrer la suture
            if is_key_pressed(0x42):
                self.suture_tension = max(0.0, self.suture_tension - 0.025)

            # Traction automatique en éloignant l'aiguille de l'orifice de sortie
            quat_cur = self._compute_quaternion()
            eye_cur = self._get_needle_eye_world_pos(quat_cur)
            if len(self.holes) >= 2:
                exit_pt = self.holes[-1]['pos']
                dist_exit = math.sqrt(sum((eye_cur[m] - exit_pt[m]) ** 2 for m in range(3)))
                if self.stitch_initial_eye_dist is None:
                    self.stitch_initial_eye_dist = dist_exit
                elif dist_exit > self.stitch_initial_eye_dist + 0.05:
                    delta = (dist_exit - self.stitch_initial_eye_dist) / 0.80
                    auto_tens = min(1.0, max(0.0, delta))
                    if auto_tens > self.suture_tension:
                        self.suture_tension = auto_tens

            # Retour télémétrique en direct dans la console à chaque palier de 10%
            if abs(self.suture_tension - self.last_reported_tension) >= 0.10 or (
                self.suture_tension >= 0.99 and self.last_reported_tension < 0.99
            ):
                self.last_reported_tension = self.suture_tension
                z_cur = 0.5 * (self.holes[0]['pos'][2] + self.holes[-1]['pos'][2])
                gap_now = self._get_local_gap_mm(z_cur)
                pct = int(round(self.suture_tension * 100))
                etat = "BERGES COAPTÉES (Prêt à nouer [N])" if gap_now <= 0.15 else "Rapprochement en cours..."
                print(f"  [TENSION DU FIL : {pct:3d}%] >>> Écart local de la fente : {gap_now:.2f} mm | {etat}")

        # 4. OPTION 2 : NOUER LE POINT DE SUTURE (TOUCHE N - 0x4E)
        n_down = is_key_pressed(0x4E)
        if n_down and not self.prev_n_down:
            self._knot_current_stitch()
        self.prev_n_down = n_down

        # 5. DÉTECTION CHIRURGICALE MULTI-SURFACES (PERÇAGE SURFACE, PAROIS DE FENTE & SORTIE)
        quat = self._compute_quaternion()
        tip_w = self._get_needle_tip_world_pos(quat)
        tx, ty, tz = tip_w[0], tip_w[1], tip_w[2]

        # Gestion sécurisée de la réactivation de collision après téléportation de réarmement
        if self.col_reenable_cooldown > 0:
            self.col_reenable_cooldown -= 1
            if self.col_reenable_cooldown == 0 and self.puncture_state == "AIR_OUTSIDE" and self.needle_sphere_col:
                self.needle_sphere_col.active.value = True
                self.needle_sphere_col.radius.value = 0.04
                self.needle_sphere_col.contactDistance.value = 0.02

        is_over_skin = abs(tx) <= 3.85 and abs(tz) <= 3.85
        is_in_fente_gap = (abs(tx) < 0.11) and (ty >= FENTE_FLOOR_Y - 0.05) and (abs(tz) <= 2.05)

        if self.puncture_state != "AIR_OUTSIDE":
            self.min_ty_since_entry = min(self.min_ty_since_entry, ty)

        # --- ÉTAPE 1 : PERCÉE INITIALE SUR LA SURFACE EXTERNE ---
        if self.puncture_state == "AIR_OUTSIDE" and is_over_skin and not is_in_fente_gap:
            # Pression chirurgicale ferme requise : indentation jusqu'à ty <= 1.34 cm
            if ty <= 1.34:
                bary = self.get_barycentric_weights(tx, SKIN_TOP_Y, tz)
                side_name = "GAUCHE" if tx <= 0 else "DROITE"
                self.holes.append({'pos': [tx, SKIN_TOP_Y, tz], 'bary': bary, 'type': 'entry_top'})
                self.puncture_state = "IN_LEFT_TISSUE" if tx <= 0 else "IN_RIGHT_TISSUE"
                self.min_ty_since_entry = ty
                self.entry_angle_deg = self.suture_angle

                # Désactive totalement la collision de l'aiguille dans les tissus :
                # élimine 100% du soulèvement/étirement parasite de la peau à la remontée
                if self.needle_sphere_col:
                    self.needle_sphere_col.active.value = False

                if self.thread_fix:
                    self.thread_fix.indices.value = [0, 14]

                print("=" * 82)
                print(f"  [PERCÉE 1 - ÉPIDERME {side_name}] >>> Pointe entrée à (X={tx:.2f}, Z={tz:.2f}) cm.")
                print("                                   Maintenez [E] pour enrouler et faire ressortir la pointe.")
                print("=" * 82)

        # --- ÉTAPE 2 : TRAVERSÉE DU TISSU GAUCHE ---
        elif self.puncture_state == "IN_LEFT_TISSUE":
            h1 = self.holes[0]['pos']
            dist_xz = math.sqrt((tx - h1[0]) ** 2 + (tz - h1[2]) ** 2)
            angle_span = abs(self.suture_angle - self.entry_angle_deg)

            # Cas A : L'aiguille ressort sur la paroi interne gauche de la fente
            if h1[0] < -0.11 and (-0.10 <= tx <= 0.09) and (0.60 <= ty <= 1.45) and abs(tz) <= 2.10:
                y_wall = max(FENTE_FLOOR_Y, min(1.42, ty))
                bary = self.get_barycentric_weights(-0.10, y_wall, tz)
                self.holes.append({'pos': [-0.10, y_wall, tz], 'bary': bary, 'type': 'exit_fente'})
                self.puncture_state = "IN_FENTE_AIR"

                if self.thread_fix:
                    self.thread_fix.indices.value = [0, 11, 14]

                print("=" * 82)
                print(f"  [SORTIE - PAROI INTERNE GAUCHE] >>> La pointe émerge dans la fente (Y={y_wall:.2f} cm) !")
                print("                                      Continuez [E] pour piquer la paroi opposée.")
                print("=" * 82)

            # Cas B : Passage sous la fente vers la berge droite
            elif h1[0] < 0 and tx >= 0.10:
                self.puncture_state = "IN_RIGHT_TISSUE"

            # Cas C : Émergence à la surface supérieure (retour droite->gauche ou suture libre sur berge gauche)
            elif ty >= 1.40 and (
                (h1[0] > 0.10 and tx <= -0.12)
                or (dist_xz >= 0.25 and (self.min_ty_since_entry <= 1.28 or angle_span >= 25.0))
            ):
                bary = self.get_barycentric_weights(tx, SKIN_TOP_Y, tz)
                self.holes.append({'pos': [tx, SKIN_TOP_Y, tz], 'bary': bary, 'type': 'exit_top'})
                self.puncture_state = "COMPLETED"
                self.suture_attached = True
                self._finalize_suture()

        # --- ÉTAPE 3 : DANS L'ESPACE DE LA FENTE -> PERCÉE DE LA PAROI OPPOSÉE ---
        elif self.puncture_state == "IN_FENTE_AIR":
            h1 = self.holes[0]['pos']
            # Sens Gauche -> Droite : percée de la paroi interne droite
            if h1[0] <= 0 and tx >= 0.10 and abs(tz) <= 2.15:
                y_wall = max(FENTE_FLOOR_Y, min(1.42, ty))
                bary = self.get_barycentric_weights(0.10, y_wall, tz)
                self.holes.append({'pos': [0.10, y_wall, tz], 'bary': bary, 'type': 'entry_fente'})
                self.puncture_state = "IN_RIGHT_TISSUE"

                if self.thread_fix:
                    self.thread_fix.indices.value = [0, 8, 11, 14]

                print("=" * 82)
                print(f"  [PERCÉE - PAROI INTERNE DROITE] >>> La pointe pénètre la berge droite (Y={y_wall:.2f} cm) !")
                print("                                      Continuez [E] pour ressortir à la surface.")
                print("=" * 82)

            # Sens Droite -> Gauche : percée de la paroi interne gauche
            elif h1[0] > 0 and tx <= -0.10 and abs(tz) <= 2.15:
                y_wall = max(FENTE_FLOOR_Y, min(1.42, ty))
                bary = self.get_barycentric_weights(-0.10, y_wall, tz)
                self.holes.append({'pos': [-0.10, y_wall, tz], 'bary': bary, 'type': 'entry_fente'})
                self.puncture_state = "IN_LEFT_TISSUE"

                if self.thread_fix:
                    self.thread_fix.indices.value = [0, 8, 11, 14]

                print("=" * 82)
                print(f"  [PERCÉE - PAROI INTERNE GAUCHE] >>> La pointe pénètre la berge gauche (Y={y_wall:.2f} cm) !")
                print("                                      Continuez [E] pour ressortir à la surface.")
                print("=" * 82)

        # --- ÉTAPE 4 : DANS LE TISSU DROIT -> ÉMERGENCE SUR L'ÉPIDERME ---
        elif self.puncture_state == "IN_RIGHT_TISSUE":
            h1 = self.holes[0]['pos']
            dist_xz = math.sqrt((tx - h1[0]) ** 2 + (tz - h1[2]) ** 2)
            angle_span = abs(self.suture_angle - self.entry_angle_deg)

            # Cas A : Si entré à droite, sortie sur la paroi interne droite de la fente
            if h1[0] > 0.11 and (-0.09 <= tx <= 0.10) and (0.60 <= ty <= 1.45) and abs(tz) <= 2.10:
                y_wall = max(FENTE_FLOOR_Y, min(1.42, ty))
                bary = self.get_barycentric_weights(0.10, y_wall, tz)
                self.holes.append({'pos': [0.10, y_wall, tz], 'bary': bary, 'type': 'exit_fente'})
                self.puncture_state = "IN_FENTE_AIR"

                if self.thread_fix:
                    self.thread_fix.indices.value = [0, 11, 14]

                print("=" * 82)
                print(f"  [SORTIE - PAROI INTERNE DROITE] >>> La pointe émerge dans la fente (Y={y_wall:.2f} cm) !")
                print("                                      Continuez pour piquer la paroi opposée.")
                print("=" * 82)

            # Cas B : Si entré à droite et passage profond vers la berge gauche
            elif h1[0] > 0.11 and tx <= -0.10:
                self.puncture_state = "IN_LEFT_TISSUE"

            # Cas C : Émergence franche à la surface supérieure (contre-ponction berge droite ou suture libre)
            elif ty >= 1.40 and (
                (h1[0] < -0.10 and tx >= 0.12)
                or (dist_xz >= 0.25 and (self.min_ty_since_entry <= 1.28 or angle_span >= 25.0))
            ):
                bary = self.get_barycentric_weights(tx, SKIN_TOP_Y, tz)
                self.holes.append({'pos': [tx, SKIN_TOP_Y, tz], 'bary': bary, 'type': 'exit_top'})
                self.puncture_state = "COMPLETED"
                self.suture_attached = True
                self._finalize_suture()

        # 6. ASSISTANCE CHIRURGICALE MANUELLE : TOUCHE T (0x54)
        t_down = is_key_pressed(0x54)
        if t_down and not self.prev_t_down:
            if self.puncture_state == "AIR_OUTSIDE":
                bary = self.get_barycentric_weights(tx, SKIN_TOP_Y, tz)
                self.holes.append({'pos': [tx, SKIN_TOP_Y, tz], 'bary': bary, 'type': 'entry_top'})
                self.puncture_state = "IN_LEFT_TISSUE" if tx <= 0 else "IN_RIGHT_TISSUE"
                self.min_ty_since_entry = ty
                self.entry_angle_deg = self.suture_angle
                if self.needle_sphere_col:
                    self.needle_sphere_col.active.value = False
                if self.thread_fix:
                    self.thread_fix.indices.value = [0, 14]
                print(f"[ASSISTANCE T] Percée d'entrée créée à (X={tx:.2f}, Z={tz:.2f}) cm.")
            elif self.puncture_state in ["IN_LEFT_TISSUE", "IN_FENTE_AIR", "IN_RIGHT_TISSUE"]:
                exit_x = tx if abs(tx - self.holes[0]['pos'][0]) >= 0.30 else -self.holes[0]['pos'][0]
                bary = self.get_barycentric_weights(exit_x, SKIN_TOP_Y, tz)
                self.holes.append({'pos': [exit_x, SKIN_TOP_Y, tz], 'bary': bary, 'type': 'exit_top'})
                self.puncture_state = "COMPLETED"
                self.suture_attached = True
                self._finalize_suture()
                print(f"[ASSISTANCE T] Contre-ponction de sortie validée à (X={exit_x:.2f}, Z={tz:.2f}) cm.")
            else:
                print("[INFO] Point déjà ressorti : maintenez [V] pour serrer les berges puis [N] pour nouer.")
        self.prev_t_down = t_down

        # 7. RÉINITIALISATION TOTALE : TOUCHE X (0x58)
        if is_key_pressed(0x58):
            self._reset_simulation()

        # 8. Mise à jour des forces de coaptation bilatérale (Fermeture berges)
        self._update_all_traction_forces()

        # 9. Synchronisation complète aiguille, peau, fil et nœuds
        self._sync_needle_and_thread(dt=0.02)

    def _finalize_suture(self):
        """Finalise la traversée de la suture, fait jaillir la pointe hors de l'épiderme et active la tension."""
        # Garantit que la pointe émerge visuellement de 1.3+ mm AU-DESSUS de l'épiderme (Y >= 1.63 cm)
        if self.pos[1] < 2.16:
            self.pos[1] = 2.16
        if 110.0 <= self.suture_angle < 146.0:
            self.suture_angle = 146.0
        quat = self._compute_quaternion()
        tip_w = self._get_needle_tip_world_pos(quat)

        if self.thread_fix:
            self.thread_fix.indices.value = [0] + list(range(7, 15))

        # Collision maintenue désactivée tant que l'aiguille est engagée dans la suture
        if self.needle_sphere_col:
            self.needle_sphere_col.active.value = False

        self.suture_tension = 0.20
        self.last_reported_tension = 0.20

        z_s = 0.5 * (self.holes[0]['pos'][2] + self.holes[-1]['pos'][2])
        gap_mm = self._get_local_gap_mm(z_s)
        n_h = len(self.holes)

        print("=" * 82)
        print(f"  [CONTRE-PONCTION RÉUSSIE] >>> La pointe ressort de la peau à (X={tip_w[0]:.2f}, Y={tip_w[1]:.2f}, Z={tip_w[2]:.2f}) cm !")
        print(f"                                Orifices traversés par le fil : {n_h} | Écart fente : {gap_mm:.2f} mm")
        print("  >>> ÉTAPES SUIVANTES :")
        print("      1. Maintenez [V] ou [Espace] (ou tirez l'aiguille) pour SERRER et FERMER les berges.")
        print("      2. Appuyez sur [N] pour NOUER LE NŒUD CHIRURGICAL et passer au point suivant !")
        print("=" * 82)

    def _reset_simulation(self):
        """Réinitialise entièrement la simulation et efface tous les points noués."""
        self.pos = list(self.init_pos)
        self.suture_angle = self.init_suture_angle
        self.yaw_angle = 0.0
        self.pitch_angle = 0.0
        self.puncture_state = "AIR_OUTSIDE"
        self.holes = []
        self.suture_attached = False
        self.suture_tension = 0.0
        self.last_reported_tension = -1.0
        self.completed_knots = []
        self.stitch_initial_eye_dist = None
        self.min_ty_since_entry = 10.0
        self.row_forces = {iz: 0.0 for iz in range(len(self.zs))}

        if self.traction_cff:
            self.traction_cff.indices.value = [0]
            self.traction_cff.forces.value = [[0.0, 0.0, 0.0]]

        for topo in self.knot_topos:
            topo.position.value = [[0, -10, 0], [0, -10, 0]]
            topo.lines.value = [[0, 1]]

        for dof in self.hole_dofs_list:
            dof.position.value = [[0.0, -10.0, 0.0, 0.0, 0.0, 0.0, 1.0]]

        for kh_dof in self.knot_hole_dofs_list:
            kh_dof.position.value = [[0.0, -10.0, 0.0, 0.0, 0.0, 0.0, 1.0]]

        if self.needle_sphere_col:
            self.needle_sphere_col.active.value = True
            self.needle_sphere_col.radius.value = 0.04
            self.needle_sphere_col.contactDistance.value = 0.02

        if self.thread_fix:
            self.thread_fix.indices.value = [0]

        if self.thread_dofs:
            eye_p = self._get_needle_eye_world_pos(self._compute_quaternion())
            n_pts = len(self.thread_dofs.position.value)
            new_pos = []
            for i in range(n_pts):
                t = i / float(n_pts - 1)
                px = eye_p[0] + 0.15 * t
                py = eye_p[1] - (self.thread_length * 0.90) * t
                pz = eye_p[2] + (self.thread_length * 0.35) * t
                new_pos.append([px, py, pz])
            self.thread_dofs.position.value = new_pos
            self.thread_dofs.rest_position.value = new_pos
            self.thread_dofs.velocity.value = [[0.0, 0.0, 0.0] for _ in range(n_pts)]

        print("[INFO] Simulation réinitialisée : plaie rouverte (2.0 mm), tous les nœuds effacés.")


# =============================================================================
# CRÉATION DE LA SCÈNE SOFA
# =============================================================================
def createScene(root):
    """
    Scène de suture chirurgicale haute fidélité (Version Finale) :
    - Bloc de peau FEM corotationnel raffiné le long de l'incision de 4 cm.
    - Percée et émergence auto-guidées sur l'arc chirurgical sans étirement parasite.
    - Coaptation régulée en boucle fermée, nœuds doubles permanents et points multiples.
    """
    root.name.value = "root"
    root.dt.value = 0.02
    root.gravity.value = [0.0, -9.81, 0.0]

    # Plugins requis
    for p in REQUIRED_PLUGINS:
        root.addObject("RequiredPlugin", name=p.replace(".", "_"), pluginName=p)

    # 1. Boucle d'animation avec solveur de contraintes
    root.addObject("FreeMotionAnimationLoop")
    root.addObject("BlockGaussSeidelConstraintSolver", name="constraintSolver", maxIterations=500, tolerance=1e-4)

    # 2. Pipeline de détection de collision
    root.addObject("CollisionPipeline", name="pipeline")
    root.addObject("BruteForceBroadPhase", name="broadPhase")
    root.addObject("BVHNarrowPhase", name="narrowPhase")
    root.addObject("NewProximityIntersection", name="intersection", alarmDistance=0.08, contactDistance=0.02)
    root.addObject("CollisionResponse", name="response", response="FrictionContactConstraint", responseParams="mu=0.3")

    root.addObject("VisualStyle", displayFlags="showVisual hideBehaviorModels hideForceFields hideCollisionModels")

    # Fond bleu ciel opératoire
    root.addObject("BackgroundSetting", color=[0.55, 0.75, 0.95, 1.0])

    # Caméra vue de dessus en biais
    root.addObject(
        "InteractiveCamera",
        name="camera",
        position=[0.0, 5.0, 7.5],
        lookAt=[0.0, 0.75, 0.0],
        fieldOfView=45,
    )

    # Sol blanc
    floor = root.addChild("Floor")
    floor.addObject(
        "MeshTopology",
        name="topo",
        position="-12 -0.01 -12  12 -0.01 -12  12 -0.01 12  -12 -0.01 12",
        triangles="0 1 2  0 2 3",
    )
    floor.addObject("OglModel", name="visual", color=[0.96, 0.96, 0.96, 1.0])

    # =========================================================================
    # 1. BLOC DE PEAU DÉFORMABLE AVEC INCISION CENTRALE RAFFINÉE (4 CM)
    # =========================================================================
    skin = root.addChild("SkinBlock")

    skin.addObject("EulerImplicitSolver", name="odeSolver", rayleighStiffness=0.12, rayleighMass=0.12)
    skin.addObject("SparseLDLSolver", name="directSolver", template="CompressedRowSparseMatrixd")

    verts, tetras, idx_map, xs_left, xs_right, zs = make_skin_mesh_with_incision()
    skin.addObject("TetrahedronSetTopologyContainer", name="topo", position=verts, tetrahedra=tetras)
    skin.addObject("TetrahedronSetTopologyModifier", name="modifier")
    skin.addObject("TetrahedronSetGeometryAlgorithms", name="algo")

    skin.addObject("MechanicalObject", name="dofs", template="Vec3d", src="@topo")
    skin.addObject("MeshMatrixMass", name="mass", totalMass=0.15)
    skin.addObject(
        "TetrahedralCorotationalFEMForceField",
        name="FEM",
        youngModulus=3000,
        poissonRatio=0.45,
        method="polar",
    )

    skin.addObject("BoxROI", name="baseROI", box=[-4.1, -0.01, -4.1, 4.1, 0.01, 4.1])
    skin.addObject("FixedProjectiveConstraint", name="fixBase", indices="@baseROI.indices")
    skin.addObject("LinearSolverConstraintCorrection", linearSolver="@directSolver")

    # Champ de forces de coaptation bilatérale régulée pour fermer les berges
    skin.addObject("ConstantForceField", name="traction", indices=[0], forces=[[0.0, 0.0, 0.0]])

    # Modèle visuel de la peau (rose chair naturel)
    vis = skin.addChild("Visual")
    vis.addObject("TriangleSetTopologyContainer", name="visTopo")
    vis.addObject("TriangleSetTopologyModifier", name="modifier")
    vis.addObject("Tetra2TriangleTopologicalMapping", input="@../topo", output="@visTopo")
    vis.addObject("OglModel", name="visual", color=[0.93, 0.70, 0.65, 1.0])
    vis.addObject("IdentityMapping", input="@../dofs", output="@visual")

    # Modèle de collision de la peau
    skin_col = skin.addChild("CollisionSurface")
    skin_col.addObject("TriangleSetTopologyContainer", name="colTopo")
    skin_col.addObject("TriangleSetTopologyModifier", name="modifier")
    skin_col.addObject("Tetra2TriangleTopologicalMapping", input="@../topo", output="@colTopo", flipNormals=False)
    skin_col.addObject("MechanicalObject", name="colDofs", rest_position="@../dofs.rest_position")
    skin_col.addObject("TriangleCollisionModel", name="triCol", simulated=True, moving=True, contactDistance=0.02)
    skin_col.addObject("IdentityMapping", input="@../dofs", output="@colDofs")

    # =========================================================================
    # 2. MARQUEURS VISUELS DYNAMIQUES DES ORIFICES (ACTIFS + PERMANENTS)
    # =========================================================================
    pos_hole, tri_hole = make_puncture_mark_data(r=0.065, n_segs=12)
    for i in range(1, 5):
        h = root.addChild(f"Hole{i}")
        h.addObject("MechanicalObject", name="dofs", template="Rigid3d", position=[0.0, -10.0, 0.0, 0.0, 0.0, 0.0, 1.0])
        h_vis = h.addChild("Visual")
        h_vis.addObject("MeshTopology", name="topo", position=pos_hole, triangles=tri_hole)
        h_vis.addObject("OglModel", name="visual", color=[0.45, 0.04, 0.06, 1.0])
        h_vis.addObject("RigidMapping", input="@../dofs", output="@visual")

    pos_khole, tri_khole = make_puncture_mark_data(r=0.052, n_segs=10)
    for i in range(1, 2 * MAX_KNOTS + 1):
        kh = root.addChild(f"KnotHole{i}")
        kh.addObject("MechanicalObject", name="dofs", template="Rigid3d", position=[0.0, -10.0, 0.0, 0.0, 0.0, 0.0, 1.0])
        kh_vis = kh.addChild("Visual")
        kh_vis.addObject("MeshTopology", name="topo", position=pos_khole, triangles=tri_khole)
        kh_vis.addObject("OglModel", name="visual", color=[0.38, 0.05, 0.08, 1.0])
        kh_vis.addObject("RigidMapping", input="@../dofs", output="@visual")

    # =========================================================================
    # 3. NŒUDS CHIRURGICAUX PERMANENTS (JUSQU'À 6 NŒUDS VISIBLES)
    # =========================================================================
    for i in range(1, MAX_KNOTS + 1):
        k = root.addChild(f"Knot{i}")
        k.addObject("MeshTopology", name="topo", position=[[0, -10, 0], [0, -10, 0]], lines=[[0, 1]])
        k.addObject("OglModel", name="vis", src="@topo", lineWidth=4, color=[0.10, 0.22, 0.82, 1.0])

    # =========================================================================
    # 4. AIGUILLE CHIRURGICALE COURBE AVEC GESTION DE CONTACT FLUIDE
    # =========================================================================
    needle = root.addChild("Needle")

    q_init = quat_from_axis_angle([0.0, 0.0, 1.0], math.radians(180.0 + 35.0))
    needle.addObject(
        "MechanicalObject",
        name="dofs",
        template="Rigid3d",
        position=[0.15, 2.12, -1.20] + q_init,
    )

    script_dir = os.path.dirname(os.path.abspath(__file__))
    mesh_path = os.path.join(script_dir, "meshes", "curved_needle.obj")
    ensure_needle_mesh(mesh_path)

    # Modèle visuel de l'aiguille (rendu chromé chirurgical)
    needle_vis = needle.addChild("Visual")
    needle_vis.addObject("MeshOBJLoader", name="loader", filename=mesh_path)
    needle_vis.addObject("OglModel", name="visual", src="@loader", color=[0.92, 0.94, 0.98, 1.0])
    needle_vis.addObject("RigidMapping", input="@../dofs", output="@visual")

    # Modèle de collision de l'aiguille
    needle_col = needle.addChild("Collision")
    n_col_pts = 35
    col_angles = np.linspace(0, math.radians(-NEEDLE_ANGLE_DEG), n_col_pts)
    col_pts = [[float(NEEDLE_RADIUS * math.cos(a)), float(NEEDLE_RADIUS * math.sin(a)), 0.0] for a in col_angles]
    needle_col.addObject("MechanicalObject", name="colDofs", position=col_pts)
    needle_col.addObject("SphereCollisionModel", name="sphereCol", radius=0.04, simulated=False, moving=True, contactDistance=0.02)
    needle_col.addObject("RigidMapping", input="@../dofs", output="@colDofs")

    # =========================================================================
    # 5. FIL CHIRURGICAL SOUPLE AVEC VERROUILLAGE COHÉRENT DANS TOUS LES TROUS
    # =========================================================================
    thread = root.addChild("Thread")
    thread.addObject("EulerImplicitSolver", name="odeSolver", rayleighStiffness=0.01, rayleighMass=0.01)
    thread.addObject("CGLinearSolver", iterations=60, tolerance=1e-5, threshold=1e-15)

    n_thread_nodes = 22
    thread_length = 7.5

    base_local = [
        NEEDLE_RADIUS * math.cos(math.radians(-NEEDLE_ANGLE_DEG)),
        NEEDLE_RADIUS * math.sin(math.radians(-NEEDLE_ANGLE_DEG)),
        0.0,
    ]
    rot_base = rotate_vec_by_quat(base_local, q_init)
    eye_init = [0.15 + rot_base[0], 2.12 + rot_base[1], -1.20 + rot_base[2]]

    thread_positions = []
    for i in range(n_thread_nodes):
        t = i / float(n_thread_nodes - 1)
        px = eye_init[0] + 0.15 * t
        py = eye_init[1] - (thread_length * 0.90) * t
        pz = eye_init[2] + (thread_length * 0.35) * t
        thread_positions.append([px, py, pz])

    thread_edges = [[i, i + 1] for i in range(n_thread_nodes - 1)]

    thread.addObject("EdgeSetTopologyContainer", name="topology", position=thread_positions, edges=thread_edges)
    thread.addObject("EdgeSetTopologyModifier", name="modifier")
    thread.addObject("MechanicalObject", name="dofs", template="Vec3d", position=thread_positions)
    thread.addObject("UniformMass", totalMass=0.002)

    # Ancrage projectif dynamique (nœud 0 au départ, puis nœuds verrouillés aux trous créés)
    thread.addObject("FixedProjectiveConstraint", name="fixPoints", indices=[0])

    # Modèle physique de câble souple
    thread.addObject("MeshSpringForceField", name="springs", stiffness=800.0, damping=2.5)
    thread.addObject("UncoupledConstraintCorrection", defaultCompliance=0.001)

    # Modèle visuel du fil (ligne bleue chirurgicale d'épaisseur 4)
    thread_vis = thread.addChild("Visual")
    thread_vis.addObject("OglModel", name="visual", edges="@../topology.edges", lineWidth=4, color=[0.12, 0.25, 0.85, 1.0])
    thread_vis.addObject("IdentityMapping", input="@../dofs", output="@visual")

    # =========================================================================
    # 6. CONTRÔLEUR CHIRURGICAL DYNAMIQUE COMPLET (VERSION FINALE)
    # =========================================================================
    controller = SurgicalNeedleController(
        name="SurgicalNeedleController",
        root=root,
        idx_map=idx_map,
        xs_left=xs_left,
        xs_right=xs_right,
        zs=zs,
    )
    root.addObject(controller)

    return root
