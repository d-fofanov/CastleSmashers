"""Brick-count model of Assets/Phys/AvbdGpu/Scenes/BrickCastle.cs for tuning CastlePlan presets without running Unity.

Mirrors the generator's tiling course by course (English-bond walls, rings, gatehouse corbels, turrets, merlons, stairs)
and reproduces its counts exactly; the aspect column is height / width of each free-standing element (dry-stacked
structures lean past about 4). Run: python Tools/castle_counts.py
"""


def merlons(length, gap=2, size=2):
    n, a = 0, 0
    while a + size <= length:
        n += 1
        a += size + gap
    return n

def ring_per_course(n):
    return n + max(n - 6, 0)

def ring_merlons(n):
    return 2 * merlons(n) + 2 * merlons(max(n - 8, 0))

def gatehouse(G):
    return 7 * 12 + 14 + 16 + 18 + max(G - 10, 0) * 18

def count(S, T, layers, W, TC, G, gateLayers, U, K, C, M=0, midC=0, stairs=10):
    L = S - 2 * T
    total = 4 * (ring_per_course(T) * TC + ring_merlons(T))
    seg = [(L - M) // 2, (L - M) // 2] if M else [L]
    for _ in range(3):
        for l in seg:
            total += (l // 3 + l // 2) * W * layers + merlons(l)
        if M:
            total += ring_per_course(M) * (W + midC) + ring_merlons(M)
    fl = (L - 18) // 2
    total += 2 * ((fl // 3 + fl // 2) * W * layers + merlons(fl))
    total += gatehouse(G) * gateLayers + 2 * (4 * U + 2) + merlons(6)
    total += ring_per_course(K) * C + ring_merlons(K)
    st = min(W, stairs)
    total += st * (st + 1) // 2
    return total, L

def aspect(courses, studs):
    return courses * 0.605 / (studs * 0.504)

# (name, Side, TowerSize, WallLayers, WallCourses, TowerCourses, GateCourses, GateLayers, TurretCourses, KeepSize, KeepCourses, MidTowerSize, MidTowerCourses)
presets = [
    ("Outpost",         38, 10, 1,  5,  8, 10, 1, 3,  8, 10,  0,  0),
    ("Fort",            38, 10, 1,  6, 11, 11, 1, 4, 10, 14,  0,  0),
    ("Small castle",    50, 10, 1,  8, 15, 12, 1, 5, 14, 20,  0,  0),
    ("Castle",          62, 10, 1, 10, 19, 14, 1, 6, 16, 26,  0,  0),
    ("Large castle",    74, 10, 1, 12, 24, 17, 1, 7, 18, 30,  0,  0),
    ("Fortress",        94, 14, 1, 14, 28, 17, 1, 7, 22, 36,  6,  3),
    ("Citadel",        106, 14, 2, 12, 30, 18, 1, 8, 24, 40,  6,  2),
    ("Stronghold",     126, 18, 2, 16, 36, 22, 2, 8, 28, 46, 18,  4),
    ("Great fortress", 150, 18, 2, 20, 42, 26, 2, 8, 34, 54, 18,  6),
    ("Royal citadel",  170, 22, 2, 24, 50, 30, 2, 8, 42, 66, 18, 10),
]
prev = None
for name, S, T, layers, W, TC, G, gl, U, K, C, M, midC in presets:
    c, L = count(S, T, layers, W, TC, G, gl, U, K, C, M, midC)
    ok = (L - 18) % 12 == 0 and ((L - M) % 12 == 0 if M else True)
    asp = f"wall {aspect(W, 5*layers):.1f} tower {aspect(TC, T):.1f} gate {aspect(G + U, 6*gl):.1f} keep {aspect(C, K):.1f} mid {aspect(W + midC, M) if M else 0:.1f}"
    print(f"{name:15s} S={S:3d} T={T:2d} L={L:3d} ok={ok} -> {c:6d} x{(c/prev if prev else 0):.2f}  {asp}")
    prev = c
