# Constructor fantasy models

13 low-poly, neutral FBX assets matching ConstructorFigure (0.48 m body height).
The trebuchet has four hierarchical meshes with animation pivots; each other
FBX has one mesh. All have explicit flat normals and an identity root transform.
Molded bevels, studs and geometric silhouettes match construction-toy pieces.
Y-up, +Z forward, dimensions in metres after Unity file-unit conversion.
No textures, UVs, vertex colors, embedded materials, rig, animation or colliders.
Assign a Unity material to the renderer; the spell assets can share a tintable
material. Effects are static meshes intended to be spawned, scaled and faded
by your gameplay/VFX code. Overlapping closed shells are intentional; these
are visual models, not boolean-unioned manufacturing solids.

| Model | Triangles | X × Y × Z (m) |
| --- | ---: | --- |
| ConstructorTrebuchet | 2160 | 0.726 × 1.080 × 1.325 |
| ConstructorRock01 | 20 | 0.097 × 0.100 × 0.095 |
| ConstructorRock02 | 20 | 0.115 × 0.078 × 0.083 |
| ConstructorRock03 | 20 | 0.090 × 0.114 × 0.080 |
| ConstructorRock04 | 80 | 0.114 × 0.107 × 0.113 |
| ConstructorBow | 676 | 0.017 × 0.349 × 0.073 |
| ConstructorBowArrow | 98 | 0.034 × 0.035 × 0.280 |
| ConstructorBowHeavyArrow | 98 | 0.044 × 0.044 × 0.280 |
| ConstructorMage | 1502 | 0.354 × 0.656 × 0.188 |
| ConstructorSpellProjectile | 60 | 0.100 × 0.099 × 0.295 |
| ConstructorSpellExplosion01 | 116 | 0.380 × 0.173 × 0.391 |
| ConstructorSpellExplosion02 | 212 | 0.378 × 0.159 × 0.378 |
| ConstructorSpellExplosion03 | 320 | 0.311 × 0.228 × 0.321 |

**ConstructorTrebuchet** — Four pivoted meshes: Base → ActiveBeam → {HangingLoad, ProjectileBasin}. Rotate each moving part about local X. Base pivot (0,0,0); beam axle (0,0.65,0); load hinge (0,0.47,-0.28); basin suspension (0,1.05,0.65), all in assembled metres. Load includes hanger straps; basin includes suspension ropes. Local positions relative to beam: load (0,-0.18,-0.28), basin (0,0.40,0.65). Zero rotations reproduce the rest pose. Counter-rotate the hanging parts to maintain their world orientation as the beam moves, or animate them independently to swing. Throws toward +Z; accepts rocks around 0.10 m. No baked animation or physics.

**ConstructorRock01** — Centred projectile pivot. Faceted molded boulder; sized for the trebuchet sling.

**ConstructorRock02** — Centred projectile pivot. Faceted molded boulder; sized for the trebuchet sling.

**ConstructorRock03** — Centred projectile pivot. Faceted molded boulder; sized for the trebuchet sling.

**ConstructorRock04** — Centred projectile pivot. Faceted molded boulder; sized for the trebuchet sling.

**ConstructorBow** — Grip-centre pivot, Y-up, shoots +Z. 0.34 m tall; 0.017 m grip fits the original figure clip opening. Suggested hand placement (±0.148, 0.197, 0.008) relative to figure.

**ConstructorBowArrow** — Centre pivot, points +Z. 0.28 m long, three solid molded fins; fits ConstructorBow. Static mesh.

**ConstructorBowHeavyArrow** — Centre pivot, points +Z. 0.28 m long, three solid molded fins; fits ConstructorBow. Static mesh.

**ConstructorMage** — Original 0.48 m figure body plus robe, beard, pointed hat and staff. Faces +Z; foot-centre pivot. Neutral single mesh, no rig; approximately 0.66 m tall with hat.

**ConstructorSpellProjectile** — Centre pivot; travels +Z with a tapered trail toward -Z. Assign any opaque, transparent or emissive material in Unity.

**ConstructorSpellExplosion01** — Radial starburst with chunky rays; impact-centre pivot. Neutral material-ready static effect.

**ConstructorSpellExplosion02** — Horizontal shock ring with crystal fragments; impact-centre pivot, Y-up. Neutral material-ready static effect.

**ConstructorSpellExplosion03** — Faceted puff cloud with flying fragments; impact-centre pivot. Neutral material-ready static effect.

Regenerate using Blender 5.2 in background mode with
`Tools/generate_constructor_fantasy.py`. The generator validates closed edges,
consistent winding, nondegenerate faces and a 4,000 triangle ceiling per asset.
Previews show the actual exported geometry
with neutral studio shading; individual preview scales vary for readability.
