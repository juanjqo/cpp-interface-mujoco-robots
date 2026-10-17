"""
Converts COLLADA (DAE) meshes to OBJ meshes for MuJoCo, which cannot load DAE: one OBJ per material
of each mesh (MuJoCo gives one material to each geom), named <mesh>_<material>.obj, with the
transformations of the DAE scene applied and the duplicated vertices merged (so that
decimate_meshes.py can decimate them). Coordinates are kept as they are (the DAE must be in meters,
with the z axis up). Triangles without a material are dropped.

Usage (needs pycollada and numpy, e.g., in a virtual environment: pip install pycollada numpy):
    python3 tools/dae_to_obj.py <output_dir> <mesh.dae> [...]

It prints the diffuse color of each material, for the materials of the model.
"""

import os
import sys

import collada
import numpy as np


def material_name(material_id):
    # "URBlue-material" -> "urblue"
    return material_id.replace("-material", "").replace("_", "").lower()


def convert(dae_file, output_dir):
    mesh = os.path.splitext(os.path.basename(dae_file))[0]
    dae = collada.Collada(dae_file)
    if dae.assetInfo.upaxis != "Z_UP" or dae.assetInfo.unitmeter not in (None, 1.0):
        raise RuntimeError(f"{dae_file}: expected meters and the z axis up")

    triangles = {}  # material -> list of (n, 3, 3) arrays
    colors = {}
    for geometry in dae.scene.objects("geometry"):  # with the transformations of the scene applied
        for primitive in geometry.primitives():
            if primitive.material is None:
                continue
            name = material_name(primitive.material.id)
            colors[name] = primitive.material.effect.diffuse
            if isinstance(primitive, collada.polylist.BoundPolylist):
                primitive = primitive.triangleset()
            triangles.setdefault(name, []).append(primitive.vertex[primitive.vertex_index])

    for name, parts in sorted(triangles.items()):
        corners = np.concatenate(parts).reshape(-1, 3)
        # Merge the vertices at the same position (DAE duplicates them across normals)
        vertices, faces = np.unique(np.round(corners, 7), axis=0, return_inverse=True)
        faces = faces.reshape(-1, 3)
        faces = faces[(faces[:, 0] != faces[:, 1]) & (faces[:, 1] != faces[:, 2]) & (faces[:, 0] != faces[:, 2])]
        output = os.path.join(output_dir, f"{mesh}_{name}.obj")
        with open(output, "w") as f:
            f.write(f"# {os.path.basename(dae_file)}, material {name}\n")
            f.writelines(f"v {x:.7g} {y:.7g} {z:.7g}\n" for x, y, z in vertices)
            f.writelines(f"f {a + 1} {b + 1} {c + 1}\n" for a, b, c in faces)
        rgba = " ".join(f"{c:.3g}" for c in colors[name])
        print(f"{os.path.basename(output)}: {len(faces)} triangles, diffuse {rgba}")


def main():
    output_dir, dae_files = sys.argv[1], sys.argv[2:]
    os.makedirs(output_dir, exist_ok=True)
    for dae_file in dae_files:
        convert(dae_file, output_dir)


if __name__ == "__main__":
    main()
