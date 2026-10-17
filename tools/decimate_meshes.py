"""
Decimates OBJ meshes with Blender, keeping their coordinates, so that the models are light to load,
compile and render.

Usage (headless):
    blender -b --factory-startup --python tools/decimate_meshes.py -- \
        <source_dir> <output_dir> [--smooth <degrees>] <mesh.obj>=<triangles> [...] [--copy <mesh.obj> ...]

Each mesh listed with a number of triangles is decimated to about that many (Blender's collapse
decimation), and the meshes after --copy are copied unchanged. The meshes are written
as OBJ, with their normals and without materials or texture coordinates. The normals are smooth
between faces that meet at less than --smooth degrees (default: 30), and sharp otherwise.

The commands that generated the meshes of each model are in its README.md.
"""

import math
import os
import shutil
import sys

import bpy


def decimate(source, output, triangles, smooth_degrees):
    bpy.ops.wm.read_factory_settings(use_empty=True)
    # Identity axes on import and export: the coordinates are not converted
    bpy.ops.wm.obj_import(filepath=source, forward_axis='Y', up_axis='Z')
    objects = [o for o in bpy.context.scene.objects if o.type == 'MESH']
    if len(objects) != 1:
        raise RuntimeError(f"{source}: expected one mesh, found {len(objects)}")
    obj = objects[0]
    bpy.context.view_layer.objects.active = obj
    obj.select_set(True)

    before = sum(len(p.vertices) - 2 for p in obj.data.polygons)
    if triangles < before:
        modifier = obj.modifiers.new("decimate", 'DECIMATE')
        modifier.decimate_type = 'COLLAPSE'
        modifier.ratio = triangles / before
        modifier.use_collapse_triangulate = True
        bpy.ops.object.modifier_apply(modifier=modifier.name)
    bpy.ops.object.shade_smooth_by_angle(angle=math.radians(smooth_degrees))
    after = sum(len(p.vertices) - 2 for p in obj.data.polygons)

    bpy.ops.wm.obj_export(filepath=output, forward_axis='Y', up_axis='Z', export_selected_objects=True,
                          export_normals=True, export_uv=False, export_materials=False,
                          export_triangulated_mesh=True)
    print(f"{os.path.basename(source)}: {before} -> {after} triangles")


def main():
    args = sys.argv[sys.argv.index("--") + 1:]
    source_dir, output_dir = args[0], args[1]
    os.makedirs(output_dir, exist_ok=True)
    copying = False
    smooth_degrees = 30.0
    rest = args[2:]
    if len(rest) >= 2 and rest[0] == "--smooth":
        smooth_degrees = float(rest[1])
        rest = rest[2:]
    for arg in rest:
        if arg == "--copy":
            copying = True
        elif copying:
            shutil.copyfile(os.path.join(source_dir, arg), os.path.join(output_dir, arg))
            print(f"{arg}: copied")
        else:
            name, triangles = arg.split("=")
            decimate(os.path.join(source_dir, name), os.path.join(output_dir, name), int(triangles), smooth_degrees)


main()
