"""Prepare the complete real-car contact surface in vehicle mm; keep the original CAD assembly.

Run with the cobot-native Python: native/contact_mesh.py. Native-only geometry
dependencies are used during preparation; RealVehicle still needs only numpy.
"""
import hashlib
import json
from pathlib import Path
import sys

import fast_simplification
import manifold3d
import numpy as np
import trimesh

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / 'ws_cobot_pjt/src/repair_car'))
from repair_car.config import PACKAGE
from repair_car.geometry.real_vehicle import RealVehicle


class ContactMesh:
    tolerance = .01  # mm for each of planar projection and surface simplification

    @staticmethod
    def mesh(triangles):
        return trimesh.Trimesh(triangles.reshape(-1, 3),
                              np.arange(len(triangles) * 3).reshape(-1, 3), process=False)

    @staticmethod
    def audit(mesh):
        mesh = mesh.copy()
        mesh.merge_vertices(digits_vertex=8)
        edges = np.bincount(mesh.edges_unique_inverse)
        angles = np.rad2deg(mesh.face_adjacency_angles)
        return dict(triangles=len(mesh.faces), vertices=len(mesh.vertices),
                    degenerate=int((~mesh.nondegenerate_faces(height=1e-6)).sum()),
                    duplicates=int((~mesh.unique_faces()).sum()),
                    boundary_edges=int((edges == 1).sum()), nonmanifold_edges=int((edges > 2).sum()),
                    winding_consistent=bool(mesh.is_winding_consistent),
                    near_coplanar_edges=int((angles < .1).sum()),
                    folded_edges=int((angles > 170).sum()), area_mm2=float(mesh.area))

    @staticmethod
    def clean(mesh):
        mesh.merge_vertices(digits_vertex=8)
        _, inverse, count = np.unique(np.sort(mesh.faces, axis=1), axis=0,
                                       return_inverse=True, return_counts=True)
        # CAD contains back-to-back copies of internal faces: remove both sides.
        keep = mesh.nondegenerate_faces(height=1e-10) & (count[inverse] == 1)
        mesh.update_faces(keep)
        mesh.remove_unreferenced_vertices()
        return mesh

    def flatten(self, mesh):
        """Project near-planar patches, leaving vertices shared with other surfaces fixed."""
        adjacent = mesh.face_adjacency
        normal = mesh.face_normals
        # A normal bin prevents a chain of small angles flattening an entire curved panel.
        bins = np.round(normal / np.deg2rad(.5)).astype(int)
        same = (bins[adjacent[:, 0]] == bins[adjacent[:, 1]]).all(1)
        groups = trimesh.graph.connected_components(adjacent[same], min_len=3,
                                                     nodes=np.arange(len(mesh.faces)))
        labels = np.full(len(mesh.faces), -1, dtype=int)
        planes = []
        for group in groups:
            ids = np.unique(mesh.faces[group])
            points = mesh.vertices[ids]
            centre = points.mean(0)
            _, _, axes = np.linalg.svd(points - centre, full_matrices=False)
            n = axes[-1]
            if np.max(np.abs((points - centre) @ n)) > self.tolerance:
                continue
            labels[group] = len(planes)
            planes.append((centre, n))
        low = np.full(len(mesh.vertices), len(planes), dtype=int)
        high = np.full(len(mesh.vertices), -2, dtype=int)
        np.minimum.at(low, mesh.faces.ravel(), np.repeat(labels, 3))
        np.maximum.at(high, mesh.faces.ravel(), np.repeat(labels, 3))
        interior = np.flatnonzero((low >= 0) & (low == high))
        interior = interior[np.argsort(low[interior], kind='stable')]
        stops = np.searchsorted(low[interior], np.arange(len(planes) + 1))
        moved = 0
        for index, (centre, normal) in enumerate(planes):
            ids = interior[stops[index]:stops[index + 1]]
            distance = (mesh.vertices[ids] - centre) @ normal
            mesh.vertices[ids] -= distance[:, None] * normal
            moved += int((np.abs(distance) > 1e-7).sum())
        return dict(planar_patches=len(planes), projected_vertices=moved)

    def prepare(self, triangles):
        source = self.mesh(triangles)
        before = self.audit(source)
        mesh = self.clean(source.copy())
        flattening = self.flatten(mesh)
        components = mesh.split(only_watertight=False, repair=False)
        output, simplified, open_sheets = [], 0, 0
        for part in components:
            solid = manifold3d.Manifold(manifold3d.Mesh64(np.asarray(part.vertices, np.float64),
                                                         np.asarray(part.faces, np.uint64)))
            if solid.status() == manifold3d.Error.NoError and not solid.is_empty():
                result = solid.simplify(self.tolerance).to_mesh64()
                part = trimesh.Trimesh(np.asarray(result.vert_properties)[:, :3],
                                       np.asarray(result.tri_verts), process=False)
                simplified += 1
            else:
                # Preserve open CAD sheets and their borders; no hole filling or convex hull.
                vertices, faces = fast_simplification.simplify(
                    np.asarray(part.vertices), np.asarray(part.faces), lossless=True, preserve_border=True)
                part = trimesh.Trimesh(vertices, faces, process=False)
                open_sheets += 1
            output.append(part)
        mesh = self.clean(trimesh.util.concatenate(output))
        return mesh, dict(before=before, after=self.audit(mesh), **flattening,
                          simplified_components=simplified, open_components=open_sheets,
                          projection_tolerance_mm=self.tolerance,
                          simplification_tolerance_mm=self.tolerance)


def main():
    car = RealVehicle(prepared=False, clip_parts=False)
    prepared, report = ContactMesh().prepare(car.triangles)
    path = PACKAGE / 'meshes/vehicles/b3_project1/assembly_surface.npz'
    np.savez_compressed(path, triangles=prepared.triangles)
    report.update(source_mesh=car.spec['mesh'], surface_mesh=str(path.relative_to(PACKAGE)),
                  source_sha256=hashlib.sha256((PACKAGE / car.spec['mesh']).read_bytes()).hexdigest(),
                  surface_sha256=hashlib.sha256(path.read_bytes()).hexdigest(), units='vehicle_mm')
    path.with_suffix('.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
