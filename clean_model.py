"""
Visualise an existing COLMAP sparse scan as a cleaner, more heavily
interpolated 3D model.

READ-ONLY:
- Reads slam_reconstruction/sparse_model.ply
- Does NOT run COLMAP
- Does NOT read the video
- Does NOT modify or save any project files
- All reconstruction/interpolation is performed in memory

Run:
    .\.venv\Scripts\python.exe clean_model.py

Approach:
- Conservatively removes isolated noise.
- Finds large roughly-planar regions and fits smooth flat surfaces through
  the points using local weighted averaging.
- Uses the remaining points with Ball Pivoting for curved/irregular geometry.
- The resulting mesh is only for display and is not saved.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import open3d as o3d


PROJECT_DIR = Path(__file__).resolve().parent
INPUT = PROJECT_DIR / "slam_reconstruction" / "sparse_model.ply"


def prepare_cloud(source: o3d.geometry.PointCloud) -> o3d.geometry.PointCloud:
    cloud = o3d.geometry.PointCloud(source)

    if len(cloud.points) >= 100:
        cloud, _ = cloud.remove_statistical_outlier(
            nb_neighbors=min(24, max(10, len(cloud.points) // 500)),
            std_ratio=2.8,
        )

    return cloud


def plane_basis(normal: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    normal = normal / np.linalg.norm(normal)

    # Pick a reference axis that is not parallel to the plane normal.
    ref = np.array([1.0, 0.0, 0.0])
    if abs(np.dot(ref, normal)) > 0.9:
        ref = np.array([0.0, 1.0, 0.0])

    u = np.cross(normal, ref)
    u /= np.linalg.norm(u)
    v = np.cross(normal, u)
    v /= np.linalg.norm(v)

    return u, v


def make_planar_patch(
    points: np.ndarray,
    colors: np.ndarray | None,
    indices: np.ndarray,
    normal: np.ndarray,
) -> o3d.geometry.TriangleMesh | None:
    """
    Fit a regular 2D grid to points belonging to one planar surface.

    Each grid vertex is an inverse-distance weighted average of nearby
    measured points. This deliberately interpolates instead of joining
    every measured point directly.
    """
    p = points[indices]
    if len(p) < 30:
        return None

    center = p.mean(axis=0)
    u, v = plane_basis(normal)

    xy = np.column_stack(((p - center) @ u, (p - center) @ v))

    x_span = float(xy[:, 0].max() - xy[:, 0].min())
    y_span = float(xy[:, 1].max() - xy[:, 1].min())

    # Reject tiny or essentially one-dimensional clusters.
    if min(x_span, y_span) < 1e-6 or min(x_span, y_span) / max(x_span, y_span) < 0.12:
        return None

    # Choose grid resolution from point count, not global cloud size.
    nx = int(np.clip(np.sqrt(len(p)) * 1.7, 7, 32))
    ny = int(np.clip(np.sqrt(len(p)) * 1.7, 7, 32))

    xmin, xmax = xy[:, 0].min(), xy[:, 0].max()
    ymin, ymax = xy[:, 1].min(), xy[:, 1].max()

    gx = np.linspace(xmin, xmax, nx)
    gy = np.linspace(ymin, ymax, ny)

    # Estimate local spacing for support radius.
    if len(p) > 1:
        d = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(p))
        spacing = d.compute_nearest_neighbor_distance()
        median_spacing = float(np.median(spacing)) if len(spacing) else 0.01
    else:
        median_spacing = 0.01

    cell = max(
        (xmax - xmin) / max(nx - 1, 1),
        (ymax - ymin) / max(ny - 1, 1),
        median_spacing,
    )
    support_radius = cell * 2.6

    vertices = []
    valid = np.zeros((ny, nx), dtype=bool)
    vertex_colors = []

    for iy, yy in enumerate(gy):
        for ix, xx in enumerate(gx):
            delta = xy - np.array([xx, yy])
            dist = np.linalg.norm(delta, axis=1)

            near = dist <= support_radius
            if np.count_nonzero(near) < 3:
                continue

            # Inverse-distance weighted average gives a smooth surface
            # rather than forcing triangles through every raw point.
            dn = np.maximum(dist[near], support_radius * 0.05)
            weights = 1.0 / (dn * dn)
            weights /= weights.sum()

            local = p[near]
            avg = np.sum(local * weights[:, None], axis=0)

            # Re-project the averaged point onto the fitted plane.
            avg = center + (avg - center).dot(u) * u + (avg - center).dot(v) * v
            vertices.append(avg)

            if colors is not None and len(colors) == len(points):
                c = colors[indices][near]
                vertex_colors.append(np.sum(c * weights[:, None], axis=0))

            valid[iy, ix] = True

    if len(vertices) < 9:
        return None

    vertex_index = -np.ones((ny, nx), dtype=int)
    k = 0
    for iy in range(ny):
        for ix in range(nx):
            if valid[iy, ix]:
                vertex_index[iy, ix] = k
                k += 1

    triangles = []
    for iy in range(ny - 1):
        for ix in range(nx - 1):
            a = vertex_index[iy, ix]
            b = vertex_index[iy, ix + 1]
            c = vertex_index[iy + 1, ix]
            d = vertex_index[iy + 1, ix + 1]

            if min(a, b, c, d) >= 0:
                triangles.append([a, b, d])
                triangles.append([a, d, c])

    if not triangles:
        return None

    mesh = o3d.geometry.TriangleMesh(
        vertices=o3d.utility.Vector3dVector(np.asarray(vertices)),
        triangles=o3d.utility.Vector3iVector(np.asarray(triangles, dtype=np.int32)),
    )

    if vertex_colors:
        mesh.vertex_colors = o3d.utility.Vector3dVector(
            np.asarray(vertex_colors)
        )

    mesh.compute_vertex_normals()
    return mesh


def find_planar_patches(
    cloud: o3d.geometry.PointCloud,
    max_patches: int = 6,
) -> tuple[list[o3d.geometry.TriangleMesh], np.ndarray]:
    """
    Repeatedly find strong planes using RANSAC.

    The largest useful planes are converted into interpolated surface
    patches. The remaining points are returned for curved-surface meshing.
    """
    working = np.asarray(cloud.points)
    original_colors = np.asarray(cloud.colors) if cloud.has_colors() else None

    remaining = np.arange(len(working))
    patches = []

    if len(remaining) < 60:
        return patches, remaining

    # Estimate local spacing and make plane fitting tolerant but not huge.
    tmp = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(working))
    distances = tmp.compute_nearest_neighbor_distance()
    spacing = float(np.median(distances)) if len(distances) else 0.01

    threshold = max(spacing * 2.5, 1e-5)

    for _ in range(max_patches):
        if len(remaining) < 50:
            break

        sub = o3d.geometry.PointCloud(
            o3d.utility.Vector3dVector(working[remaining])
        )

        try:
            plane_model, inliers_local = sub.segment_plane(
                distance_threshold=threshold,
                ransac_n=3,
                num_iterations=800,
            )
        except RuntimeError:
            break

        if len(inliers_local) < max(30, int(len(remaining) * 0.04)):
            break

        normal = np.asarray(plane_model[:3], dtype=float)
        normal /= np.linalg.norm(normal)

        inlier_global = remaining[np.asarray(inliers_local, dtype=int)]

        patch = make_planar_patch(
            working,
            original_colors,
            inlier_global,
            normal,
        )

        if patch is None:
            # Remove a small amount and keep searching so a bad plane does
            # not trap the algorithm forever.
            mask = np.ones(len(remaining), dtype=bool)
            mask[np.asarray(inliers_local, dtype=int)] = False
            remaining = remaining[mask]
            continue

        patches.append(patch)

        keep = np.ones(len(remaining), dtype=bool)
        keep[np.asarray(inliers_local, dtype=int)] = False
        remaining = remaining[keep]

    return patches, remaining


def make_curved_mesh(
    points_cloud: o3d.geometry.PointCloud,
) -> o3d.geometry.TriangleMesh | None:
    if len(points_cloud.points) < 20:
        return None

    work = o3d.geometry.PointCloud(points_cloud)
    distances = work.compute_nearest_neighbor_distance()
    if len(distances) == 0:
        return None

    median = float(np.median(distances))

    work.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(
            radius=max(median * 3.0, 1e-5),
            max_nn=30,
        )
    )

    radii = o3d.utility.DoubleVector([
        max(median * 2.0, 1e-5),
        max(median * 3.5, 1e-5),
        max(median * 5.5, 1e-5),
    ])

    try:
        mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
            work,
            radii,
        )
    except RuntimeError:
        return None

    if mesh.is_empty():
        return None

    mesh.remove_duplicated_vertices()
    mesh.remove_duplicated_triangles()
    mesh.remove_degenerate_triangles()
    mesh.remove_unreferenced_vertices()
    mesh.compute_vertex_normals()
    return mesh


def main() -> int:
    if not INPUT.exists():
        print(f"ERROR: Source point cloud not found:\n{INPUT}")
        print("Run video_to_3d.py first.")
        return 1

    print(f"Loading existing scan (read-only): {INPUT}")
    source = o3d.io.read_point_cloud(str(INPUT))

    if source.is_empty():
        print("ERROR: Source point cloud is empty.")
        return 1

    print(f"Source points: {len(source.points):,}")

    cloud = prepare_cloud(source)
    print(f"Points used for display reconstruction: {len(cloud.points):,}")

    planar_meshes, remaining_indices = find_planar_patches(cloud)
    print(f"Interpolated planar surfaces: {len(planar_meshes)}")
    print(f"Points remaining for curved geometry: {len(remaining_indices):,}")

    remaining_cloud = o3d.geometry.PointCloud(
        o3d.utility.Vector3dVector(np.asarray(cloud.points)[remaining_indices])
    )

    if cloud.has_colors():
        remaining_cloud.colors = o3d.utility.Vector3dVector(
            np.asarray(cloud.colors)[remaining_indices]
        )

    curved = make_curved_mesh(remaining_cloud)

    geometries = []

    # Show the interpolated surfaces first.
    geometries.extend(planar_meshes)

    if curved is not None:
        geometries.append(curved)

    # Keep the cleaned measured points visible so the user can distinguish
    # observed geometry from interpolated geometry.
    geometries.append(cloud)

    # Coordinate axes only: no grid.
    bbox = cloud.get_axis_aligned_bounding_box()
    extent = max(float(np.max(bbox.get_extent())), 1.0)
    geometries.append(
        o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=extent * 0.12,
            origin=np.asarray(cloud.get_center()),
        )
    )

    print("\nOpening interpolated 3D model...")
    print("No files will be changed or saved.")
    print("Controls: left-drag rotate, wheel zoom, Shift+drag pan, H for help.")

    vis = o3d.visualization.Visualizer()
    vis.create_window(
        window_name="SLAM Project - Interpolated 3D Model",
        width=1100,
        height=750,
    )

    for geometry in geometries:
        vis.add_geometry(geometry)

    render = vis.get_render_option()
    render.point_size = 2.2
    render.background_color = np.asarray([0.06, 0.06, 0.06])

    vis.run()
    vis.destroy_window()

    return 0


if __name__ == "__main__":
    sys.exit(main())
