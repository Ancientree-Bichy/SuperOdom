#!/usr/bin/env python3
import argparse
from pathlib import Path

import numpy as np
import open3d as o3d


def parse_args() -> argparse.Namespace:
	parser = argparse.ArgumentParser(
		description="Read a point cloud file and visualize it with Open3D."
	)
	parser.add_argument(
		"pcd_path",
		type=Path,
		help="Path to point cloud file (.pcd/.ply/.xyz/.xyzn/.xyzrgb, etc.)",
	)
	return parser.parse_args()


def main() -> None:
	args = parse_args()
	pcd_path = args.pcd_path.expanduser().resolve()

	if not pcd_path.exists():
		raise FileNotFoundError(f"Point cloud file not found: {pcd_path}")

	pcd = o3d.io.read_point_cloud(str(pcd_path))
	if pcd.is_empty():
		raise ValueError(f"Loaded point cloud is empty: {pcd_path}")

	orig_count = len(pcd.points)
	points = np.asarray(pcd.points)
	dist = np.linalg.norm(points, axis=1)
	# Remove points that are both far away and too high.
	remove_mask = (dist > 9.0) | (points[:, 2] > 1.5)
	keep_indices = np.where(~remove_mask)[0]
	removed_count = int(remove_mask.sum())
	pcd = pcd.select_by_index(keep_indices)
	if pcd.is_empty():
		raise ValueError("Point cloud became empty after pass filter.")

	print(f"Loaded point cloud: {pcd_path}")
	print(f"Number of points before filter: {orig_count}")
	print(f"Number of removed points: {removed_count}")
	print(f"Number of points after filter: {len(pcd.points)}")

	# Scale origin marker from map size so it stays visible on small and large clouds.
	bbox = pcd.get_axis_aligned_bounding_box()
	extent = bbox.get_extent()
	diag = float((extent[0] ** 2 + extent[1] ** 2 + extent[2] ** 2) ** 0.5)
	axis_size = max(diag * 0.05, 0.5)
	center_radius = axis_size * 0.08

	origin_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
		size=axis_size,
		origin=[0.0, 0.0, 0.0],
	)
	origin_center = o3d.geometry.TriangleMesh.create_sphere(radius=center_radius)
	origin_center.paint_uniform_color([1.0, 0.0, 0.0])
	origin_center.translate([0.0, 0.0, 0.0])

	o3d.visualization.draw_geometries(
		[pcd, origin_frame, origin_center],
		window_name=f"Open3D Viewer - {pcd_path.name}",
		width=1280,
		height=720,
	)


if __name__ == "__main__":
	main()
