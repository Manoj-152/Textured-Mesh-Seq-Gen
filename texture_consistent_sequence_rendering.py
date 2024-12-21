import cv2
import numpy as np
import open3d as o3d
from random import random
import os
from glob import glob
from tqdm import tqdm
import argparse

parser = argparse.ArgumentParser(description='Applying texture to the whole sequence')
parser.add_argument('mesh_root_dir', type=str, help='Path to folder with mesh sequences')
parser.add_argument('initial_mesh_dir', type=str, help='Path to the initial meshes with texture applied')
parser.add_argument('save_dir', type=str, help='Save Directory')
parser.add_argument('--increase_res', action='store_true', help='Need to increase mesh resolution?')
args = parser.parse_args()

# Path Examples
# mesh_root_dir = '/scratch/ms14845/DFAUST Mesh/Coregistered_Meshes/'
# initial_mesh_dir = 'Initial_frames_with_texture'
# save_dir = 'sequences'

os.makedirs(args.save_dir, exist_ok=True)
sequence_paths = sorted(glob(os.path.join(args.mesh_root_dir, '*')))
sequence_names = [os.path.basename(path) for path in sequence_paths]

for seq_num, seq_name in enumerate(sequence_names):
    mesh_list = sorted(glob(os.path.join(args.mesh_root_dir, seq_name, '*.obj')))
    print(f'Sequence {seq_num+1}/{len(sequence_names)}')
    os.makedirs(os.path.join(args.save_dir, seq_name), exist_ok=True)
    print_flag = True

    for mesh_path in tqdm(mesh_list):
        reference_mesh_path = os.path.join(args.initial_mesh_dir, seq_name, '00000.obj')
        mesh: o3d.geometry.TriangleMesh = o3d.io.read_triangle_mesh(mesh_path, enable_post_processing=True)
        reference_mesh: o3d.geometry.TriangleMesh = o3d.io.read_triangle_mesh(reference_mesh_path, enable_post_processing=True)

        if args.increase_res:
            mesh = mesh.subdivide_loop(number_of_iterations=2)    
            assert len(mesh.triangles) == len(reference_mesh.triangles)

        if len(mesh.triangles) != len(reference_mesh.triangles) and print_flag:
            print(f"Debug Printing: {seq_name}")
            print_flag = False
            
        # print(len(np.asarray(reference_mesh.triangles)))
        # print(len(np.asarray(mesh.triangles)))

        texture_path = os.path.join(os.path.dirname(reference_mesh_path), '00000_0.png')
        texture_o3d = o3d.io.read_image(texture_path)
        texture_np = np.asarray(texture_o3d)
        mesh.triangle_uvs = reference_mesh.triangle_uvs
        mesh.textures = [texture_o3d]

        current_mesh_folder = os.path.join(args.save_dir, seq_name, os.path.basename(mesh_path)[:-4])
        os.makedirs(current_mesh_folder, exist_ok=True)
        mesh_save_path = os.path.join(current_mesh_folder, '00000.obj')
        o3d.io.write_triangle_mesh(mesh_save_path, mesh)

        cv2.imwrite(os.path.join(current_mesh_folder, '00000_0.png'), cv2.cvtColor(texture_np, cv2.COLOR_RGB2BGR))