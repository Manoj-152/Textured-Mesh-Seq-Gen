import cv2
import numpy as np
import open3d as o3d
import random
import os
from tqdm import tqdm
from glob import glob


def rot_and_apply_uv(mesh, angle = None, center_w = None, ref_uv_max_vals = None, ref_uv_min_vals = None):
    #warning: for random texture

    if angle is not None:
        rot_mat = mesh.get_rotation_matrix_from_xyz(angle)
        mesh.rotate(rot_mat, center = center_w)
    
    # uv_map = [[(mesh.vertices[pid][:2]+1.0)/2 for pid in pids] for pids in mesh.triangles]
    uv_map = [[((mesh.vertices[pid][0]+mesh.vertices[pid][2])/2, (mesh.vertices[pid][1]+mesh.vertices[pid][2])/2) \
                                        for pid in pids] for pids in mesh.triangles]
    uv_map = np.array(uv_map).reshape((len(mesh.triangles)*3,2))
    # print(uv_map.min(axis=0))
    # exit()

    if ref_uv_max_vals is None:
        ref_uv_max_vals = np.array([1.,1.])

    if ref_uv_min_vals is None:
        ref_uv_min_vals = np.array([0.,0.])
            
    u_values = uv_map[:, 0]
    v_values = uv_map[:, 1]
    min_u_old = np.min(u_values)
    max_u_old = np.max(u_values)
    min_v_old = np.min(v_values)
    max_v_old = np.max(v_values)

    min_u_new, min_v_new = ref_uv_min_vals
    max_u_new, max_v_new = ref_uv_max_vals

    scaled_u_values = min_u_new + ((u_values - min_u_old) / (max_u_old - min_u_old)) * (max_u_new - min_u_new)
    scaled_v_values = min_v_new + ((v_values - min_v_old) / (max_v_old - min_v_old)) * (max_v_new - min_v_new)

    uv_map[:, 0] = scaled_u_values
    uv_map[:, 1] = scaled_v_values

    mesh.triangle_uvs = o3d.utility.Vector2dVector(uv_map)
    return mesh


def read_textures(root_dir, num_textures=130):
    img_paths = sorted(glob(os.path.join(root_dir, '*.png')))
    random.shuffle(img_paths)
    img_paths = img_paths[:num_textures]
    return img_paths

def read_initial_mesh_paths(root_dir):
    meshes = sorted(glob(os.path.join(root_dir, '*', '00000.obj')))
    return meshes


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='Applying texture to initial frames')
    parser.add_argument('mesh_root_dir', type=str, help='Path to folder with mesh sequences')
    parser.add_argument('texture_root_dir', type=str, help='Path to the texture image folder')
    parser.add_argument('save_dir', type=str, help='Save Directory')
    parser.add_argument('--increase_res', action='store_true', help='Need to increase mesh resolution?')
    args = parser.parse_args()

    # Path Examples
    # texture_root_dir = 'Augmented_Images_DIV2k'
    # mesh_root_dir = '/scratch/ms14845/DFAUST Mesh/Coregistered_Meshes/'
    # save_dir = 'Initial_frames_with_texture'
    os.makedirs(args.save_dir, exist_ok=True)

    texture_img_paths = read_textures(args.texture_root_dir, num_textures=129)
    mesh_paths = read_initial_mesh_paths(args.mesh_root_dir)
    assert len(texture_img_paths) == len(mesh_paths)

    for texture_path, mesh_path in zip(tqdm(texture_img_paths), mesh_paths):
        texture_o3d = o3d.io.read_image(texture_path)
        texture_np = np.asarray(texture_o3d)
        mesh: o3d.geometry.TriangleMesh = o3d.io.read_triangle_mesh(mesh_path, enable_post_processing=True)

        if args.increase_res:
            # mesh.compute_vertex_normals()
            print(f"Previous: {len(mesh.triangles)}")
            mesh = mesh.subdivide_midpoint(number_of_iterations=2)
            mesh = mesh.subdivide_loop(number_of_iterations=1)
            # mesh = mesh.subdivide_midpoint(number_of_iterations=2)
            mesh.compute_vertex_normals()
            print(f"Now: {len(mesh.triangles)}")

        scale = np.max(mesh.get_axis_aligned_bounding_box().get_half_extent())
        center_w = mesh.get_axis_aligned_bounding_box().get_center()

        # If meshes need to be rotated
        # rand_x = random.random()*np.pi*2
        # rand_y = random.random()*np.pi*2
        # rand_z = random.random()*np.pi*2
        # angle = (rand_x,rand_y,rand_z)
        mesh = rot_and_apply_uv(mesh)

        mesh.textures = [texture_o3d]
        path_components = os.path.normpath(mesh_path)
        path_components = path_components.split(os.sep)
        texture_folder = path_components[-2]
        os.makedirs(os.path.join(args.save_dir, texture_folder), exist_ok=True)
        output_mesh_path = os.path.join(args.save_dir, texture_folder, path_components[-1])
        o3d.io.write_triangle_mesh(output_mesh_path, mesh)

        cv2.imwrite(output_mesh_path[:-4]+'_0.png', cv2.cvtColor(texture_np, cv2.COLOR_RGB2BGR))
