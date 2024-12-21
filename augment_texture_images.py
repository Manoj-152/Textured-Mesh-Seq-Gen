import cv2
import open3d as o3d
import os
import numpy as np

def augment_image(img_path, save_folder, gaussian_kernel_size=13):
    texture_o3d = o3d.io.read_image(img_path)
    texture_np = np.asarray(texture_o3d)
    texture_np = cv2.cvtColor(texture_np, cv2.COLOR_RGB2BGR)
    
    blurred_texture_np = cv2.GaussianBlur(texture_np, (13,13), 0)
    cv2.imwrite(os.path.join(save_folder, os.path.basename(img_path)), blurred_texture_np)


if __name__ == "__main__":
    import argparse
    from glob import glob
    from tqdm import tqdm

    parser = argparse.ArgumentParser(description='Gaussian Blurring for Texture Images')
    parser.add_argument('input_img_folder', type=str, help='Path to the input image folder')
    parser.add_argument('save_folder', type=str, help='Path to save folder')
    parser.add_argument('--kernel_size', type=int, default=13, help='Gaussian kernel size')
    args = parser.parse_args()

    os.makedirs(args.save_folder, exist_ok=True)
    img_paths = glob(os.path.join(args.input_img_folder, '*'))
    print("Augmenting the Texture Images...")
    print(f"Gaussian Kernel size: {args.kernel_size}x{args.kernel_size}")

    for img_path in tqdm(img_paths):
        augment_image(img_path, args.save_folder, args.kernel_size)