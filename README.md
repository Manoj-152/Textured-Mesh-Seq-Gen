# Textured Mesh Sequence Generation

```initial_frame_rendering.py```: Script responsible for rendering the initial frames with texture applied to it.

Command line:
```
python3 initial_frame_rendering.py <MESH ROOT DIR> <TEXTURE ROOT DIR> <SAVE DIR> 
```
where the `<MESH_ROOT_DIR` is the directory where the original DFAUST mesh sequences are stored, `<TEXTURE ROOT DIR>` is the directory where the textures to be pasted are present, and `<SAVE_DIR>` is where we want the initial mesh frames from each sequence with texture applied need to be stored.

Example:
```
python3 initial_frame_rendering.py Coregistered_Meshes Augmented_Textures_DIV2k Initial_frames_with_texture
```
----

```texture_consistent_sequence_rendering.py```: Script responsible for rendering the subsequent frames in a sequence with respect to its first frame.

Command line:
```
python3 texture_consistent_sequence_rendering.py <MESH ROOT DIR> <INITIAL MESH DIR> <SAVE DIR> 
```
where the `<MESH_ROOT_DIR` is the directory where the original DFAUST mesh sequences are stored, `<INITIAL MESH DIR>` is the directory where the initial mesh frames from each sequence with texture applied are stored, and `<SAVE_DIR>` is where the mesh sequences with textures applied are to be stored.

Example:
```
python3 texture_consistent_sequence_rendering.py Coregistered_Meshes Initial_frames_with_texture sequences
```
----

```augment_texture_images.py```: Script to gaussian blur the original texture images, if needed

Command line:
```
python3 augment_texture_images.py <INPUT IMG DIR> <SAVE DIR> --kernel_size 13 
```
where the `<INPUT IMG DIR>` is the directory where the original images selected from DIV 2K are stored, and `<SAVE_DIR>` is where we want to store the gaussian-blurred images.

Example:
```
python3 augment_texture_images.py Selected_Textures_DIV2K Augmented_Textures_DIV2k --kernel_size 13 
```
