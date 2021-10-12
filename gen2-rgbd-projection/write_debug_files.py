import cv2
import open3d
import os
from datetime import datetime
from pathlib import Path

def ensure_debug_path():
    path_to_current_file = Path(__file__).parent
    debug_path = path_to_current_file.joinpath(
        f"__debug_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S-%f')}__"
    )
    debug_path.mkdir(parents=True, exist_ok=True)
    return str(debug_path.absolute())

def write_debug_files(
    image_list = None,
    pcl_converter = None,
    path = None
):
    if path is None:
        path = ensure_debug_path()

    for name, image in image_list:
        cv2.imwrite(os.path.join(path, f"{name}.png"), image)

    if pcl_converter is not None:
        pcl_converter.vis.capture_screen_image(os.path.join(path, f"open3d_screen.png"))
        pcl_converter.vis.capture_depth_image(os.path.join(path, f"open3d_depth.png"))
        open3d.io.write_point_cloud(
            os.path.join(path, f"pcl.ply"),
            pcl_converter.pcl,
            print_progress=True
        )

debug_call = 0
def write_debug_files_on_nth_call(
    nth,
    image_list = None,
    pcl_converter = None,
    path = None
):
    global debug_call
    debug_call += 1
    if debug_call == nth:
        write_debug_files(image_list, pcl_converter, path)
