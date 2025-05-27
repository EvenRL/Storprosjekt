import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from pathlib import Path
import math

# Configure project paths
root = Path("results")
if not root.exists():
    raise FileNotFoundError("Directory 'results/' not found. Run evaluator first.")

param_dirs = sorted([d for d in root.iterdir() if d.is_dir()])
variants = [d.name for d in param_dirs]
images = {d.name: sorted(list(d.glob("*.png"))) for d in param_dirs}

# Determine grid size (square-ish)
n = len(variants)
n_cols = math.ceil(math.sqrt(n))
n_rows = math.ceil(n / n_cols)

# State
frame_idx = 0
num_frames = min(len(imgs) for imgs in images.values())

# Create subplots grid
fig, axes = plt.subplots(n_rows, n_cols, figsize=(4 * n_cols, 4 * n_rows))
axes = axes.flatten()  # make indexing easier
plt.suptitle("← / → to change frame index", fontsize=16)

def show_frame(idx):
    for ax in axes:
        ax.clear()
        ax.axis('off')

    for ax, var in zip(axes, variants):
        frame_list = images[var]
        img_path = frame_list[idx % len(frame_list)]
        img = mpimg.imread(str(img_path))
        ax.imshow(img)
        ax.set_title(f"{var}\n{img_path.name}")

    fig.canvas.draw()

def on_key(event):
    global frame_idx
    if event.key == 'right':
        frame_idx = (frame_idx + 1) % num_frames
        show_frame(frame_idx)
    elif event.key == 'left':
        frame_idx = (frame_idx - 1) % num_frames
        show_frame(frame_idx)

fig.canvas.mpl_connect('key_press_event', on_key)
show_frame(frame_idx)
plt.tight_layout(rect=[0, 0.03, 1, 0.95])  # leave space for title
plt.show()
