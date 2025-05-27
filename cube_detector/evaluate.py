#!/usr/bin/env python3
import cv2 as cv, yaml, glob, csv, time, pathlib, numpy as np
from cube_detector.detector_core import findCubePoses

def load_calib(yaml_file):
    data = yaml.safe_load(open(yaml_file))
    K = np.array(data["camera_matrix"]["data"]).reshape(3,3)
    D = np.array(data["distortion_coefficients"]["data"])
    return K, D

def as_np_colors(raw):
    return {n: [np.asarray(lo, np.uint8), np.asarray(hi, np.uint8)]
            for n,(lo,hi) in raw.items()}

def main():
    root = pathlib.Path(__file__).resolve().parent
    imgs   = sorted((root/"test_images").glob("*.png"))
    param_files = sorted((root/"config"/"detect_params").glob("*.yaml"))
    K, D  = load_calib(root/"config"/"camera_info_usb2.yaml")

    (root/"results").mkdir(exist_ok=True)
    csv_path = root/"results"/"metrics.csv"
    with open(csv_path, "w", newline="") as fcsv:
        wr = csv.writer(fcsv)
        wr.writerow(["param_set","image","n_cubes","mean_px_err","runtime_ms"])

        for pf in param_files:
            params = yaml.safe_load(open(pf))
            colors = as_np_colors(params.pop("hsv"))
            tag    = pf.stem
            (root/"results"/tag).mkdir(exist_ok=True)

            for img_path in imgs:
                img = cv.imread(str(img_path))
                t0  = time.perf_counter()
                cubes, overlay = findCubePoses(img, colors, K, D, params)
                ms = 1000*(time.perf_counter()-t0)

                err = (sum(c["error"] for c in cubes)/len(cubes)) if cubes else 0
                cv.imwrite(str(root/"results"/tag/img_path.name), overlay)

                wr.writerow([tag, img_path.name, len(cubes), f"{err:.2f}", f"{ms:.1f}"])

if __name__ == "__main__":
    main()
