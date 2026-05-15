"""Dataset loader for VPR evaluation.

Loads database and query images from two folders. If filenames follow the
convention ``@x@y@z@...@.jpg``, positional ground truth is computed for
recall@K evaluation.

Evaluation-only -- the live pipeline (``vpr_preprocess.py`` /
``vpr_detect.py``) does not use this class.
"""

import os
from glob import glob

import numpy as np
import torch.utils.data as data
import torchvision.transforms as transforms
from PIL import Image
from sklearn.neighbors import NearestNeighbors


def read_images_paths(dataset_folder):
    if not os.path.exists(dataset_folder):
        raise FileNotFoundError(f"Folder {dataset_folder} does not exist")

    file_with_paths = dataset_folder + "_images_paths.txt"
    if os.path.exists(file_with_paths):
        with open(file_with_paths, "r") as file:
            images_paths = file.read().splitlines()
        images_paths = [dataset_folder + "/" + path for path in images_paths]
        if not os.path.exists(images_paths[0]):
            raise FileNotFoundError(
                f"Image with path {images_paths[0]} does not exist within {dataset_folder}."
            )
    else:
        images_paths = sorted(glob(f"{dataset_folder}/**/*", recursive=True))
        images_paths = [
            p for p in images_paths
            if os.path.isfile(p) and os.path.splitext(p)[1].lower() in [".jpg", ".jpeg", ".png"]
        ]
        if len(images_paths) == 0:
            raise FileNotFoundError(f"Directory {dataset_folder} does not contain any images")

    print (f"Found {len(images_paths)} images in {dataset_folder}")        
    return images_paths


class VPRDataset(data.Dataset):
    """Dataset with images from database and queries.

    Parameters
    ----------
    database_folder : str
        Path to folder with database images.
    queries_folder : str
        Path to folder with query images.
    positive_dist_threshold : int
        Distance in meters for a prediction to be considered a positive.
    image_size : list[int] or None
        Optional resizing shape (H, W).
    use_labels : bool
        If True, parse coordinates from filenames for ground-truth recall.
    """

    def __init__(self, database_folder, queries_folder, positive_dist_threshold=25,
                 image_size=None, use_labels=True):
        super().__init__()

        self.database_paths = read_images_paths(database_folder)
        self.queries_paths = read_images_paths(queries_folder)
        self.images_paths = list(self.database_paths) + list(self.queries_paths)
        self.num_database = len(self.database_paths)
        self.num_queries = len(self.queries_paths)

        if use_labels:
            try:
                image_path = self.database_paths[0]
                float(image_path.split("@")[1])
                float(image_path.split("@")[2])
                float(image_path.split("@")[3])
            except Exception:
                raise ValueError(
                    "Filenames must be path/to/file/@x@y@z@...@.jpg for labeled mode. "
                    f"Got: {self.database_paths[0]}"
                )

            self.database_utms = np.array(
                [(path.split("@")[1], path.split("@")[2], path.split("@")[3]) for path in self.database_paths]
            ).astype(float)
            self.queries_utms = np.array(
                [(path.split("@")[1], path.split("@")[2], path.split("@")[3]) for path in self.queries_paths]
            ).astype(float)

            # 分别用 xy 平面距离和 z 差异来判断正样本
            # 条件：xy 距离 <= positive_dist_threshold 且 |z差| <= 1.0
            database_xy = self.database_utms[:, :2]
            database_z = self.database_utms[:, 2]
            queries_xy = self.queries_utms[:, :2]
            queries_z = self.queries_utms[:, 2]

            # 用 xy 坐标做 knn 搜索
            knn = NearestNeighbors(n_jobs=-1)
            knn.fit(database_xy)
            xy_neighbors = knn.radius_neighbors(
                queries_xy, radius=positive_dist_threshold, return_distance=False
            )

            # 过滤：只保留 z 差异在 1.0 以内的
            z_threshold = 1.0
            self.positives_per_query = []
            for qi, db_indices in enumerate(xy_neighbors):
                valid_indices = [
                    idx for idx in db_indices
                    if abs(queries_z[qi] - database_z[idx]) <= z_threshold
                ]
                self.positives_per_query.append(np.array(valid_indices))
            self.positives_per_query = np.array(self.positives_per_query, dtype=object)

        transformations = [
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
        ]
        if image_size:
            transformations.append(transforms.Resize(size=image_size, antialias=True))
        self.transform = transforms.Compose(transformations)

    def __getitem__(self, index):
        pil_img = Image.open(self.images_paths[index]).convert("RGB")
        return self.transform(pil_img), index

    def __len__(self):
        return len(self.images_paths)

    def __repr__(self):
        return f"< #queries: {self.num_queries}; #database: {self.num_database} >"

    def get_positives(self):
        return self.positives_per_query
