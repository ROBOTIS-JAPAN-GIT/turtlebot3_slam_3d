#!/usr/bin/python

# http://scikit-learn.org/stable/auto_examples/cluster/plot_dbscan.html

import numpy as np
from sklearn.cluster import DBSCAN

class DetectionClustering(object):
    def __init__(self, detections, eps=0.3, min_samples=10):
        self.dbscan = DBSCAN(eps=eps, min_samples=min_samples)
        self.clusters = {}
        for name, pos in detections.iteritems():
            midpoints = self.compute(pos)
            if midpoints: self.clusters[name] = midpoints

    def compute(self, array):
        # Compute DBSCAN
        X = np.array(array)
        db = self.dbscan.fit(X)
        core_samples_mask = np.zeros_like(db.labels_, dtype=bool)
        core_samples_mask[db.core_sample_indices_] = True
        labels = db.labels_

        # Number of clusters in labels, ignoring noise if present.
        # n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)

        # Compute midpoints
        label_types = set(labels)
        if -1 in label_types: label_types.remove(-1)
        result = []
        for k in label_types:
            class_member_mask = (labels == k)
            points = X[class_member_mask & core_samples_mask]
            mean = points.mean(axis=0)
            result.append(mean.tolist())
        return result
