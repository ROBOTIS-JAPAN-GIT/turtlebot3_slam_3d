#!/usr/bin/python

# http://scikit-learn.org/stable/auto_examples/cluster/plot_dbscan.html

import os
import yaml
import argparse
import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN

class DetectionClustering(object):
    def __init__(self, detections, eps=0.3, min_samples=10, plot=False, out_dir='out'):
        self.dbscan = DBSCAN(eps=eps, min_samples=min_samples)
        self.clusters = {}
        for name, pos in detections.iteritems():
            midpoints = self.compute(name, pos, plot=plot, out_dir=out_dir)
            if midpoints: self.clusters[name] = midpoints

    def compute(self, name, array, plot=False, out_dir='out'):
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

        if plot:
            # Black is removed and used for noise instead.
            unique_labels = set(labels)
            colors = [plt.cm.Spectral(each)
                      for each in np.linspace(0, 1, len(unique_labels))]

            for k, col in zip(unique_labels, colors):
                if k == -1: col = [0, 0, 0, 1] # Black is used for noise.

                class_member_mask = (labels == k)

                xy = X[class_member_mask & core_samples_mask]
                plt.plot(xy[:, 0], xy[:, 1], 'o', markerfacecolor=tuple(col),
                         markeredgecolor='k', markersize=14)

                mean = xy.mean(axis=0)
                plt.plot(mean[0], mean[1], 'k+', markeredgewidth=2, markersize=10)

                xy = X[class_member_mask & ~core_samples_mask]
                plt.plot(xy[:, 0], xy[:, 1], 'o', markerfacecolor=tuple(col),
                         markeredgecolor='k', markersize=6)

                output_file = out_dir + '/' + name + '.png'
                print 'Writing graph to {}...'.format(output_file)
                plt.title("Clustering result of class '{}'".format(name))
                plt.savefig(output_file)

        return result

if __name__ == '__main__':
    # Custom Positive Integer type
    def positive_int(val):
        i = int(val)
        if i <= 0:
            raise argparse.ArgumentTypeError('invalid positive_int value: {}'.format(val))
        return i

    # Argument Parser
    parser = argparse.ArgumentParser()
    parser.add_argument('-i', '--input_file', type=str, default='~/.ros/detections_raw.db')
    parser.add_argument('-o', '--output_file', type=str, default='detections_dbscan.db')
    parser.add_argument('-p', '--plot', type=bool, default=False)
    parser.add_argument('-d', '--output_directory', type=str, default='out',
                        help='Directory to where graphs are saved when --plot is set.')
    parser.add_argument('--eps', type=float, default=0.3)
    parser.add_argument('--min_samples', type=positive_int, default=10)

    args, _ = parser.parse_known_args()

    if args.plot and not os.path.exists(args.output_directory):
        os.makedirs(args.output_directory)

    print 'Reading from {}...'.format(args.input_file)
    with open(os.path.expanduser(args.input_file), 'r') as infile:
        detections = yaml.load(infile)

    dc = DetectionClustering(detections, args.eps, args.min_samples,
                             args.plot, args.output_directory)

    print 'Writing to {}...'.format(args.output_file)
    with open(os.path.expanduser(args.output_file), 'w') as outfile:
        yaml.dump(dc.clusters, outfile)
