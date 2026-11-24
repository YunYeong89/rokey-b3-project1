"""One PNG containing the existing report groups as real Matplotlib subfigures."""
import math

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


class FigureBook:
    def __init__(self, path, panels):
        self.path, self.index = path, 0
        path.parent.mkdir(parents=True, exist_ok=True)
        rows = math.ceil(panels / 2)
        self.figure = plt.figure(figsize=(28, rows * 6), layout='constrained')
        self.panels = self.figure.subfigures(rows, 2, squeeze=False).ravel()

    def next(self):
        panel = self.panels[self.index]
        panel.set_visible(True)
        self.index += 1
        return panel

    def subplots(self, *args, **kwargs):
        kwargs.pop('figsize', None)
        panel = self.next()
        return panel, panel.subplots(*args, **kwargs)

    def save(self):
        for panel in self.panels[self.index:]:
            panel.set_visible(False)
        self.figure.savefig(self.path, dpi=120)
        return self.path

    def close(self):
        plt.close(self.figure)
