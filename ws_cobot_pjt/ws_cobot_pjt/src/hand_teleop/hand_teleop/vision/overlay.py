"""Mirrored camera view: hand skeleton, grounded wrist -> wrist arrow, grounding progress, robot status.
Keys: h HOME and pause, s stop in place and pause, c (paused) ground in place and continue, q / Esc quit."""
import cv2
import numpy as np

CONNECTIONS = [(0, 1), (1, 2), (2, 3), (3, 4), (0, 5), (5, 6), (6, 7), (7, 8), (5, 9), (9, 10), (10, 11), (11, 12),
               (9, 13), (13, 14), (14, 15), (15, 16), (13, 17), (17, 18), (18, 19), (19, 20), (0, 17)]
KEYS = {ord('h'): 'home', ord('s'): 'stop', ord('c'): 'continue', ord('q'): 'quit', 27: 'quit'}


class Overlay:
    def __init__(self, title='hand_teleop'):
        self.title = title

    def show(self, frame, hand, grounding, status):
        view = cv2.flip(frame, 1)
        width = view.shape[1]
        mirror = lambda point: (int(width - point[0]), int(point[1]))
        if hand is not None:
            for a, b in CONNECTIONS:
                cv2.line(view, mirror(hand.image[a]), mirror(hand.image[b]), (0, 255, 0), 2)
            if grounding.neutral is not None:
                cv2.arrowedLine(view, mirror(grounding.neutral.wrist), mirror(hand.wrist), (0, 200, 255), 2)
        lines = [f'robot: {status}']
        if grounding.id >= 0 and grounding.neutral is None:
            lines.append(f'grounding: open palm, hold still {grounding.progress * 100:.0f}%')
        out = grounding.output
        if out['tracked']:
            angle = np.degrees(2 * np.arccos(min(abs(out['rotation'][3]), 1.0)))
            lines.append(f'move {np.linalg.norm(out["offset"]):.0f} mm  turn {angle:.0f} deg  open {out["openness"]:.2f}')
        lines.append('keys: h home | s stop | c continue | q quit')
        for row, text in enumerate(lines):
            cv2.putText(view, text, (10, 25 + 25 * row), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.imshow(self.title, view)
        return KEYS.get(cv2.waitKey(1) & 0xFF)
