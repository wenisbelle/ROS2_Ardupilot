# -*- coding: utf-8 -*-
"""Formation geometry: slot offsets (dx forward, dy left) relative to the formation
centre, in the formation frame (x = heading direction). Pure functions, no ROS."""
import math


def slots(shape, n, spacing):
    """List of n (dx, dy) offsets [m] for the formation `shape`."""
    if n <= 0:
        return []
    if shape == 'line':          # side by side, perpendicular to the heading
        return [(0.0, ((n - 1) / 2.0 - i) * spacing) for i in range(n)]
    if shape == 'column':        # one behind the other
        return [(((n - 1) / 2.0 - i) * spacing, 0.0) for i in range(n)]
    if shape == 'circle':
        if n == 1:
            return [(0.0, 0.0)]
        r = max(spacing, spacing / (2.0 * math.sin(math.pi / n)))
        return [(r * math.cos(2 * math.pi * i / n), r * math.sin(2 * math.pi * i / n)) for i in range(n)]
    if shape == 'grid':
        cols = math.ceil(math.sqrt(n))
        rows = math.ceil(n / cols)
        out = []
        for i in range(n):
            r, c = divmod(i, cols)
            out.append((((rows - 1) / 2.0 - r) * spacing, ((cols - 1) / 2.0 - c) * spacing))
        return out
    if shape == 'v':             # leader in front, the others behind on both sides
        out = [(0.0, 0.0)]
        for i in range(1, n):
            k = (i + 1) // 2
            side = 1 if i % 2 else -1
            out.append((-k * spacing * 0.8, side * k * spacing * 0.6))
        return out
    raise ValueError(f'unknown formation "{shape}" (line, column, circle, grid, v)')


def to_map(offsets, center, heading):
    """Rotate formation offsets by `heading` (rad, ENU) and translate to `center` (x, y)."""
    c, s = math.cos(heading), math.sin(heading)
    return [(center[0] + dx * c - dy * s, center[1] + dx * s + dy * c) for dx, dy in offsets]
