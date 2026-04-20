"""Fuses sensor sources into [L, C, R] risk scores and smooths with EMA."""

from indoor_nav.config import W_LIDAR, W_MIDAS, EMA_ALPHA


def fuse_sector_risks(
    midas_risks,
    yolo_obstacles,
    lidar_sectors=None,
):
    """Weighted decision of all sensors into [L, C, R], then YOLO overlay on top."""
    risks = [0.0, 0.0, 0.0]

    # weighted sensor decision per sector
    for i in range(3):
        sources = []

        sources.append((W_MIDAS, midas_risks[i]))

        if lidar_sectors is not None:
            sources.append((W_LIDAR, lidar_sectors[i]))  # already 0–1 risk


        # absent sensors get their weight redistributed automatically
        total_weight = sum(w for w, _ in sources)
        risks[i] = (
            sum(w * r for w, r in sources) / total_weight
            if total_weight > 0
            else 0.0
        )

    # YOLO can only raise risk
    # for sector_idx, proximity, _box in yolo_obstacles:
    #     if proximity > risks[sector_idx]:
    #         risks[sector_idx] = proximity

    return risks


class RiskSmoother:
    """EMA filter to smooth out frame-to-frame noise in risk values."""

    def __init__(self, alpha=EMA_ALPHA):
        self.alpha = alpha
        self._smooth = [0.0, 0.0, 0.0]

    def update(self, raw_risks):
        """Blend new values into smoothed state, returns [L, C, R]."""
        for i in range(3):
            self._smooth[i] += self.alpha * (raw_risks[i] - self._smooth[i])
        return list(self._smooth)

    def reset(self):
        """Reset to zero."""
        self._smooth = [0.0, 0.0, 0.0]
