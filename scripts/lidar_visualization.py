from lidar.api import Lidar
import matplotlib.pyplot as plt
import numpy as np


def visualize_lidar():
    """
    Live LIDAR visualization using a polar plot.
    Angle is theta, radius is distance in meters.
    """
    lidar = Lidar()
    lidar.start()
    plt.ion()
    fig, ax = plt.subplots(subplot_kw={"projection": "polar"})
    scatter = ax.scatter([], [], s=2, color="blue")
    ax.set_title("LIDAR Visualization")

    # Draw rings every 1 meter
    for r in range(1, 11):
        ax.plot(
            np.linspace(0, 2 * np.pi, 360),
            [r] * 360,
            color="gray",
            lw=0.5,
            alpha=0.3,
            zorder=1,
        )
        # ax.set_rticks(range(1, 11))
        # ax.set_rlabel_position(135)

    try:
        while True:
            data = lidar.read()
            theta = []
            r = []
            for angle, (distance, quality) in data:
                if distance is not None and quality > 0:
                    theta.append(np.radians(angle))
                    r.append(distance / 1000.0)

            scatter.set_offsets(np.c_[theta, r])
            ax.set_title("LIDAR Polar Visualization ({} points)".format(len(r)))
            plt.draw()
            plt.pause(0.1)
    except KeyboardInterrupt:
        lidar.stop()
        plt.ioff()
        plt.show()


if __name__ == "__main__":
    visualize_lidar()
