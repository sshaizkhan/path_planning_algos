from py_rrt.rrt import RRT

import rclpy
from rclpy.node import Node

show_animation = True
import matplotlib.pyplot as plt


def main(args=None):

    print("start " + __file__)

    # ====Search Path with RRT====
    obstacleList = [
        (5, 5, 1),
        (3, 6, 2),
        (3, 8, 2),
        (3, 10, 2),
        (7, 5, 2),
        (9, 5, 2),
        (8, 10, 1),
    ]  # [x, y, radius]
    # Set Initial parameters
    rrt = RRT(
        start=[0, 0],
        goal=[6.0, 10.0],
        rand_area=[-2, 15],
        obstacle_list=obstacleList,
        # play_area=[0, 10, 0, 14]
        robot_radius=0.8,
    )
    path = rrt.planning(animation=show_animation)

    if path is None:
        print("Cannot find path")
    else:

        print("found path!!")

        # Draw final path
        if show_animation:
            rrt.draw_graph()
            plt.plot([x for (x, y) in path], [y for (x, y) in path], "-r")
            plt.grid(True)
            plt.pause(0.01)  # Need for Mac
            plt.show()


if __name__ == "__main__":
    main()
