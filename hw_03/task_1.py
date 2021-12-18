from gym_duckietown.tasks.task_solution import TaskSolution
import numpy as np
import cv2


class DontCrushDuckieTaskSolution(TaskSolution):
    def __init__(self, generated_task):
        super().__init__(generated_task)

    def solve(self):
        env = self.generated_task['env']
        # getting the initial picture
        img, _, _, _ = env.step([0, 0])

        condition = True
        while condition:
            img, _, _, _ = env.step([1, 0]) # img in RGB

            # найдем, какую часть картинки занимает уточка
            img_hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
            col_low_bound = np.array([20, 100, 100])
            col_up_bound = np.array([30, 255, 255])
            mask = cv2.inRange(img_hsv, col_low_bound, col_up_bound)
            yellow_part = cv2.countNonZero(mask) / (img_hsv.shape[0] * img_hsv.shape[1]) * 100

            if yellow_part > 10:
                condition = False
                img, _, _, _ = env.step([0, 0])
                env.render()

