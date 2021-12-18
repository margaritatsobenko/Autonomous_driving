from gym_duckietown.tasks.task_solution import TaskSolution
import numpy as np
import cv2


class DontCrushDuckieTaskSolution(TaskSolution):
    def __init__(self, generated_task):
        super().__init__(generated_task)

    def make_turn(self, env, turn, max_iter=15):
        if turn == 'left':
            sign_angle = 1
        else:
            sign_angle = -1

        for i in range(max_iter):
            img, _, _, _ = env.step([0.4, sign_angle])
            env.render()

        for i in range(3):
            img, _, _, _ = env.step([1, 0])
            env.render()

    def find_yellow_part(self, img):
        # найдем, какую часть картинки занимает уточка
        img_hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        col_low_bound = np.array([20, 100, 100])
        col_up_bound = np.array([30, 255, 255])
        mask = cv2.inRange(img_hsv, col_low_bound, col_up_bound)
        yellow_part = cv2.countNonZero(mask) / (img_hsv.shape[0] * img_hsv.shape[1]) * 100
        return yellow_part

    def stop(self, env, percent=2):
        condition = True
        while condition:
            img, _, _, _ = env.step([1, 0])  # img in RGB

            yellow_part = self.find_yellow_part(img)

            if yellow_part > percent:
                condition = False
                img, _, _, _ = env.step([0, 0])
                env.render()

    def solve(self):
        max_iter = 10
        env = self.generated_task['env']
        # getting the initial picture
        img, _, _, _ = env.step([0, 0])

        self.stop(env)

        # выход на новую полосу
        self.make_turn(env, turn="left")
        self.make_turn(env, turn="right")

        img, _, _, _ = env.step([0.1, 0])

        # едем прямо, пока утка не пропадет
        for i in range(max_iter):
            yellow_part = self.find_yellow_part(img)
            if yellow_part < 2:
                # выход на старую полосу
                self.make_turn(env, turn="right")
                self.make_turn(env, turn="left")
                break
            else:
                img, _, _, _ = env.step([1, 0])
                env.render()
        img, _, _, _ = env.step([1, 0])
        env.render()
        img, _, _, _ = env.step([0, 0])
        env.render()







