'''In this exercise you need to use the learned classifier to recognize current posture of robot

* Tasks:
    1. load learned classifier in `PostureRecognitionAgent.__init__`
    2. recognize current posture in `PostureRecognitionAgent.recognize_posture`

* Hints:
    Let the robot execute different keyframes, and recognize these postures.

'''


from angle_interpolation import AngleInterpolationAgent
from sklearn import svm, metrics
import numpy as np
import pickle as pk
from keyframes import *
from os import listdir, path

ROBOT_POSE_DATA_DIR = './robot_pose_data/'

class PostureRecognitionAgent(AngleInterpolationAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(PostureRecognitionAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.posture = 'unknown'
        self.posture_classifier = None  # LOAD YOUR CLASSIFIER

    def think(self, perception):
        self.posture = self.recognize_posture(perception)
        return super(PostureRecognitionAgent, self).think(perception)

    def recognize_posture(self, perception):
        if not hasattr(self, 'classes'):
            self.classes = listdir(ROBOT_POSE_DATA_DIR)

            all_data = []
            all_target = []

            for i in range(0, len(self.classes)):
                filename = path.join(ROBOT_POSE_DATA_DIR, self.classes[i])
                data = pk.load(open(filename))
                target = [i] * len(data)

                all_data += data
                all_target += target

            clf = svm.SVC(gamma=0.001, C=100.)
            clf.fit(np.array(all_data), np.array(all_target))

            self.posture_classifier = clf.predict

        return self.classes[self.posture_classifier(np.array([perception.joint['LHipYawPitch'], perception.joint['RHipYawPitch'], perception.joint['LHipRoll'], perception.joint['RHipRoll'], perception.joint['LHipPitch'], perception.joint['RHipPitch'], perception.joint['LKneePitch'], perception.joint['RKneePitch'], perception.imu[0], perception.imu[1]]).reshape((1, -1)))[0]]

        return posture

if __name__ == '__main__':
    agent = PostureRecognitionAgent()
    agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
