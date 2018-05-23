'''In this exercise you need to implement an angle interploation function which makes NAO executes keyframe motion

* Tasks:
    1. complete the code in `AngleInterpolationAgent.angle_interpolation`,
       you are free to use splines interploation or Bezier interploation,
       but the keyframes provided are for Bezier curves, you can simply ignore some data for splines interploation,
       please refer data format below for details.
    2. try different keyframes from `keyframes` folder

* Keyframe data format:
    keyframe := (names, times, keys)
    names := [str, ...]  # list of joint names
    times := [[float, float, ...], [float, float, ...], ...]
    # times is a matrix of floats: Each line corresponding to a joint, and column element to a key.
    keys := [[float, [int, float, float], [int, float, float]], ...]
    # keys is a list of angles in radians or an array of arrays each containing [float angle, Handle1, Handle2],
    # where Handle is [int InterpolationType, float dTime, float dAngle] describing the handle offsets relative
    # to the angle and time of the point. The first Bezier param describes the handle that controls the curve
    # preceding the point, the second describes the curve following the point.
'''


from pid import PIDAgent
from keyframes import hello


class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def bezier_with(self, p0, p1, p2, p3, i):
        return (1 - i) ** 3 * p0 \
            + 3 * (1 - i) ** 2 * i * p1 \
            + 3 * (1 - i) * i ** 2 * p2 \
            + i ** 3 * p3

    def bezier_params(self, dt, i, name, time, key, joint):
        if dt < time[0] and i == 0:
            return 0.0, time[0], joint, key[0][0]
        elif time[i] < dt < time[i+1]:
            return time[i], time[i + 1], key[i][0], key[i + 1][0]

        return None, None, None, None

    def angle_interpolation(self, keyframes, perception):
        if not hasattr(self, 'start_time'):
            self.start_time = perception.time

        target_joints = {}

        names, times, keys = keyframes

        for i in range(len(names)):
            name = names[i]
            time = times[i]
            key = keys[i]

            if name not in self.joint_names:
                continue

            dt = perception.time - self.start_time

            for j in range(len(time) - 1):
                t0, t3, p0, p3 = self.bezier_params(dt, j, name, time, key, perception.joint[name])

                if t0 == None or t3 == None or p0 == None or p3 == None:
                    continue

                target_joints[name] = self.bezier_with(p0, key[j][1][1] + p0, key[j][2][1] + p3, p3, (dt - t0) / (t3 - t0))

        return target_joints

if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
