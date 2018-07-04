'''In this file you need to implement remote procedure call (RPC) client

* The agent_server.py has to be implemented first (at least one function is implemented and exported)
* Please implement functions in ClientAgent first, which should request remote call directly
* The PostHandler can be implement in the last step, it provides non-blocking functions, e.g. agent.post.execute_keyframes
 * Hints: [threading](https://docs.python.org/2/library/threading.html) may be needed for monitoring if the task is done
'''

import weakref
from threading import Thread
import requests

class PostHandler(object):
    '''the post hander wraps function to be excuted in paralle
    '''
    def __init__(self, obj):
        self.proxy = weakref.proxy(obj)

    def execute_keyframes(self, keyframes):
        '''non-blocking call of ClientAgent.execute_keyframes'''
        Thread(target = self.proxy.execute_keyframes, kwargs = {'keyframes': keyframes}).start()

    def set_transform(self, effector_name, transform):
        '''non-blocking call of ClientAgent.set_transform'''
        Thread(target = self.proxy.set_transform, kwargs = {'effector_name': effector_name, 'transform': transform}).start()


class ClientAgent(object):
    '''ClientAgent request RPC service from remote server
    '''
    URL = 'http://127.0.0.1:3000/jsonrpc'
    HEADERS = {'content-type': 'application/json'}

    def build_request(self, method, params = []):
        if not hasattr(self, 'id'):
            self.id = 0
        else:
            self.id += 1

        return requests.post(URL, data = json.dumps({'method': method, 'params': params, 'jsonrpc': '2.0', 'id': self.id}), headers = headers).json()

    def __init__(self):
        self.post = PostHandler(self)

    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        return self.send('get_angle', [joint_name])

    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        return self.send('set_angle', [joint_name, angle])

    def get_posture(self):
        '''return current posture of robot'''
        return self.send('get_posture')

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        return self.send('execute_keyframes', [keyframes])

    def get_transform(self, name):
        '''get transform with given name
        '''
        return self.send('get_transform', [name])

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        return self.send('set_transform', [effector_name, joint_name])

if __name__ == '__main__':
    agent = ClientAgent()
