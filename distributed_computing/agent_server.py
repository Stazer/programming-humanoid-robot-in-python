'''In this file you need to implement remote procedure call (RPC) server

* There are different RPC libraries for python, such as xmlrpclib, json-rpc. You are free to choose.
* The following functions have to be implemented and exported:
 * get_angle
 * set_angle
 * get_posture
 * execute_keyframes
 * get_transform
 * set_transform
* You can test RPC server with ipython before implementing agent_client.py
'''

import os
import sys
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'kinematics'))

from inverse_kinematics import InverseKinematicsAgent
from werkzeug.wrappers import Request, Response
from werkzeug.serving import run_simple
from jsonrpc import JSONRPCResponseManager, dispatcher

class ServerAgent(InverseKinematicsAgent):
    '''ServerAgent provides RPC service
    '''
    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        return self.perception.joint[joint_name]

    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        self.perception.joint[joint_name] = name

    def get_posture(self):
        '''return current posture of robot'''
        return self.recognize_posture()

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        self.keyframes = keyframes
        self.run()

    def get_transform(self, name):
        '''get transform with given name
        '''
        pass

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        super(ServerAgent, self).set_transform(effector_name, transform)

@Request.application
def application(request):
    agent = ServerAgent()

    for x in ['get_angle', 'set_angle', 'get_posture', 'execute_keyframes', 'get_transform', 'set_transform']:
        dispatcher[x] = getattr(agent, x)

    return Response(JSONRPCResponseManager.handle(request.data, dispatcher).json, mimetype = 'application/json')

if __name__ == '__main__':
    run_simple('localhost', 3000, application)
