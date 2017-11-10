"""
Executes ros commands
"""

import subprocess
from tornado.gen import Task, Return, coroutine
import tornado.process
from tornado.ioloop import IOLoop
import time


class RosCommandExecuter():

    """
    Executes ros commands
    """

    VALID_COMMANDS = {'rostopic', 'rosservice'}

    def __init__(self, sendmsg):
        self.sendmsg = sendmsg
        self.current_process = None

    @coroutine
    def incoming(self, msg):
        """
        handles incoming ros cmd message
        """

        print 'RosCommandExecuter command received', msg
        cmd = msg['cmd']
        if cmd == 'STOP_CURRENT_EXECUTION':
            if self.current_process:
                self.current_process.kill()
        elif cmd in self.VALID_COMMANDS:
            args = msg['args']
            self.execute(cmd, args)
        else:
            self.sendRosResponse(
                data=['Invalid command: ' + cmd], running=False)
            print "Invalid command: ", cmd

    def sendRosResponse(self, data=None, running=True):
        self.sendmsg(
            {'op': 'ros_response', 'msg': {'data': data}, 'running': running})

    def finish(self):
        """
        Dispose Commander resources
        """
        if self.current_process:
            self.current_process.kill()

    def execute(self, cmd, args):
        try:
            self.current_process = subprocess.Popen(
                [cmd] + args,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT)

            # we must buffer otherwise too frequent messages will overload the client
            buffering = {
                'data': [],
                'start_time': time.time()
            }

            def stdout_data_available(fd, mode):
                try:
                    if mode == 16:  # stream closed
                        IOLoop.current().remove_handler(fd)
                        self.sendRosResponse(
                            data=buffering['data'], running=False)

                        if self.current_process:  # could have been killed by STOP_CURRENT_EXECUTION
                            self.current_process.stdout.close()
                            self.current_process.wait()
                            self.current_process = None

                        return

                    data = buffering['data']
                    data.append(fd.readline())
                    if time.time() - buffering['start_time'] > 0.2:
                        self.sendRosResponse(data=data)
                        buffering['data'] = []
                        buffering['start_time'] = time.time()
                        
                except Exception as e:
                    print "ERROR stdout_data_available", e

            # handle file descriptor with the event loop
            IOLoop.current().add_handler(
                self.current_process.stdout, stdout_data_available, IOLoop.current().READ | IOLoop.current().ERROR)
        except Exception as e:
            print "ERROR", e
