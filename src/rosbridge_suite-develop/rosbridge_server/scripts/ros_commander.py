"""
Executes ros commands
"""

import subprocess, os
from tornado.gen import Task, Return, coroutine
import tornado.process
from tornado.ioloop import IOLoop
import time


class RosCommandExecuter():
    """
    Executes ros commands
    """

    VALID_COMMANDS = {'rostopic', 'rosservice', 'rosmsg', 'rossrv', 'rosnode', 'help'}
    ROS_COMPLETER_SCRIPT = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'roscommand_completer.sh')

    def __init__(self, sendmsg):
        self.sendmsg = sendmsg
        self.current_process = None

    @coroutine
    def incoming(self, msg):
        """
        handles incoming ros cmd message
        """

        cmd = msg['cmd']
        if cmd == 'COMPLETE':
            self.complete_cmd(msg['args'])
        elif cmd == 'STOP_CURRENT_EXECUTION':
            if self.current_process:
                self.current_process.kill()
        elif cmd in self.VALID_COMMANDS:
            self.execute(cmd, msg['args'])
        else:
            self.sendRosResponse(data=[{'data': 'Invalid command: ' + cmd, 'type': 'stderr'}], running=False)

    def sendRosResponse(self, data=None, running=True):
        self.sendmsg({'op': 'ros_response', 'msg': {'data': data}, 'running': running})

    def complete_cmd(self, cmd):
        res = self.retrieve_completion(cmd)
        if res:
            self.sendRosResponse(data=res, running=False)

    def retrieve_completion(self, cmd):
        parts = [p for p in cmd.split(' ') if p]
        if parts[0] == 'help':
            return None

        l = len(parts)
        if cmd[-1] == ' ':
            l += 1

        if l <= 1:
            completion = [c for c in self.VALID_COMMANDS if c.startswith(cmd)]
            return [{'data': completion, 'type': 'completion'}]

        if parts[0] not in self.VALID_COMMANDS:
            return [{'data': 'Invalid command: ' + parts[0], 'type': 'stderr'}]

        res = subprocess.check_output([self.ROS_COMPLETER_SCRIPT] + [cmd] + [str(max(1, l - 1))], shell=False)
        res = [l for l in res.split('\n') if l]
        return [{'data': res, 'type': 'completion'}]

    def finish(self):
        """
        Dispose Commander resources
        """
        if self.current_process:
            self.current_process.kill()

    def execute(self, cmd, args):
        try:
            self.current_process = subprocess.Popen([cmd] + args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)

            # we must buffer otherwise too frequent messages will overload the client
            buffering = {'data': [], 'start_time': time.time()}

            def stdout_data_available(fd, mode):
                try:
                    if mode == 16:  # stream closed
                        IOLoop.current().remove_handler(fd)
                        self.sendRosResponse(data=buffering['data'], running=False)

                        if self.current_process:  # could have been killed by STOP_CURRENT_EXECUTION
                            self.current_process.stdout.close()
                            self.current_process.wait()
                            self.current_process = None

                        return

                    data = buffering['data']
                    data.append({'data': fd.readline(), 'type': 'stdout'})
                    if time.time() - buffering['start_time'] > 0.2:
                        self.sendRosResponse(data=data)
                        buffering['data'] = []
                        buffering['start_time'] = time.time()

                except Exception as e:
                    print "ERROR stdout_data_available", e

            # handle file descriptor with the event loop
            IOLoop.current().add_handler(self.current_process.stdout, stdout_data_available,
                                         IOLoop.current().READ | IOLoop.current().ERROR)
        except Exception as e:
            print "ERROR", e
