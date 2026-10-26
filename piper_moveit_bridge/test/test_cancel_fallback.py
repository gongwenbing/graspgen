from types import SimpleNamespace as NS
from piper_moveit_bridge.execute_pregrasp import HoverExecution


def test_unconfirmed_cancellation_closes_both_software_gates_without_motor_commands():
    calls=[];clients=[]
    class Client:
        def wait_for_service(self,timeout_sec):return True
        def call_async(self,req):
            assert req.data is False
            return NS(done=lambda:True,result=lambda:NS(success=True))
    class Node:
        def create_client(self,kind,name):calls.append(name);c=Client();clients.append(c);return c
        def destroy_client(self,c):assert c in clients
    r=HoverExecution();r.node=Node();r.handle=NS(cancel_goal_async=lambda:NS(done=lambda:False))
    def wait(done,timeout,error=''):
        if not done():raise RuntimeError(error)
    r.wait=wait;r.close()
    assert calls==['/piper_can_gateway/enable_outputs','/piper_trajectory_bridge/enable_execution']


def test_successful_execution_never_closes_gates_as_cleanup():
    r=HoverExecution();r.finished=True;r.handle=object()
    r.disable_software_gates=lambda:(_ for _ in ()).throw(AssertionError('must not close'))
    r.close()
