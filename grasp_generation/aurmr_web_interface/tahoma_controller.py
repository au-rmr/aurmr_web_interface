import rospy
from aurmr_tasks.common.tahoma import Tahoma
from aurmr_tasks.common import motion
from aurmr_tasks.common import states

from geometry_msgs.msg import PoseStamped

from smach import State, StateMachine

from aurmr_web_interface.srv import (
    ExecuteGrasp,
    ExecuteGraspResponse
)

robot = None

def handle_execute_grasp(req):
    global robot

    p = PoseStamped()
    p.header.frame_id = "world"
    p.pose = req.pose

    grasp_sm = StateMachine(["succeeded", "preempted", "aborted"], input_keys=[], output_keys=[])

    with grasp_sm:
        StateMachine.add_auto("CLEAR_SCENE", motion.ClearCollisionGeometry(robot), ["succeeded"])
        StateMachine.add_auto("SETUP_COLLISION_SCENE", motion.AddPodCollisionGeometry(robot), ["succeeded"])
        StateMachine.add_auto("MOVE_TO_POSE", motion.MoveEndEffectorToPose(robot, p), ['succeeded'])
    
    grasp_sm.execute()
    rospy.sleep(60)

    return ExecuteGraspResponse("success")

def execute_grasp_server():
    global robot
    rospy.init_node('execute_grasp_server')

    rospy.Service('execute_grasp', ExecuteGrasp, handle_execute_grasp)

    simulation = rospy.get_param("/use_sim_time", False)
    robot = Tahoma(simulation)

    rospy.spin()

if __name__ == "__main__":
    execute_grasp_server()