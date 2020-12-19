import rospy
from spawn_ball import SpawnModel
import zmq


class JoyconSpawner()




if __name__ == '__main__':
    


    sm = SpawnModel()
    # sm.parseUserInputs()

    sm.setModelName("ping_pong_ball")
    sm.callDeleteService()
    sm.setPose(pose)
    sm.callSpawnService(vel)
    print("spawning " + sm.model_name)
    rospy.sleep(1)
