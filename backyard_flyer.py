import argparse
import time
from enum import Enum

import numpy as np

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection, WebSocketConnection  # noqa: F401
from udacidrone.messaging import MsgID


class States(Enum):
    MANUAL = 0
    ARMING = 1
    TAKEOFF = 2
    WAYPOINT = 3
    LANDING = 4
    DISARMING = 5


class BackyardFlyer(Drone):

    def __init__(self, connection):
        super().__init__(connection)
        self.target_position = np.array([0.0, 0.0, 0.0])
        self.target_altitude = 0
        self.all_waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL
        self.target_altitude = 3
        self.all_waypoints = self.calculate_box()

        # TODO: Register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        """
        This triggers when `MsgID.LOCAL_POSITION` is received and self.local_position contains new data
        """

        if self.flight_state == States.TAKEOFF:
            if self.is_altitude_reached(d=0.95):
                self.waypoint_transition()

        if self.flight_state == States.WAYPOINT:
            if self.is_target_reached(d=0.8) and self.local_velocity_absolute() < 1:
                if len(self.all_waypoints) == 0:
                    self.landing_transition()
                else:
                    self.waypoint_transition()

    def velocity_callback(self):
        """
        This triggers when `MsgID.LOCAL_VELOCITY` is received and self.local_velocity contains new data
        """
        if self.flight_state == States.LANDING:
            if self.is_altitude_reached(d=0.9):
                self.disarming_transition()

    def state_callback(self):
        """
        This triggers when `MsgID.STATE` is received and self.armed and self.guided contain new data
        """
        if not self.in_mission:
            return

        if self.flight_state == States.MANUAL:
            self.arming_transition()
        elif self.flight_state == States.ARMING:
            self.takeoff_transition()
        elif self.flight_state == States.DISARMING:
            self.manual_transition()

    def local_velocity_absolute(self):
        return np.linalg.norm(self.local_velocity, 2)

    def is_altitude_reached(self, d=0.95):
        diff = abs(self.local_position[2]) - self.target_altitude
        # print(diff)
        return abs(diff) < 1 - d

    def is_target_reached(self, d=0.95):
        """
        check if local_position within d of target_position
        """

        # z axis of local_position is inverted
        diff_position = np.array([
            self.target_position[0] - self.local_position[0],
            self.target_position[1] - self.local_position[1],
            self.target_position[2] + self.local_position[2]])

        norm = np.linalg.norm(diff_position, 2)
        # print(self.target_position, self.local_position, np.linalg.norm(diff_position, 2), self.local_velocity_absolute())
        return norm < 1 - d

    def calculate_box(self):
        """
        1. Return waypoints to fly a box
        """
        return np.array(
            [[10.0,  0.0, self.target_altitude],
            [ 10.0, 10.0, self.target_altitude],
            [  0.0, 10.0, self.target_altitude],
            [  0.0,  0.0, self.target_altitude]])

    def arming_transition(self):
        """
        1. Take control of the drone
        2. Pass an arming command
        3. Set the home location to current position
        4. Transition to the ARMING state
        """
        print("arming transition")

        self.take_control()
        time.sleep(1)
        self.arm()

        # set current location to the home position
        self.set_home_position(
            self.global_position[0],
            self.global_position[1],
            self.global_position[2])

        self.flight_state = States.ARMING

    def takeoff_transition(self):
        """
        1. Set target_position altitude to 3.0m
        2. Command a takeoff to 3.0m
        3. Transition to the TAKEOFF state
        """
        print("takeoff transition")

        # takeoff to target_altitude
        self.target_position = np.array([0.0, 0.0, self.target_altitude])
        self.takeoff(self.target_altitude)
        self.flight_state = States.TAKEOFF

    def waypoint_transition(self):
        """
        1. Command the next waypoint position
        2. Transition to WAYPOINT state
        """
        print("waypoint transition")

        self.target_position = self.all_waypoints[0]
        self.all_waypoints = self.all_waypoints[1:]

        self.cmd_position(
            self.target_position[0],
            self.target_position[1],
            self.target_position[2],
            0)
        self.flight_state = States.WAYPOINT

    def landing_transition(self):
        """
        1. Command the drone to land
        2. Transition to the LANDING state
        """
        print("landing transition")

        self.land()
        self.target_altitude = 0
        self.flight_state = States.LANDING

    def disarming_transition(self):
        """
        1. Command the drone to disarm
        2. Transition to the DISARMING state
        """
        print("disarm transition")

        self.disarm()
        self.flight_state = States.DISARMING

    def manual_transition(self):
        """This method is provided

        1. Release control of the drone
        2. Stop the connection (and telemetry log)
        3. End the mission
        4. Transition to the MANUAL state
        """
        print("manual transition")

        self.release_control()
        self.stop()
        self.in_mission = False
        self.flight_state = States.MANUAL

    def start(self):
        """This method is provided

        1. Open a log file
        2. Start the drone connection
        3. Close the log file
        """
        print("Creating log file")
        self.start_log("Logs", "NavLog.txt")
        print("starting connection")
        self.connection.start()
        print("Closing log file")
        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), threaded=False, PX4=False)
    #conn = WebSocketConnection('ws://{0}:{1}'.format(args.host, args.port))
    drone = BackyardFlyer(conn)
    time.sleep(2)
    drone.start()
