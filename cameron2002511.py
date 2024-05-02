"""
Name: Rory Cameron
Student Number: 2002511
Supervisor: Mark Zarb

"""

# ====== Imports ======
from time import sleep, time
import os
import sys
import torch
import numpy as np

import dronekit_sitl
from dronekit import connect, VehicleMode, LocationGlobalRelative

from pymavlink import mavutil

import cv2
from ultralytics import YOLO

from supervision.draw.color import ColorPalette
from supervision import Detections, BoxAnnotator

from imutils.video import FPS
from scipy.spatial import distance as dist

import math

import threading


# ====== Connect to UAV ======
sitl = dronekit_sitl.start_default()
connection_string = "127.0.0.1:14551" # UDP endpoint for copter simulation
# connection_string = "/dev/ttyAMA0" # UDP endpoint for connection to pixhawk via serial port

vehicle = connect(connection_string, wait_ready=True)
vehicle.wait_ready(True, raise_eception=False)
print("\n============\nConnected to UAV: \n%s\n============" % (connection_string))


"""
# ====== Initial parameters ======
print("\n====== Initial parameter set up ======")
targetAltitude = int(input("Enter target altitude (M): "))
boxWidth = input("Enter box dimension (M): ")
startingCoordX = float(input("Enter starting coordinates (Lat): "))
startingCoordY = float(input("Enter starting coordinates (Lon): "))
print("====== Parameters set ======")
"""

def get_valid_integer_input(prompt):
    while True:
        user_input = input(prompt)
        if user_input.isdigit():
            return int(user_input)
        else:
            print("Please enter a valid integer.")

# Function to validate float input
def get_valid_float_input(prompt):
    while True:
        user_input = input(prompt)
        try:
            return float(user_input)
        except ValueError:
            print("Please enter a valid number.")

# ====== Initial parameters ======
print("\n====== Initial parameter set up ======")
while True:
    targetAltitude = get_valid_integer_input("Enter target altitude (M): ")
    if targetAltitude > 0:
        break
    else:
        print("Altitude must be a positive integer.")

while True:
    boxWidth = get_valid_integer_input("Enter box dimension (M): ")
    if boxWidth > 0:
        break
    else:
        print("Box dimension must be a positive number.")

startingCoordX = get_valid_float_input("Enter starting coordinates (Lat): ")
startingCoordY = get_valid_float_input("Enter starting coordinates (Lon): ")
print("====== Parameters set ======")


def arm_and_takeoff(targetAltitude):

    print("\n====== Arming and take off =====")

    print("Basic pre-arm checks")

    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        #time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.airspeed=20 # can change this
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...") # To be removed
        sleep(1)

    print("\n====== UAV armed ======")

    print("\n====== Taking off ======")
    vehicle.simple_takeoff(targetAltitude)

    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt) # To be removed
        if vehicle.location.global_relative_frame.alt >=targetAltitude * 0.95: # Stop just below target altitude
            print("\n====== Reached target altitude ======")
            break
        sleep(1)


def move(coordX, coordY, moveType):

    if moveType == "starting":
        print("\n====== Moving to starting location ======")
    else:
        print("\n====== Moving ======")

    location = LocationGlobalRelative(coordX, coordY, 20)
    vehicle.simple_goto(location)

    sleep(60) # Temperary pause to allow map to update
    print("\n====== UAV reached position ======")

def spin_360(vehicle):
    print("\n====== Initiating 360-degree spin ======")
    # Define the rate of spinning (yaw)
    spin_rate = 30  # degrees per second
    # Get the current heading
    current_heading = vehicle.heading
    # Calculate the target heading for a 360-degree spin
    target_heading = (current_heading + 360) % 360
    # Start the spin by gradually changing the heading
    while abs(vehicle.heading - target_heading) > 5:  # Continue spinning until close to the target heading
        # Calculate the direction of the spin (CW or CCW)
        if target_heading > current_heading:
            direction = 1  # Clockwise
        else:
            direction = -1  # Counter-clockwise
        # Change the heading
        change_heading(vehicle, spin_rate * direction, "360")
        sleep(1)  # Adjust the spinning rate by controlling the sleep duration
    print("\n====== Finished 360-degree spin ======")

def change_heading(vehicle, heading, type):

    current_heading = vehicle.heading

    heading_difference = heading - current_heading

    if type == "track":
        if heading_difference > 180:


            msg = vehicle.message_factory.command_long_encode(
            0, 0,  # System ID and Component ID (reserved)
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # Command ID
            0,  # Confirmation
            heading,  # Param 1: Target heading in degrees
            0,  # Param 2: Speed of turn in degrees per second (0 means instant)
            -1,  # Param 3: Direction -1:CCW, 1:CW
            1,  # Relative offset to the previous yaw
            0, 0, 0  # Unused parameters
        )

        else:

            msg = vehicle.message_factory.command_long_encode(
            0, 0,  
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,  
            0, 
            heading, 
            0, 
            1,  
            1,  
            0, 0, 0  
        )
    elif type == "360":
         msg = vehicle.message_factory.command_long_encode(
            0, 0,  
            mavutil.mavlink.MAV_CMD_CONDITION_YAW, 
            0,  
            heading,  
            0, 
            1, 
            1, 
            0, 0, 0 
        )


    vehicle.send_mavlink(msg)

def manualMove(direction):

    # print("\n ====== THIS HAS BEEN CALLED")
    current_heading = vehicle.heading

    if direction == "right":
        current_heading += 90

        msg = vehicle.message_factory.command_long_encode(
            0, 0, 
            mavutil.mavlink.MAV_CMD_CONDITION_YAW, 
            0, 
            current_heading, 
            0, 
            1,  
            1, 
            0, 0, 0  
        )

        vehicle.send_mavlink(msg)
    elif direction == "backwards":
        current_heading -= 180

        msg = vehicle.message_factory.command_long_encode(
            0, 0,  
            mavutil.mavlink.MAV_CMD_CONDITION_YAW, 
            0, 
            current_heading, 
            0,  
            1, 
            1, 
            0, 0, 0  
        )

        vehicle.send_mavlink(msg)
    elif direction == "left":
        current_heading -= 90

        msg = vehicle.message_factory.command_long_encode(
            0, 0,  
            mavutil.mavlink.MAV_CMD_CONDITION_YAW, 
            0,  
            current_heading,  
            0,  
            -1,  
            1,  
            0, 0, 0  
        )

        vehicle.send_mavlink(msg)

    
    def forward():
        msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_BODY_NED, # frame Needs to be MAV_FRAME_BODY_NED for forward/back left/right control.
            0b0000111111000111, # type_mask
            0, 0, 0, # x, y, z positions (not used)
            0, boxWidth, 0, # m/s
            0, 0, 0, # x, y, z acceleration
            0, 0)
        for x in range(0,10): # fix duration thingy
            vehicle.send_mavlink(msg)
            time.sleep(1)

    forward()


class ObjectDetection:

    def __init__(self, capture_index, vehicle, boxWidth):
        self.capture_index = capture_index
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        print("Using Device: ", self.device)
        self.model = self.load_model()
        self.CLASS_NAMES_DICT = self.model.names
        self.box_annotator = BoxAnnotator(thickness=3, text_thickness=3, text_scale=1.5)
        self.tracker = cv2.TrackerCSRT_create()  # Initialize the tracker
        self.vehicle = vehicle

    def load_model(self):
        model = YOLO("yolov8n.pt")
        model.fuse()
        return model
    
    def predict(self, frame):
        results = self.model(frame)
        return results
    
    def track_object(self, frame, bbox):
        if frame is None or frame.size == 0:  # Check if the frame is empty
            print("Empty frame or invalid dimensions")
            return frame, None

        try:
            ok, bbox = self.tracker.update(frame)
            if ok:
                # Draw bounding box
                p1 = (int(bbox[0]), int(bbox[1]))
                p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
                cv2.rectangle(frame, p1, p2, (0, 255, 0), 2, 1)
                # Move the drone to follow the human
                self.move_drone_to_target(p1, p2)
            else:
                # Tracking failure
                cv2.putText(frame, "Tracking failure detected", (100, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0,0,255), 2)
        except Exception as e:
            print("Error tracking object:", e)
            return frame, None

        return frame, bbox

    def move_drone_to_target(self, p1, p2):
        if p1 is not None and p2 is not None:
            # Calculate the center of the bounding box
            target_x = (p1[0] + p2[0]) // 2
            target_y = (p1[1] + p2[1]) // 2
            # Move the drone towards the target location


            target_angle_rad = math.atan2(target_y - self.vehicle.location.global_relative_frame.lat,
                                        target_x - self.vehicle.location.global_relative_frame.lon)
            # Convert the angle from radians to degrees
            target_angle_deg = math.degrees(target_angle_rad)

            # Calculate the difference between current heading and target heading
            heading_difference = target_angle_deg - self.vehicle.heading

            # Apply proportional control to yaw change
            yaw_change = heading_difference * 0.1  # Adjust the proportional control gain as needed

            # Calculate the new target heading after applying proportional control
            target_heading_deg = self.vehicle.heading + yaw_change

            # Turn the drone to face the new target angle
            print("Turning drone to face target angle:", target_heading_deg)
            change_heading(vehicle, target_heading_deg, "track")

            print("Moving drone to target location:", target_x, target_y)
            vehicle.simple_goto(LocationGlobalRelative(target_x, target_y, vehicle.location.global_relative_frame.alt))

    def plot_bboxes(self, results, frame):

        human_detected = False

        for result in results[0]:
            class_id = result.boxes.cls.cpu().numpy().astype(int)
            if class_id == 0:
                print("\n====== Human detected ======")

                human_detected = True

                detections = Detections(
                    xyxy=result.boxes.xyxy.cpu().numpy(),
                    confidence=result.boxes.conf.cpu().numpy(),
                    class_id=result.boxes.cls.cpu().numpy().astype(int),
                )
                for box in detections.xyxy:
                    # Convert box to tuple of integers if not already
                    box = tuple(map(int, box))
                    # Start tracking the detected human
                    self.tracker.init(frame, (box[0], box[1], box[2]-box[0], box[3]-box[1])) # (x, y, w, h)
                frame = self.box_annotator.annotate(frame, detections)

        if not human_detected:
            print("\n====== No human detected ======")
        
        return frame
                

    def __call__(self):
        cap = cv2.VideoCapture(self.capture_index, cv2.CAP_DSHOW)
        assert cap.isOpened()
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

      
        while True:
            start_time = time()
            ret, frame = cap.read()
            assert ret
            
            results = self.predict(frame)
            frame = self.plot_bboxes(results, frame)
            
            # Track objects
            #frame = self.track_object(frame, None)
            frame, _ = self.track_object(frame, None)
      
            end_time = time()
            fps = 1 / np.round(end_time - start_time, 2)
             
            cv2.putText(frame, f'FPS: {int(fps)}', (20,70), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0,255,0), 2)
            
            cv2.imshow('YOLOv8 Detection', frame)
 
            """"           
            if cv2.waitKey(5) & 0xFF == 27:
                break
            """

            # Implementation not complete  
            key = cv2.waitKey(1) & 0xFF

            if key == 27:
                break
            elif key == ord("w"):
                print("\n ====== MOVING FORWARD")
                manualMove("forward")
                break
            elif key == ord("d"):
                print("\n ====== MOVING RIGHT")
                manualMove("right")
                break
            elif key == ord("s"):
                print("\n ====== MOVING BACKWARDS")
                manualMove("backwards")
                break
            elif key == ord("a"):
                print("\n ====== MOVING LEFT")
                manualMove("left")
                break            
        
        cap.release()
        cv2.destroyAllWindows()



arm_and_takeoff(targetAltitude)
move(startingCoordX,startingCoordY, "starting")

spin_360(vehicle)

detector = ObjectDetection(capture_index=0, vehicle=vehicle, boxWidth=boxWidth)
detector()




