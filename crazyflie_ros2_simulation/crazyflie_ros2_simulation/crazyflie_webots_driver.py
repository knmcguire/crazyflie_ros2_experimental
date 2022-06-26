import rclpy
from geometry_msgs.msg import Twist
from math import cos, sin, degrees, radians 
import sys
# Change this path to your crazyflie-firmware folder
sys.path.append('/home/knmcguire/Development/bitcraze/c/crazyflie-firmware')
import cffirmware


class MyRobotDriver:
    def init(self, webots_node, properties):
        self.robot = webots_node.robot

        ## Initialize motors
        self.m1_motor = self.robotrobot.getDevice("m1_motor")
        self.m1_motor.setPosition(float('inf'))
        self.m1_motor.setVelocity(-1)
        self.m2_motor = self.robotrobot.getDevice("m2_motor")
        self.m2_motor.setPosition(float('inf'))
        self.m2_motor.setVelocity(1)
        self.m3_motor = self.robotrobot.getDevice("m3_motor")
        self.m3_motor.setPosition(float('inf'))
        self.m3_motor.setVelocity(-1)
        self.m4_motor = self.robotrobot.getDevice("m4_motor")
        self.m4_motor.setPosition(float('inf'))
        self.m4_motor.setVelocity(1)

        self.target_twist = Twist()

        ## Initialize Sensors
        self.imu = self.robotrobot.getDevice("inertial unit")
        self.imu.enable(timestep)
        self.gps = self.robotrobot.getDevice("gps")
        self.gps.enable(timestep)
        self.gyro = self.robotrobot.getDevice("gyro")
        self.gyro.enable(timestep)
        self.range_front = self.robotrobot.getDevice("range_front")
        self.range_front.enable(timestep)
        self.range_left = self.robotrobot.getDevice("range_left")
        self.range_left.enable(timestep)
        self.range_back = self.robotrobot.getDevice("range_back")
        self.range_back.enable(timestep)
        self.range_right = self.robotrobot.getDevice("range_right")
        self.range_right.enable(timestep)

        ## Intialize Variables
        self.past_x_global = 0
        self.past_y_global = 0
        self.past_z_global = 0
        self.past_time = self.robot.getTime()

        cffirmware.controllerPidInit()

        rclpy.init(args=None)
        self.node = rclpy.create_node('crazyflie_webots_driver')
        
    def step(self):
        rclpy.spin_once(self.node, timeout_sec=0)

        dt = robot.getTime() - self.past_time

        ## Get measurements
        roll = self.imu.getRollPitchYaw()[0]
        pitch = self.imu.getRollPitchYaw()[1]
        yaw = self.imu.getRollPitchYaw()[2]
        roll_rate = self.gyro.getValues()[0]
        pitch_rate = self.gyro.getValues()[1]
        yaw_rate = self.gyro.getValues()[2]
        x_global = self.gps.getValues()[0]
        vx_global = (x_global - self.past_x_global)/dt
        y_global = gps.getValues()[1]
        vy_global = (y_global - past_y_global)/dt
        z_global = gps.getValues()[2]
        vz_global = (z_global - past_z_global)/dt


        ## Put measurement in state estimate
        # TODO replace these with a EKF python binding
        state = cffirmware.state_t()
        state.attitude.roll = degrees(roll)
        state.attitude.pitch = -degrees(pitch)
        state.attitude.yaw = degrees(yaw)
        state.position.x = x_global
        state.position.y = y_global
        state.position.z = z_global
        state.velocity.x = vx_global
        state.velocity.y = vy_global
        state.velocity.z = vz_global
        
        # Put gyro in sensor data
        sensors = cffirmware.sensorData_t()
        sensors.gyro.x = degrees(roll_rate)
        sensors.gyro.y = degrees(pitch_rate)
        sensors.gyro.z = degrees(yaw_rate)

        ## Fill in Setpoints
        setpoint = cffirmware.setpoint_t()
        setpoint.mode.z = cffirmware.modeAbs
        setpoint.position.z = 1.0
        setpoint.mode.yaw = cffirmware.modeVelocity
        setpoint.attitudeRate.yaw = degrees(yawDesired)
        setpoint.mode.x = cffirmware.modeVelocity
        setpoint.mode.y = cffirmware.modeVelocity
        setpoint.velocity.x = 0
        setpoint.velocity.y = 0
        setpoint.velocity_body = True

        ## Fill in Setpoints
        setpoint = cffirmware.setpoint_t()
        setpoint.mode.z = cffirmware.modeAbs
        setpoint.position.z = 1.0
        setpoint.mode.yaw = cffirmware.modeVelocity
        setpoint.attitudeRate.yaw = degrees(yawDesired)
        setpoint.mode.x = cffirmware.modeVelocity
        setpoint.mode.y = cffirmware.modeVelocity
        setpoint.velocity.x = forwardDesired
        setpoint.velocity.y = sidewaysDesired
        setpoint.velocity_body = True

        ## 
        cmd_roll = radians(control.roll)
        cmd_pitch = radians(control.pitch)
        cmd_yaw = -radians(control.yaw)
        cmd_thrust = control.thrust

        ## Motor mixing
        motorPower_m1 =  cmd_thrust - cmd_roll + cmd_pitch + cmd_yaw
        motorPower_m2 =  cmd_thrust - cmd_roll - cmd_pitch - cmd_yaw
        motorPower_m3 =  cmd_thrust + cmd_roll - cmd_pitch + cmd_yaw
        motorPower_m4 =  cmd_thrust + cmd_roll + cmd_pitch - cmd_yaw

        scaling = 1000 ##Todo, remove necessity of this scaling (SI units in firmware)
        self.m1_motor.setVelocity(-motorPower_m1/scaling)
        self.m2_motor.setVelocity(motorPower_m2/scaling)
        self.m3_motor.setVelocity(-motorPower_m3/scaling)
        self.m4_motor.setVelocity(motorPower_m4/scaling)

        self.past_time = self.robot.getTime()
        self.past_x_global = self.x_global
        self.past_y_global = self.y_global
        self.past_z_global = self.z_global