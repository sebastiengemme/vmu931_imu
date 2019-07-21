#!/usr/bin/env python3

# Software License Agreement (BSD License)
#
# Copyright (c) 2015, Robotnik Automation SLL
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Robotnik Automation SSL nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rclpy
from rclpy.node import Node
from rclpy.time import Time

import time, threading

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped, QuaternionStamped
from std_msgs.msg import Float32
from . import vmu931_driver as vmu
from . import dato
from vmu931_msgs.msg import VMUState as VmuState
import math
from std_srvs.srv import Trigger

DEFAULT_FREQ = float(200.0)
MAX_FREQ = float(1000.0)
VMU931_MODE_QUATERNION = 'quaternion-euler-heading'
VMU931_MODE_GYROSCOPE = 'gyro-accel-mag'
VMU931_MODE_CUSTOM = 'custom'
VMU931_CALIBRATION_DURATION = float(5.0)


# Class Template of Robotnik component for Pyhton
class Vmu931Node(Node):
    
    def __init__(self):
        
        super().__init__('vmu931_imu')

        self.node_name = self.get_name() #.replace('/','')

        args = self.__initParameters()

        self.desired_freq = args['desired_freq'] 
        # Checks value of freq
        if self.desired_freq <= 0.0 or self.desired_freq > MAX_FREQ:
            self.get_logger().info('%s::init: Desired freq (%f) is not possible. Setting desired_freq to %f'%(self.node_name,self.desired_freq, DEFAULT_FREQ))
            self.desired_freq = DEFAULT_FREQ
   
    
        self.real_freq = 0.0
        # configuration driver to imu
        self._port = args['port']
        self._frame_id = args['frame_id']
        self._mode = args['mode']
        if self._mode == VMU931_MODE_QUATERNION:
            self._stream_gyro = False
            self._stream_magnetometer = False
            self._stream_accelerometer = False
            self._stream_quaternion = True
            self._stream_euler = True
            self._stream_heading = True
        elif self._mode == VMU931_MODE_GYROSCOPE:
            self._stream_gyro = True
            self._stream_magnetometer = True
            self._stream_accelerometer = True
            self._stream_quaternion = False
            self._stream_euler = False
            self._stream_heading = False
        else:
            if self._mode != VMU931_MODE_CUSTOM:
                self.get_logger().warn('{0}::init: unknown mode {1}. Setting {2} mode'.format(self.node_name, self._mode, VMU931_MODE_CUSTOM))
                self._mode = VMU931_MODE_CUSTOM
            self._stream_gyro = args['gyroscope']
            self._stream_magnetometer = args['magnetometer']
            self._stream_accelerometer = args['accelerometer']
            self._stream_quaternion = args['quaternion']
            self._stream_euler = args['euler']
            self._stream_heading = args['heading']
        
        # Saves the state of the component
        self.state = State.INIT_STATE
        # Saves the previous state
        self.previous_state = State.INIT_STATE
        # flag to control the initialization of the component
        self.initialized = False
        # flag to control the initialization of ROS stuff
        self.ros_initialized = False
        # flag to control that the control loop is running
        self.running = False
        # Variable used to control the loop frequency
        self.time_sleep = 1.0 
        # State msg to publish
        self.msg_state = VmuState()
        self.msg_state.mode = self._mode
        # Timer to publish state
        self.publish_state_timer = 1
        # Flag active to calibrate the sensor
        self._calibration = False
        self._calibration_step = 0
        
        self._emergency_recovery_timer = self.get_clock().now()
        
        self._imu_dev = vmu.vmu931(self._port, baudrate = 9600)
        
#        self.t_publish_state = threading.Timer(self.publish_state_timer, self.publishROSstate)
        self._imu_msg = Imu()
        self._imu_msg.header.frame_id = self._frame_id
        self._imu_msg.orientation_covariance = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self._imu_msg.angular_velocity_covariance = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self._imu_msg.linear_acceleration_covariance = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        self._gyro_msg = Vector3Stamped()
        self._gyro_msg.header.frame_id = self._frame_id
        
        self._euler_msg = Vector3Stamped()
        self._euler_msg.header.frame_id = self._frame_id
        
        self._accel_msg = Vector3Stamped()
        self._accel_msg.header.frame_id = self._frame_id
        
        self._magnet_msg = Vector3Stamped()
        self._magnet_msg.header.frame_id = self._frame_id
        
        self._quaternion_msg = QuaternionStamped()
        self._quaternion_msg.header.frame_id = self._frame_id
        
        self._heading_msg = Vector3Stamped()
        self._heading_msg.vector.x = self._heading_msg.vector.y = 0.0
        self._heading_msg.header.frame_id = self._frame_id

    def setup(self):
        '''
            Initializes de hand
            @return: True if OK, False otherwise
        '''        
        
        if not self._imu_dev.setup():
            return -1
        
        msg = ''
        trials = 0
        max_trials = 1000
        
        self.get_logger().info('{0}::setup: reading sensor status'.format(self.node_name))
        while msg!=vmu.STATUS and trials<=max_trials:
            print('trial %d'%trials)
            self._imu_dev.readStatus()
            #rospy.sleep(0.1)
            ret,msg = self._imu_dev.readOneTime()
            trials+=1
        print("done")
            
        if msg != vmu.STATUS:
            self.get_logger().err('{0}::setup: error reading sensor status'.format(self.node_name))
            return -1
        
        self.get_logger().info("IMU Dev Status")
        self._imu_dev.printStatus()
        
        self.get_logger().info('{0}::setup: configuring sensor streaming'.format(self.node_name))
        
        self._imu_dev.streamingAccelerometers(self._stream_accelerometer)
        self._imu_dev.streamingGyroscopes(self._stream_gyro)
        self._imu_dev.streamingQuaternions(self._stream_quaternion)
        self._imu_dev.streamingHeading(self._stream_heading)
        self._imu_dev.streamingEulerAngles(self._stream_euler)
        self._imu_dev.streamingMagnetometers(self._stream_magnetometer)
        
        msg = ''
        trials = 0
        self.get_logger().info('{0}::setup: reading sensor status to confirm streaming'.format(self.node_name))
        while msg!=vmu.STATUS and trials<=max_trials:
            self._imu_dev.readStatus()
            #rospy.sleep(0.1)
            ret,msg = self._imu_dev.readOneTime()
            trials+=1
        
        #self._imu_dev.printStatus()
        
        self.desired_freq = self._imu_dev.status.ouptputRate
        self.time_sleep = 1.0 / self.desired_freq

        self.get_logger().info("Desired freq: {0:.3f}".format(1.0/self.time_sleep))
            
        self.initialized = True
        
        return 0
        
        
    def rosSetup(self):
        '''
            Creates and inits ROS components
        '''
        if self.ros_initialized:
            return 0
        
        # Publishers
#        self._state_publisher = self.create_publisher(VmuState, "~state", 10)
        self._imu_publisher = self.create_publisher(Imu, 'data_raw', 10)
        self._gyro_publisher = self.create_publisher(Vector3Stamped, 'gyro', 10)
        self._euler_publisher = self.create_publisher(Vector3Stamped, 'euler', 10)
        self._accel_publisher = self.create_publisher(Vector3Stamped, 'accelerometer', 10)
        self._magnet_publisher = self.create_publisher(Vector3Stamped, 'magnetometer', 10)
        self._quaternion_publisher = self.create_publisher(QuaternionStamped, 'quaternion', 10)
        self._heading_publisher = self.create_publisher(Vector3Stamped, 'heading', 10)

        # Service Servers
        self.service_server = self.create_service(Trigger, 'calibrate', self.calibrationServiceCb)
        
        # Subscribers
        # topic_name, msg type, callback, queue_size
        # self.topic_sub = rospy.Subscriber('topic_name', Int32, self.topicCb, queue_size = 10)
        
        # Service Clients
        # self.service_client = rospy.ServiceProxy('service_name', ServiceMsg)
        # ret = self.service_client.call(ServiceMsg)
        
        self.ros_initialized = True
        
#        self.publishROSstate()
        
        return 0
        
        
    def shutdown(self):
        '''
            Shutdowns device
            @return: 0 if it's performed successfully, -1 if there's any problem or the component is running
        '''
        if self.running or not self.initialized:
            return -1
        self.get_logger().info('%s::shutdown'%self.node_name)
        
        # Cancels current timers
        self.t_publish_state.cancel()
        
        self._state_publisher.unregister()
        
        self.initialized = False
        
        return 0
    
    
    def rosShutdown(self):
        '''
            Shutdows all ROS components
            @return: 0 if it's performed successfully, -1 if there's any problem or the component is running
        '''
        if self.running or not self.ros_initialized:
            return -1
        
        # Performs ROS topics & services shutdown
        self._state_publisher.unregister()
        
        self.ros_initialized = False
        
        return 0
            
    
    def stop(self):
        '''
            Creates and inits ROS components
        '''
        self.running = False
        
        return 0
    
    
    def start(self):
        '''
            Runs ROS configuration and the main control loop
            @return: 0 if OK
        '''
        self.get_logger().info('%s: starting'%(self.get_name()))

        self.rosSetup()
        
        if self.running:
            return 0
            
        self.running = True

        self.__controlLoop = self.create_timer(0.1,self.controlLoop)
        
#        self.controlLoop()
        
        return 0
    
    def dummyControlLoop(self):

        self.__controlLoop.cancel()

        while self.running:

            if self.state == State.INIT_STATE:
                self.initState()
            else:
                self._quaternion_publisher.publish(QuaternionStamped())

            time.sleep(0.1)
            
    
    def controlLoop(self):
        '''
            Main loop of the component
            Manages actions by state
        '''

        self.__controlLoop.cancel()
        
        while self.running:
            t1 = time.time()
            
            if self.state == State.INIT_STATE:
                self.initState()
                
            elif self.state == State.STANDBY_STATE:
                self.standbyState()
                
            elif self.state == State.READY_STATE:
                self.readyState()
                
            elif self.state == State.EMERGENCY_STATE:
                self.emergencyState()
                
            elif self.state == State.FAILURE_STATE:
                self.failureState()
                
            elif self.state == State.SHUTDOWN_STATE:
                self.shutdownState()
                
            self.allState()
            
            t2 = time.time()
            tdiff = (t2 - t1)
            
            
            t_sleep = self.time_sleep - tdiff
            
            if t_sleep > 0.0:
                time.sleep(t_sleep)
            
            t3= time.time()
            self.real_freq = 1.0/(t3 - t1)

        self.running = False
        # Performs component shutdown
        self.shutdownState()
        # Performs ROS shutdown
        self.rosShutdown()
        self.get_logger().nfo('%s::controlLoop: exit control loop'%self.node_name)
        
        return 0
        
        
    def rosPublish(self):
        '''
            Publish topics at standard frequency
        '''
                    
        return 0
        
    
    def initState(self):
        '''
            Actions performed in init state
        '''
        
        if not self.initialized:
            self.setup()
            
        else:         
            self.switchToState(State.STANDBY_STATE)
    
    def standbyState(self):
        '''
            Actions performed in standby state
        '''
        if self._calibration:
            if self._calibration_step == 0:
                self.get_logger().info('%s::standbyState: calibrating'%self.node_name)
                self._imu_dev.calibrate()
                self._calibration_step+=1
                self._expected_calibration_response = self.get_clock().now() + rclpy.duration.Duration(VMU931_CALIBRATION_DURATION)

                '''
                cont = 0
                while (not self.value["Text"].update):
                        self.readOneTime()
                        print cont
                        cont = cont+1
                    print self.value["Text"].msg
                    self.value["Text"].notUpdate()
                    while (not self.value["Text"].update):
                        self.readOneTime()
                        cont = cont +1
                        print cont
                    print self.value["Text"].msg

                '''
            elif self._calibration_step == 1:
                ret,msg = self._imu_dev.readOneTime()            
                if self._imu_dev.value["Text"].update:
                    if self._imu_dev.value["Text"].msg.find('started') > 0:
                        self._imu_dev.value["Text"].notUpdate()
                        self._calibration_step += 1
                    elif self._imu_dev.value["Text"].msg.find('completed') > 0:
                        self._calibration = False
                        self.switchToState(State.READY_STATE)
                    else:
                        self._calibration_step = 0
                elif self.get_clock().now() > self._expected_calibration_response:
                        self._calibration_step = 0
                        
            elif self._calibration_step == 2:
                ret,msg = self._imu_dev.readOneTime()            
                if self._imu_dev.value["Text"].update:
                    if self._imu_dev.value["Text"].msg.find('completed') > 0:
                        self._calibration = False
                        self.switchToState(State.READY_STATE)
                    else:
                        self._calibration_step = 0
                elif self.get_clock().now() > self._expected_calibration_response:
                        self._calibration_step = 0
        else:
            self.switchToState(State.READY_STATE)
        
        return
    
    
    def readyState(self):
        '''
            Actions performed in ready state
        '''
        if self._calibration:
            self.switchToState(State.STANDBY_STATE)
            return
        
        ret, msg = self._imu_dev.readOneTime()
        
        if ret == 0:
            gy = self._imu_dev.value["Gyroscopes"]
            acc = self._imu_dev.value["Accelerometers"]
            quat = self._imu_dev.value["Quaternions"]
            euler = self._imu_dev.value["Euler"]
            mag = self._imu_dev.value["Magnetometers"]
            head = self._imu_dev.value["Heading"]
            
            current_time = Time.to_msg(self.get_clock().now())
            
            if msg == vmu.GYROSCOPES:
                self._gyro_msg.header.stamp = current_time
                self._gyro_msg.vector.x = math.radians(gy.x)
                self._gyro_msg.vector.y = math.radians(gy.y)
                self._gyro_msg.vector.z = math.radians(gy.z)
                self._gyro_publisher.publish(self._gyro_msg)
            
            if msg == vmu.EULER_ANGLES:
                self._euler_msg.header.stamp = current_time
                self._euler_msg.vector.x = math.radians(euler.x)
                self._euler_msg.vector.y = math.radians(euler.y)
                self._euler_msg.vector.z = math.radians(euler.z)
                self._euler_publisher.publish(self._euler_msg)
                    
            if msg == vmu.ACCELEROMETERS:
                self._accel_msg.header.stamp = current_time
                self._accel_msg.vector.x = acc.x*9.81
                self._accel_msg.vector.y = acc.y*9.81
                self._accel_msg.vector.z = acc.z*9.81
                self._accel_publisher.publish(self._accel_msg)
                    
            if msg == vmu.MAGNETOMETERS:
                self._magnet_msg.header.stamp = current_time
                self._magnet_msg.vector.x = math.radians(mag.x)
                self._magnet_msg.vector.y = math.radians(mag.y)
                self._magnet_msg.vector.z = math.radians(mag.z)
                self._magnet_publisher.publish(self._magnet_msg)
                
            if msg == vmu.HEADING:
                self._heading_msg.header.stamp = current_time
                self._heading_msg.vector.z = math.radians(head.heading)
                self._heading_publisher.publish(self._heading_msg)
                    
            if msg == vmu.QUATERNIONS:
                self._quaternion_msg.header.stamp = current_time
                self._quaternion_msg.quaternion.x = quat.x
                self._quaternion_msg.quaternion.y = quat.y
                self._quaternion_msg.quaternion.z = quat.z
                self._quaternion_msg.quaternion.w = quat.w
                self._quaternion_publisher.publish(self._quaternion_msg)
            
                self._imu_msg.header.stamp = current_time
                #print dir(self._imu_msg.angular_velocity) 
                self._imu_msg.angular_velocity.x = 0.0
                self._imu_msg.angular_velocity.y = 0.0
                self._imu_msg.angular_velocity.z = 0.0
                
                self._imu_msg.linear_acceleration.x = 0.0
                self._imu_msg.linear_acceleration.y = 0.0
                self._imu_msg.linear_acceleration.z = 0.0
                
                self._imu_msg.orientation.x = quat.x
                self._imu_msg.orientation.y = quat.y
                self._imu_msg.orientation.z = quat.z
                self._imu_msg.orientation.w = quat.w
                
                self._imu_publisher.publish(self._imu_msg)
        elif ret == -5:
            self.switchToState(State.EMERGENCY_STATE)
        
        return
        
    
    def shutdownState(self):
        '''
            Actions performed in shutdown state 
        '''
        if self.shutdown() == 0:
            self.switchToState(State.INIT_STATE)
        
        return
    
    
    def emergencyState(self):
        '''
            Actions performed in emergency state
        '''
        if (self.get_clock().now() - self._emergency_recovery_timer).to_sec() >= 5.0:
            self.get_logger().warn('{0}::emergencyState: closing and reopenning device'.format(self.node_name))
            self._imu_dev.close()
            
            if self._imu_dev.setup():
                self.initialized = False
                self.switchToState(State.INIT_STATE)
            else:
                self._emergency_recovery_timer = self.get_clock().now()

        return
    
    
    def failureState(self):
        '''
            Actions performed in failure state
        '''
        
            
        return
    
    
    def switchToState(self, new_state):
        '''
            Performs the change of state
        '''
        if self.state != new_state:
            self.previous_state = self.state
            self.state = new_state
            if new_state == State.READY_STATE or  new_state == State.STANDBY_STATE:
                self.time_sleep = 1.0 / self.desired_freq
            else:
                self.time_sleep = 1.0
            self.get_logger().info('%s::switchToState: %s'%(self.node_name, self.stateToString(self.state)))
        
        return
    
        
    def allState(self):
        '''
            Actions performed in all states
        '''
        self.rosPublish()
        
        return
    
    
    def stateToString(self, state):
        '''
            @param state: state to set
            @type state: State
            @returns the equivalent string of the state
        '''
        if state == State.INIT_STATE:
            return 'INIT_STATE'
                
        elif state == State.STANDBY_STATE:
            return 'STANDBY_STATE'
            
        elif state == State.READY_STATE:
            return 'READY_STATE'
            
        elif state == State.EMERGENCY_STATE:
            return 'EMERGENCY_STATE'
            
        elif state == State.FAILURE_STATE:
            return 'FAILURE_STATE'
            
        elif state == State.SHUTDOWN_STATE:
            return 'SHUTDOWN_STATE'
        else:
            return 'UNKNOWN_STATE'
    
        
    def publishROSstate(self):
        '''
            Publish the State of the component at the desired frequency
        '''
        self.msg_state.state.state = self.state
        self.msg_state.state.state_description = self.stateToString(self.state)
        print("Desired freq is {0}".format(self.desired_freq))
        self.msg_state.state.desired_freq = float(self.desired_freq)
        self.msg_state.state.real_freq = float(self.real_freq)
        
        self.msg_state.gyroscope_status = self._imu_dev.status.gyroStatus
        self.msg_state.accelerometer_status = self._imu_dev.status.gyroStatus
        self.msg_state.magnetometer_status = self._imu_dev.status.magStatus
        self.msg_state.output_rate = self._imu_dev.status.ouptputRate
        self.msg_state.heading_streaming = self._imu_dev.status.streamingHead
        self.msg_state.euler_streaming = self._imu_dev.status.streamingEuler
        self.msg_state.gyro_streaming = self._imu_dev.status.streamingGyro
        self.msg_state.magnet_streaming = self._imu_dev.status.streamingMag
        self.msg_state.quaternion_streaming = self._imu_dev.status.streamingQuart
        self.msg_state.accel_streaming = self._imu_dev.status.streamingAcc
        self.msg_state.calibrating = self._calibration
                        
            # self._state_publisher.publish(self.msg_state)
            
        #    self.t_publish_state = threading.Timer(self.publish_state_timer, self.publishROSstate)
        #    self.t_publish_state.start()

    """
    def topicCb(self, msg):
        '''
            Callback for inelfe_video_manager state
            @param msg: received message
            @type msg: std_msgs/Int32
        '''
        # DEMO
        self.get_logger().info('Vmu931Node:topicCb')
   """    
    
    async def calibrationServiceCb(self, req, res):
        '''
            ROS service server
            @param req: Required action
            @type req: std_srv/Trigger
        '''
        # DEMO
        if self.state != State.READY_STATE:
            return False,'Service not available in %s'%self.stateToString(self.state)    
            
        if not self._calibration:
            self._calibration_step = 0
        
        self._calibration = True        
        self.get_logger().info('%s::calibrationServiceCb'%self.node_name)    

        res.success = True
        res.message = 'ok'
        
        return res

    def __initParameters(self) -> dict:

        arg_defaults = {
          'topic_state': 'state',
          'desired_freq': DEFAULT_FREQ,
          'port': '/dev/ttyACM0',
          'frame_id': 'imu_link',
          'mode': VMU931_MODE_QUATERNION,
          'gyroscope': False,
          'magnetometer': False,
          'accelerometer': False,
          'quaternion': True,
          'euler': True,
          'heading': True,
        }
        
        args = {}
        
        for name in arg_defaults:
            try:
                if self.has_parameter(name): 
                    args[name] = self.get_parameter('~%s'%(name)) # Adding the name of the node, because the para has the namespace of the node
                else:
                    args[name] = arg_defaults[name]
                #print name
            except Exception as e:
                rclpy.logging._root_logger.error('%s: %s'%(e, name))

        return args
        
def main(args=None):

    rclpy.init(args=args)
    
    rc_node = Vmu931Node()
    
    rc_node.start()

    rclpy.spin(rc_node)
    rc_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
