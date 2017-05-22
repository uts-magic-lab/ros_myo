#!/usr/bin/env python

from __future__ import print_function

import re
import sys
from math import sqrt, degrees
import serial
from serial.tools.list_ports import comports
from common import pack, unpack, multiord
from bluetooth import BT
import rospy
from tf.transformations import euler_from_quaternion
from std_msgs.msg import String, Header, UInt8
from geometry_msgs.msg import Quaternion, Vector3, PoseStamped, Point, Pose
from sensor_msgs.msg import Imu
from ros_myo.msg import MyoArm, MyoPose, EmgArray


class MyoRaw(object):
    '''Implements the Myo-specific communication protocol.'''

    def __init__(self, tty=None):
        if tty is None:
            tty = self.detect_tty()
        if tty is None:
            raise ValueError('Myo dongle not found!')

        self.bt = BT(tty)
        self.conn = None
        self.emg_handlers = []
        self.imu_handlers = []
        self.arm_handlers = []
        self.pose_handlers = []

    def detect_tty(self):
        for p in comports():
            if re.search(r'PID=2458:0*1', p[2]):
                print('using device:', p[0])
                return p[0]

        return None

    def run(self, timeout=None):
        self.bt.recv_packet(timeout)

    def connect(self):
        # stop everything from before
        self.bt.end_scan()
        self.bt.disconnect(0)
        self.bt.disconnect(1)
        self.bt.disconnect(2)

        # start scanning
        print('scanning...')
        self.bt.discover()
        while True:
            p = self.bt.recv_packet()
            print('scan response:', p)

            if p.payload.endswith(b'\x06\x42\x48\x12\x4A\x7F\x2C\x48\x47\xB9\xDE\x04\xA9\x01\x00\x06\xD5'):
                addr = list(multiord(p.payload[2:8]))
                break
        self.bt.end_scan()

        # connect and wait for status event
        conn_pkt = self.bt.connect(addr)
        self.conn = multiord(conn_pkt.payload)[-1]
        self.bt.wait_event(3, 0)

        # get firmware version
        fw = self.read_attr(0x17)
        _, _, _, _, v0, v1, v2, v3 = unpack('BHBBHHHH', fw.payload)
        print('firmware version: %d.%d.%d.%d' % (v0, v1, v2, v3))

        self.old = (v0 == 0)

        if self.old:
            # don't know what these do
            # Myo Connect sends them, though we get data
            # fine without them
            self.write_attr(0x19, b'\x01\x02\x00\x00')
            self.write_attr(0x2f, b'\x01\x00')
            self.write_attr(0x2c, b'\x01\x00')
            self.write_attr(0x32, b'\x01\x00')
            self.write_attr(0x35, b'\x01\x00')

            # enable EMG data
            self.write_attr(0x28, b'\x01\x00')
            # enable IMU data
            self.write_attr(0x1d, b'\x01\x00')

            # Sampling rate of the underlying EMG sensor, capped to 1000.
            # If it's less than 1000, emg_hz is correct. If it is greater,
            # the actual framerate starts dropping inversely. Also, if this
            # is much less than 1000, EMG data becomes slower to respond to
            # changes. In conclusion, 1000 is probably a good value.
            C = 1000
            emg_hz = 50
            # strength of low-pass filtering of EMG data
            emg_smooth = 100

            imu_hz = 50

            # send sensor parameters, or we don't get any data
            self.write_attr(0x19, pack('BBBBHBBBBB', 2, 9, 2, 1, C,
                                       emg_smooth, C // emg_hz, imu_hz, 0, 0))

        else:
            name = self.read_attr(0x03)
            print('device name: %s' % name.payload)

            # enable IMU data
            self.write_attr(0x1d, b'\x01\x00')
            # enable on/off arm notifications
            self.write_attr(0x24, b'\x02\x00')

            self.start_raw()

        # add data handlers
        def handle_data(p):
            if (p.cls, p.cmd) != (4, 5):
                return

            c, attr, typ = unpack('BHB', p.payload[:4])
            pay = p.payload[5:]

            if attr == 0x27:
                vals = unpack('8HB', pay)
                # not entirely sure what the last byte is,
                # but it's a bitmask that seems to indicate which
                # sensors think they're being moved around or something
                emg = vals[:8]
                moving = vals[8]
                self.on_emg(emg, moving)
            elif attr == 0x1c:
                vals = unpack('10h', pay)
                quat = vals[:4]
                acc = vals[4:7]
                gyro = vals[7:10]
                self.on_imu(quat, acc, gyro)
            elif attr == 0x23:
                if len(pay) == 6:
                    try:
                        typ, val, xdir, _, _, _ = unpack('6B', pay)
                    except Exception as e:
                        print("Got exception: " + str(e) + "\nContinuing...")
                        return
                elif len(pay) == 3:
                    try:
                        typ, val, xdir = unpack('3B', pay)
                    except Exception as e:
                        print("Got exception: " + str(e) + "\nContinuing...")
                        return

                if typ == 1:  # on arm
                    self.on_arm(MyoArm(arm=val, xdir=xdir))
                elif typ == 2:  # removed from arm
                    self.on_arm(MyoArm(MyoArm.UNKNOWN, MyoArm.UNKNOWN))
                elif typ == 3:  # pose
                    if val == 255:
                        pose = MyoPose(0)
                    else:
                        pose = MyoPose(val + 1)
                    self.on_pose(pose)
            else:
                print('data with unknown attr: %02X %s' % (attr, p))

        self.bt.add_handler(handle_data)

    def write_attr(self, attr, val):
        if self.conn is not None:
            self.bt.write_attr(self.conn, attr, val)

    def read_attr(self, attr):
        if self.conn is not None:
            return self.bt.read_attr(self.conn, attr)
        return None

    def disconnect(self):
        if self.conn is not None:
            self.bt.disconnect(self.conn)

    def start_raw(self):
        '''Sending this sequence for v1.0 firmware seems to enable both raw data and
        pose notifications.
        '''

        self.write_attr(0x28, b'\x01\x00')
        self.write_attr(0x19, b'\x01\x03\x01\x01\x01')

    def mc_start_collection(self):
        '''Myo Connect sends this sequence (or a reordering) when starting data
        collection for v1.0 firmware; this enables raw data but disables arm
        and pose notifications.'''

        self.write_attr(0x28, b'\x01\x00')
        self.write_attr(0x1d, b'\x01\x00')
        self.write_attr(0x24, b'\x02\x00')
        self.write_attr(0x19, b'\x01\x03\x01\x01\x01')
        self.write_attr(0x28, b'\x01\x00')
        self.write_attr(0x1d, b'\x01\x00')
        self.write_attr(0x19, b'\x09\x01\x01\x00\x00')
        self.write_attr(0x1d, b'\x01\x00')
        self.write_attr(0x19, b'\x01\x03\x00\x01\x00')
        self.write_attr(0x28, b'\x01\x00')
        self.write_attr(0x1d, b'\x01\x00')
        self.write_attr(0x19, b'\x01\x03\x01\x01\x00')

    def mc_end_collection(self):
        '''Myo Connect sends this sequence (or a reordering) when ending data
        collection for v1.0 firmware; this reenables arm and pose
        notifications, but doesn't disable raw data.'''

        self.write_attr(0x28, b'\x01\x00')
        self.write_attr(0x1d, b'\x01\x00')
        self.write_attr(0x24, b'\x02\x00')
        self.write_attr(0x19, b'\x01\x03\x01\x01\x01')
        self.write_attr(0x19, b'\x09\x01\x00\x00\x00')
        self.write_attr(0x1d, b'\x01\x00')
        self.write_attr(0x24, b'\x02\x00')
        self.write_attr(0x19, b'\x01\x03\x00\x01\x01')
        self.write_attr(0x28, b'\x01\x00')
        self.write_attr(0x1d, b'\x01\x00')
        self.write_attr(0x24, b'\x02\x00')
        self.write_attr(0x19, b'\x01\x03\x01\x01\x01')

    def vibrate(self, length):
        if length in xrange(1, 4):
            # first byte tells it to vibrate; purpose of second byte is unknown
            self.write_attr(0x19, pack('3B', 3, 1, length))

    def add_emg_handler(self, h):
        self.emg_handlers.append(h)

    def add_imu_handler(self, h):
        self.imu_handlers.append(h)

    def add_pose_handler(self, h):
        self.pose_handlers.append(h)

    def add_arm_handler(self, h):
        self.arm_handlers.append(h)

    def on_emg(self, emg, moving):
        for h in self.emg_handlers:
            h(emg, moving)

    def on_imu(self, quat, acc, gyro):
        for h in self.imu_handlers:
            h(quat, acc, gyro)

    def on_pose(self, p):
        for h in self.pose_handlers:
            h(p)

    def on_arm(self, myoarm_msg):
        for h in self.arm_handlers:
            h(myoarm_msg)


global vibrate_order
vibrate_order = 0

if __name__ == '__main__':
    # Start by initializing the Myo and attempting to connect.
    # If no Myo is found, we attempt to reconnect every 0.5 seconds
    connected = 0
    print("Initializing...")
    while(connected == 0):
        try:
            m = MyoRaw(sys.argv[1] if len(sys.argv) >= 2 else None)
            connected = 1
        except (ValueError, KeyboardInterrupt) as e:
            print("Myo Armband not found. Attempting to connect...")
            rospy.sleep(0.5)
            pass

    rospy.init_node('myo_raw')

    # Define Publishers
    imuPub = rospy.Publisher('~myo_imu', Imu, queue_size=1)
    oriPub = rospy.Publisher('~myo_ori', Vector3, queue_size=1)
    oriDegPub = rospy.Publisher('~myo_ori_deg', Vector3, queue_size=1)
    emgPub = rospy.Publisher('~myo_emg', EmgArray, queue_size=1)
    armPub = rospy.Publisher('~myo_arm', MyoArm, queue_size=1, latch=True)
    gestPub = rospy.Publisher('~myo_gest', MyoPose, queue_size=1)
    gestStrPub = rospy.Publisher('~myo_gest_str', String, queue_size=1)
    posePub = rospy.Publisher('~pose', PoseStamped, queue_size=1)

    # Package the EMG data into an EmgArray
    def proc_emg(emg, moving):
        # create an array of ints for emg data
        # and moving data
        emgPub.publish(EmgArray(emg, moving))

    # Package the IMU data into an Imu message
    def proc_imu(quat1, acc, gyro):
        # New info:
        # https://github.com/thalmiclabs/myo-bluetooth/blob/master/myohw.h#L292-L295
        # Scale values for unpacking IMU data
        # define MYOHW_ORIENTATION_SCALE   16384.0f
        # See myohw_imu_data_t::orientation
        # define MYOHW_ACCELEROMETER_SCALE 2048.0f
        # See myohw_imu_data_t::accelerometer
        # define MYOHW_GYROSCOPE_SCALE     16.0f
        # See myohw_imu_data_t::gyroscope
        if not any(quat1):
            # If it's all 0's means we got no data, don't do anything
            return
        h = Header()
        h.stamp = rospy.Time.now()
        # frame_id is node name without /
        h.frame_id = rospy.get_name()[1:]
        # We currently don't know the covariance of the sensors with each other
        cov = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        quat = Quaternion(quat1[0] / 16384.0,
                          quat1[1] / 16384.0,
                          quat1[2] / 16384.0,
                          quat1[3] / 16384.0)
        # Normalize the quaternion and accelerometer values
        quatNorm = sqrt(quat.x * quat.x + quat.y *
                        quat.y + quat.z * quat.z + quat.w * quat.w)
        normQuat = Quaternion(quat.x / quatNorm,
                              quat.y / quatNorm,
                              quat.z / quatNorm,
                              quat.w / quatNorm)
        normAcc = Vector3(acc[0] / 2048.0,
                          acc[1] / 2048.0,
                          acc[2] / 2048.0)
        normGyro = Vector3(gyro[0] / 16.0, gyro[1] / 16.0, gyro[2] / 16.0)
        imu = Imu(h, normQuat, cov, normGyro, cov, normAcc, cov)
        imuPub.publish(imu)
        roll, pitch, yaw = euler_from_quaternion([normQuat.x,
                                                  normQuat.y,
                                                  normQuat.z,
                                                  normQuat.w])
        oriPub.publish(Vector3(roll, pitch, yaw))
        oriDegPub.publish(Vector3(degrees(roll), degrees(pitch), degrees(yaw)))
        posePub.publish( PoseStamped(h,Pose(Point(0.0,0.0,0.0),normQuat)) )

    # Package the arm and x-axis direction into an Arm message
    def proc_arm(myoarm_msg):
        # When the arm state changes, publish the new arm and orientation
        calibArm = myoarm_msg
        armPub.publish(calibArm)

    # Publish the value of an enumerated gesture
    def proc_pose(myo_pose):
        gestPub.publish(myo_pose)
        p = myo_pose.pose
        if p == 0:
            s = "UNKNOWN"
        elif p == 1:
            s = "REST"
        elif p == 2:
            s = "FIST"
        elif p == 3:
            s = "WAVE_IN"
        elif p == 4:
            s = "WAVE_OUT"
        elif p == 5:
            s = "FINGERS_SPREAD"
        elif p == 6:
            s = "THUMB_TO_PINKY"
        else:
            s = "ERROR"
        gestStrPub.publish(s)

    m.add_emg_handler(proc_emg)
    m.add_imu_handler(proc_imu)
    m.add_arm_handler(proc_arm)
    m.add_pose_handler(proc_pose)

    m.connect()

    global vibrate_order
    vibrate_order = 0

    def vibrate_cb(data):
        if data.data in range(1, 4):
            print("Vibrating with intensity (1, 2, 3): " + str(data.data))
            global vibrate_order
            vibrate_order = data.data

    # Add a way to make the controller vibrate
    vibrateSub = rospy.Subscriber('~vibrate', UInt8, vibrate_cb, queue_size=1)

    try:
        while not rospy.is_shutdown():
            m.run(1)
            if vibrate_order in range(1, 4):
                m.vibrate(vibrate_order)
                vibrate_order = 0

    except (rospy.ROSInterruptException, serial.serialutil.SerialException) as e:
        print("Exception: " + str(e))
    finally:
        print()
        print("Disconnecting...")
        m.disconnect()
        print()
