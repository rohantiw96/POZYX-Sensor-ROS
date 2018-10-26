#!/usr/bin/env python

import rospy
from pypozyx import POZYX_POS_ALG_UWB_ONLY, POZYX_3D, POZYX_SUCCESS, Quaternion, Coordinates, PositionError, SingleRegister, DeviceList, DeviceCoordinates, PozyxSerial, AngularVelocity, LinearAcceleration
from std_msgs.msg import Header
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Point
from geometry_msgs.msg import Quaternion as Quat
from sensor_msgs.msg import Imu

class Localize(object):
    def __init__(self, pozyx, anchors, world_frame_id='world', tag_frame_id='pozyx', algorithm=POZYX_POS_ALG_UWB_ONLY, dimension=POZYX_3D, height=1000, remote_id=None):
        self.pozyx = pozyx
        self.anchors = anchors
        self.algorithm = algorithm
        self.dimension = dimension
        self.height = height
        self.remote_id = remote_id

        self.world_frame_id = world_frame_id
        self.tag_frame_id = tag_frame_id

    def setup(self):
        self.pozyx.printDeviceInfo(self.remote_id)
        self.pozyx.clearDevices(self.remote_id)
        self.setAnchorsManual()
        self.printPublishConfigurationResult()

    def loop(self):
        # pose stamped
        position = Coordinates()
        # quaternion = Quaternion()
        cov = PositionError()

        status = self.pozyx.doPositioning(position, self.dimension, self.height, self.algorithm)
        # status &= self.pozyx.getQuaternion(quaternion)
        status_cov = self.pozyx.getPositionError(cov)

        if status_cov == POZYX_SUCCESS:
            rospy.loginfo("Got Covariance %.4f " % (cov))

        if status == POZYX_SUCCESS:
            point = Point(position.x * 0.001, position.y * 0.001, position.z * 0.001)
            # quat = Quat(quaternion.x, quaternion.y, quaternion.z, quaternion.w)

            poseWithCovarianceStamped = PoseWithCovarianceStamped()

            poseWithCovarianceStamped.header.stamp = rospy.get_rostime()
            poseWithCovarianceStamped.header.frame_id = self.world_frame_id

            poseWithCovarianceStamped.pose.pose.position = point
            # poseWithCovarianceStamped.pose.pose.orientation = quat

            poseWithCovarianceStamped.pose.covariance = [cov.x, cov.xy, cov.xz, 0, 0, 0]
            poseWithCovarianceStamped.pose.covariance += [cov.xy, cov.y, cov.yz, 0, 0, 0]
            poseWithCovarianceStamped.pose.covariance += [cov.xz, cov.yz, cov.z, 0, 0, 0]
            poseWithCovarianceStamped.pose.covariance += [0, 0, 0, 0, 0, 0]
            poseWithCovarianceStamped.pose.covariance += [0, 0, 0, 0, 0, 0]
            poseWithCovarianceStamped.pose.covariance += [0, 0, 0, 0, 0, 0]

            pose = PoseStamped()

            pose.header.stamp = poseWithCovarianceStamped.header.stamp
            pose.header.frame_id = self.world_frame_id

            pose.pose.position = point
            # pose.pose.orientation = quat

            rospy.loginfo("x(m): %.4f y(m): %.4f z(m): %.4f" % (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z))

            pose_with_cov_publisher.publish(poseWithCovarianceStamped)
            pose_publisher.publish(pose)
        else:
            self.printPublishErrorCode("positioning")

    def setAnchorsManual(self):
        status = self.pozyx.clearDevices(self.remote_id)
        for anchor in self.anchors:
            status &= self.pozyx.addDevice(anchor, self.remote_id)
        if len(self.anchors) > 4:
            status &= self.pozyx.setSelectionOfAnchors(POZYX_ANCHOR_SEL_AUTO, len(self.anchors))
        return status

    def printPublishConfigurationResult(self):
        list_size = SingleRegister()

        status = self.pozyx.getDeviceListSize(list_size, self.remote_id)
        rospy.loginfo("List size: {0}".format(list_size[0]))
        if list_size[0] != len(self.anchors):
            self.printPublishErrorCode("configuration")
            return
        device_list = DeviceList(list_size=list_size[0])
        status = self.pozyx.getDeviceIds(device_list, self.remote_id)
        rospy.loginfo("Calibration result: ")
        rospy.loginfo("Anchors found: {0}".format(list_size[0]))
        
    def printPublishAnchorConfiguration(self):
        for anchor in self.anchors:
            rospy.loginfo("ANCHOR,0x%0.4x,%s" % (anchor.network_id, str(anchor.coordinates)))

    def printPublishErrorCode(self, operation):
        error_code = SingleRegister()
        network_id = self.remote_id
        if network_id is None:
            self.pozyx.getErrorCode(error_code)
            rospy.logerror("LOCAL ERROR %s, %s" % (operation, self.pozyx.getErrorMessage(error_code)))
            return
        status = self.pozyx.getErrorCode(error_code, self.remote_id)
        if status == POZYX_SUCCESS:
            rospy.logerror("ERROR %s on ID %s, %s" %
                  (operation, "0x%0.4x" % network_id, self.pozyx.getErrorMessage(error_code)))
        else:
            self.pozyx.getErrorCode(error_code)
            rospy.logerror("ERROR %s, couldn't retrieve remote error code, LOCAL ERROR %s" %
                  (operation, self.pozyx.getErrorMessage(error_code)))

if __name__=="__main__":
    serial_port = get_first_pozyx_serial_port()
    if serial_port is None:
        print("No Pozyx connected. Check your USB cable or your driver!")
        quit()

    rospy.init_node('pozyx_node')
    f = KalmanFilter (dim_x=4, dim_z=2)
    delta = 0.001
    f.F = np.matrix([[ 1.,0.,1.,0.],[ 0.,1.,0.,1.],[ 0.,0.,1.,0.],[ 0.,0.,0.,1.]])
    f.x = np.array([[0.],[0.],[0.],[0.]])
    f.H = np.matrix([[1,0,0,0],[0,1,0,0]])
    # f.P = np.matrix([ [5, 0, 0, 0],[0, 10, 0, 0],[0, 0, 5, 0],[0, 0, 0, 10]])
    f.P *= 1000.
    # f.R=np.matrix([[10,0],[0,10]])
    f.R = 10
    f.Q=np.matrix([[0.013, 0.025, 0, 0],
     [0.025, 0.05, 0, 0],
     [0, 0, 0.013, 0.025],
     [0, 0, 0.025, 0.05]])

    anchor_ids = []
    anchor_ids.append(int(rospy.get_param('~anchor0_id', '0x6931'), 16))
    anchor_ids.append(int(rospy.get_param('~anchor1_id', '0x6E4A'), 16))
    anchor_ids.append(int(rospy.get_param('~anchor2_id', '0x693F'), 16))
    anchor_ids.append(int(rospy.get_param('~anchor3_id', '0x6E15'), 16))

    anchor_coordinates = []
    anchor_coordinates.append(eval(rospy.get_param('~anchor0_coordinates', '0, 0, 1700')))
    anchor_coordinates.append(eval(rospy.get_param('~anchor1_coordinates', '2300, 0, 1700')))
    anchor_coordinates.append(eval(rospy.get_param('~anchor2_coordinates', '0, 4930, 1700')))
    anchor_coordinates.append(eval(rospy.get_param('~anchor3_coordinates', '2300, 4930, 1700')))

    anchors = [DeviceCoordinates(anchor_ids[i], 1, Coordinates(anchor_coordinates[i][0], anchor_coordinates[i][1], anchor_coordinates[i][2])) for i in range(4)]

    algorithm = int(rospy.get_param('~algorithm', POZYX_POS_ALG_UWB_ONLY))
    dimension = int(rospy.get_param('~dimension', POZYX_3D))
    height    = int(rospy.get_param('~height', '1000'))
    frequency = int(rospy.get_param('~frequency', '10'))

    world_frame_id = str(rospy.get_param('~world_frame_id', 'world'))
    tag_frame_id = str(rospy.get_param('~tag_frame_id', 'pozyx'))

    # Creating publishers
    pose_with_cov_publisher = rospy.Publisher('~pose_with_cov', PoseWithCovarianceStamped, queue_size=1)
    pose_publisher = rospy.Publisher('~pose', PoseStamped , queue_size=1)
    # imu_publisher = rospy.Publisher('~imu', Imu, queue_size=1)
    
    rate = rospy.Rate(frequency)

    pozyx = PozyxSerial(serial_port)
    r = Localize(pozyx, anchors, world_frame_id, tag_frame_id, algorithm, dimension, height)
    r.setup()
    while not rospy.is_shutdown():
        r.loop()
        rate.sleep()
