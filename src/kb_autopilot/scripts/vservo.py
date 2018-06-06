import cv2
import rospy
from sensor_msgs.msg import CompressedImage
from kb_autopilot.msg import Vo, Encoder
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from vo import VisualOdometery

class Vservo:
    def __init__(self):
        self.bridge = CvBridge()
        # Subscribing to the ros bag compressed image
        self.image_sub = rospy.Subscriber('/camera/color/image_raw/compressed', CompressedImage, self.callback)
        self.encoder_sub = rospy.Subscriber('/encoder', Encoder, self.get_vel)
        self.vservo_pub = rospy.Publisher('visual_servo', Vo, queue_size=10)
        self.first_time = True
        self.old_frame = None
        self.new_frame = None
        self.p0 = None
        self.R = np.array([[1,0,0],[0,1,0], [0,0,1]])
        self.t = np.array([[0],[0],[0]])
        self.R1 = self.R
        self.t1 = self.t
        self.yaw_angle = 0
        self.real_velocity = 0

    def get_vel(self, encmsg):
        self.real_velocity = encmsg.vel

    def callback(self, msg):
        np_arr = np.fromstring(msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        (rows,cols,channels) = cv_image.shape
        try:
            # Vo odometry overhere
            #width:640, height:480
            # fx,fy: 602.595726, 603.3249
            # cx,cy: 313.324336, 248.223737

            vo = VisualOdometery(640, 480, 602.595726, 603.3249,313.324336, 248.223737)
            self.new_frame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            if self.first_time:
                self.p0, self.old_frame = vo.get_new_features(self.new_frame)
                self.first_time = False
            else:
                #vo.start_tracking(old_frame, img_gray)
                p0, p1, vel = vo.featureTracking(self.old_frame, self.new_frame, self.p0)
                #print ('velocity',vel)
                if p0.shape[0] < vo.min_features:
                    p1, new_frame = vo.get_new_features(self.new_frame)
                else:
                    self.R1, self.t1 = vo.get_pose(p0, p1)
                if self.real_velocity > 0.1:
                    # Get original velocity from inboard sensor
                    self.t, self.R = vo.update_pose(self.R, self.t, self.R1, self.t1, self.real_velocity/30)
                self.old_frame = self.new_frame
                self.p0 = p1

            self.yaw_angle = np.arctan2(self.R[2][1], self.R[2][2])

            #print ('RT', self.R.shape, self.t.shape, self.R[2][1],self.R[2][2])
            # print (self.t)
            pub_msg = Vo()
            pub_msg.header.stamp = rospy.Time.now()
            pub_msg.angle = self.yaw_angle
            pub_msg.x = self.t[0][0]
            pub_msg.y = self.t[1][0]
            pub_msg.z = self.t[2][0]
            self.vservo_pub.publish(pub_msg)
            
            cv2.imshow('cv_img', self.new_frame)
            cv2.waitKey(2)

        except CvBridgeError as e:
            print(e)


if __name__ == '__main__':
    # Initialize Node
    rospy.init_node('Vservo')

    # Vservo object
    convert = Vservo()

    while not rospy.is_shutdown():
        rospy.spin()