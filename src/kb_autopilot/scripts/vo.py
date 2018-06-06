import numpy as np 
import cv2


class VisualOdometery:
    def __init__(self, width, height, fx, fy, cx, cy):
        self.width = width
        self.height = height
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy
        self.camera_mat = np.array([[self.fx, 0.0, self.cx], [0.0, self.fy, self.cy], [0.0, 0.0, 1.0]])    
        self.focal = (fx, fy)
        self.pc = (cx, cy)
        self.min_features = 400

        self.lk_params = dict(winSize  = (21, 21), 
                        criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))

        self.traj = np.zeros((1200, 1600, 3), dtype=np.uint8)

        self.detector = cv2.FastFeatureDetector_create(threshold=25, nonmaxSuppression=True)

    def featureTracking(self, image_old, image_cur, p0):
        p1, st, err = cv2.calcOpticalFlowPyrLK(image_old, image_cur, p0, None, **self.lk_params)  #shape: [k,2] [k,1] [k,1]

        st = st.reshape(st.shape[0])
        kp0 = p0[st == 1]
        kp1 = p1[st == 1]
        
        u = (kp1 - kp0) * (1.0/30) * (1.0/50)
        vel = np.linalg.norm(u)
        return kp0, kp1, vel
    
    def get_pose(self, p0, p1):
        E, mask = cv2.findEssentialMat(p1, p0, cameraMatrix=self.camera_mat, method=cv2.RANSAC, prob=0.9, threshold=1.0)
        _, R, t, mask = cv2.recoverPose(E, p1, p0, cameraMatrix=self.camera_mat)
        return R, t
            

    def update_pose(self, R, t, R1, t1, vel):
        t = t + vel*R.dot(t1)
        R = R1.dot(R)
        return t, R


    def get_new_features(self, old_frame):
        # Get the features
        # old_frame = self.get_nextframe()
        p0 = self.detector.detect(old_frame)
        p0 = np.array([x.pt for x in p0], dtype=np.float32)
        return p0, old_frame
    
    def get_nextframe(self):
        # Return cv2 image object
        pass
    
    def draw(self, t, rp):
        x, y, z = t[0], t[1], t[2]
        draw_x, draw_y = int(x)+400, int(z)+400    		
        true_x, true_y = int(rp[0])+400, int(rp[2] + 400)
        #print ((draw_x, draw_y), (true_x, true_y))
        cv2.circle(self.traj, (draw_x, draw_y), 1, (255,255,255), 1)
        cv2.circle(self.traj, (true_x, true_y), 1, (0,255,0), 2)
        cv2.rectangle(self.traj, (10, 20), (600, 60), (0,0,0), -1)
        text = "Coordinates: x=%2fm y=%2fm z=%2fm"%(x,y,z)
        cv2.putText(self.traj, text, (20,40), cv2.FONT_HERSHEY_PLAIN, 1, (255,255,255), 1, 8)

        #cv2.imshow('Road facing camera', img)
        cv2.imshow('Trajectory', self.traj)

    def start_tracking(self, old_frame, new_frame):
        p0, old_frame = self.get_new_features(old_frame)
        # new_frame = self.get_nextframe()
        p0, p1, vel = self.featureTracking(old_frame, new_frame, p0)
        R, t = self.get_pose(p0, p1)
        old_frame = new_frame
        p0 = p1
        return old_frame, p0

"""
while True:
    new_frame = get_nextframe(srcobj)
    p0, p1, vel = featureTracking(old_frame, new_frame, p0)
    R1, t1 = get_pose(p0, p1)
    #tvel = get_true_velocity(srcobj)
    if vel > 0.1:
        t, R = update_pose(R, t, R1, t1, vel)
        
    if p0.shape[0] < min_features:
        p1, new_frame = get_new_features(srcobj)
        #continue
    p0 = p1
    old_frame = new_frame
    
    rp = get_true_position(srcobj)

    draw(t, rp)    
    if source == "IMAGES":
        srcobj += 1

    #print (srcobj)
    cv2.imshow('frame', old_frame)
    cv2.waitKey(10) 

"""