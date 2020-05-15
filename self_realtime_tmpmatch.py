import cv2
import rospy
from geometry_msgs.msg import PoseArray, Pose

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 504)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1116)
template = cv2.imread("/home/cjj/H_shape.png", cv2.IMREAD_GRAYSCALE)
w, h = template.shape[::-1]

pub = rospy.Publisher('vertices', PoseArray, queue_size=1)
rospy.init_node('test_node', anonymous=True)
rate=rospy.Rate(10)

while True:
    ret, frame = cap.read()
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    #cv.TM_CCOEFF,cv.TM_CCOEFF_NORMED,cv.TM_CCORR,cv.TM_CCORR_NORMED > using max value
    res = cv2.matchTemplate(gray_frame, template, cv2.TM_CCOEFF_NORMED)
    threshold = 0.7
    _, max_val, _, max_loc = cv2.minMaxLoc(res)
    top_left = max_loc
    bottom_right = (top_left[0] + w, top_left[1] + h)
    poseArr = PoseArray()
    if max_val >= threshold:
        cv2.rectangle(frame, top_left, bottom_right, (0, 255, 0), 3)

        point1 = Pose()
        point1.position.x = top_left[0]
        point1.position.y = top_left[1]
        point2 = Pose()
        point2.position.x = bottom_right[0]
        point2.position.y = bottom_right[1]
        poseArr.poses = [point1, point2]

        pub.publish(poseArr)

    cv2.imshow("Frame", frame)
    key = cv2.waitKey(1)

    if key ==27:
        break

cap.release()
cv2.destroyAllWindows()