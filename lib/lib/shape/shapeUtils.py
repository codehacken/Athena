import cv2


def compare_items(cnt1, cnt2):
   ret = cv2.matchShapes(cnt1,cnt2,1,0.0)
   print "Shape is compared : ", ret

