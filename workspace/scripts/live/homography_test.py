import cv2
import time
import numpy as np
Camera_Index = 1


Webcam = cv2.VideoCapture(Camera_Index, cv2.CAP_DSHOW)
cv2.namedWindow("Webcam")
cv2.namedWindow("Transformed Workspace", cv2.WINDOW_NORMAL)

clicks = []

workspace_cords = []

Display_Scale = 3
Crop_Min_X = 0
Crop_Min_Y = 0

def mouse_click(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        print("Left Click at: ", x, y)
        clicks.append(((x, y,), (0, 255, 255)))
    elif event == cv2.EVENT_RBUTTONDOWN:
        print("Right Click at: ", x, y)
        clicks.append(((x, y,), (0, 0, 255)))



def workspace_click(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        workspace_x = x / Display_Scale + Crop_Min_X
        workspace_y = y / Display_Scale + Crop_Min_Y
        conversion_factor = 1
        

        print("Workspace coordinate:", round(conversion_factor*workspace_x,3),"mm ", round(conversion_factor*workspace_y,3), "mm" )
        workspace_cords.append((round(conversion_factor*workspace_x,3), round(conversion_factor*workspace_y,3)))


cv2.setMouseCallback("Webcam", mouse_click)
cv2.setMouseCallback("Transformed Workspace", workspace_click)
Workspace_Width = round(1100) 
Workspace_Height = round(850)
P1 = (400,400)
P2 = (400,100)
P3 = (100,100)
P4 = (100,400)

Workspcace_Estimate = [P1, P2, P3, P4]

Workspace_Width = 300
Workspace_Height = 300

H = None
while True:
    ret, frame = Webcam.read()
    if not ret:
        break
    
 
    action = cv2.waitKey(1)
    if action == ord("q"):
        break
    if action == ord("s"):
        cv2.imwrite("screenshot"+str(int(time.time()))+".png", frame)
        print("Screenshot saved as screenshot"+str(int(time.time()))+".png")

    if action == ord("c"):
            clicks.clear()
            print("Cleared all clicks")
            
    if action == ord("h"):
        pixel_points = []
        if len(clicks) == 4:
            for (x,y), color in clicks:
                pixel_points.append([x, y])
            pixel_points = np.array(pixel_points, dtype=np.float32)
            workspace_points = np.array(Workspcace_Estimate, dtype=np.float32)

            H_robot, status = cv2.findHomography(pixel_points, workspace_points)

            Crop_Min_X = np.min(workspace_points[:, 0])
            Crop_Max_X = np.max(workspace_points[:, 0])
            Crop_Min_Y = np.min(workspace_points[:, 1])
            Crop_Max_Y = np.max(workspace_points[:, 1])

            Workspace_Width = int(round((Crop_Max_X - Crop_Min_X) * Display_Scale)) + 1
            Workspace_Height = int(round((Crop_Max_Y - Crop_Min_Y) * Display_Scale)) + 1

            Scale_Crop = np.array([
                [Display_Scale, 0, -Crop_Min_X * Display_Scale],
                [0, Display_Scale, -Crop_Min_Y * Display_Scale],
                [0, 0, 1]
            ], dtype=np.float32)

            H = Scale_Crop @ H_robot

            print("Homography Matrix: ", H)

            cv2.resizeWindow("Transformed Workspace", Workspace_Width, Workspace_Height)
            warpedframe = cv2.warpPerspective(frame, H, (Workspace_Width, Workspace_Height))

    if action == ord("d"):
        if len(workspace_cords) > 1:
            diff = np.array(workspace_cords[-1]) - np.array(workspace_cords[-2])
            print("Difference between last two workspace coordinates: ", round(diff[0],3), "mm ", round(diff[1],3), "mm")

    if H is not None:
        warpedframe = cv2.warpPerspective(frame, H, (Workspace_Width, Workspace_Height))
        cv2.imshow("Transformed Workspace", warpedframe)

            
                    
            



    for click, color in clicks:
        cv2.circle(frame, click, 4, color, 5)

 
    cv2.imshow("Webcam", frame)

    

Webcam.release()
cv2.destroyAllWindows()