import numpy as np
import cv2
import cv2.cv as cv
import facetracker
from video import create_capture
from common import clock, draw_str
import itertools
import scipy.io as sio

help_message = '''
USAGE: facedetect.py [--cascade <cascade_fn>] [--nested-cascade <cascade_fn>] [<video_source>]
'''

def main(video_src, face_fn, con_fn, tri_fn):
    
    #
    # Create the face tracker
    #
    tracker = facetracker.FaceTracker(face_fn)
    conns = facetracker.LoadCon(con_fn)
    trigs = facetracker.LoadTri(tri_fn)
    tracker.setWindowSizes((7,))

    #
    # Create a video capture
    #
    cam = create_capture(video_src)
    cam.set(cv2.cv.CV_CAP_PROP_POS_FRAMES, 0)
    frames_count = int(cam.get(cv2.cv.CV_CAP_PROP_FRAME_COUNT))
    
    total_frames = 0
    frames_noface = 0
    delta_index_error = 0
    frames_error = 0
    
    shape2D_mat = []
    shape3D_mat = []
    try:
        while True:
            t = clock()
            
            #
            # Get the new image
            #
            ret, img = cam.read()
            if not ret:
                frame_index = int(cam.get(cv2.cv.CV_CAP_PROP_POS_FRAMES))
                
                #
                # Check if last frame
                #
                if frame_index == frames_count:
                    break
                
                #
                # Advance (or repeat) frame
                #
                cam.set(cv2.cv.CV_CAP_PROP_POS_FRAMES, frame_index+delta_index_error)
                
                print frame_index, delta_index_error
                
                #
                # On repeat we need to break
                #
                if delta_index_error > 0:
                    break

                delta_index_error = 1
                continue
            
            delta_index_error = 0
                
            total_frames += 1
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            gray = cv2.equalizeHist(gray)
    
            #
            # Perform tracking
            #
            if tracker.update(gray):
                draw_str(img, (20, 40), 'pos: %.1f, %.1f' % tracker.getPosition())
                draw_str(img, (20, 60), 'scale: %.1f ' % tracker.getScale())
                draw_str(img, (20, 80), 'orientation: %.1f, %.1f, %.1f' % tracker.getOrientation())
                tracker.getScale()
                tracker.getOrientation()
                img = tracker.draw(img, conns, trigs)
                
                shape2D = tracker.get2DShape()[0].reshape((2, 66))
                shape2D_mat.append(shape2D)
                shape3D = tracker.get3DShape().reshape((3, 66))
                shape3D_mat.append(shape3D)
                
            else:
                frames_noface += 1
                shape2D_mat.append(-np.ones((2, 66)))
                shape3D_mat.append(-np.ones((3, 66)))
                tracker.setWindowSizes((11, 9, 7))
            
            dt = clock() - t
    
            #
            # Show image
            #
            draw_str(img, (20, 20), 'time: %.1f ms' % (dt*1000))
            cv2.imshow('facedetect', img)
    
            #
            # Save in video
            #
            if 0xFF & cv2.waitKey(5) == 27:
                break
    finally:
        cv2.destroyAllWindows() 			
        print total_frames, frames_noface, frames_error
            
        #
        # Store the tracked shape
        #
        if shape3D_mat != []:
            sio.savemat('shape.mat', {'shape3D':shape3D_mat, 'shape2D':shape2D_mat}, do_compression=True)
        

if __name__ == '__main__':
    import sys, getopt
    print help_message
    
    args, video_src = getopt.getopt(sys.argv[1:], '', ['face=', 'con=', 'tri='])
    
    try:
        video_src = video_src[0]
    except:
        video_src = 0
        
    args = dict(args)
    face_fn = args.get('--con', r"..\external\FaceTracker\model\face.tracker")
    con_fn = args.get('--con', r"..\external\FaceTracker\model\face.con")
    tri_fn  = args.get('--tri', r"..\external\FaceTracker\model\face.tri")

    main(video_src, face_fn, con_fn, tri_fn)