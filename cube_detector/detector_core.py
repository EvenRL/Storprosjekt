import cv2 as cv
import numpy as np

def findCubePoses(image: np.ndarray, colors: dict, K: np.ndarray, D: np.ndarray, params: dict):
    # Setup - Load parameters
    cubeSize     = params.get('cubeSize', 0.05) # Size of cube
    minArea      = params.get('minArea', 2500) # Minimum area of contour
    maxArea      = params.get('maxArea', 30000) # Maximum area of contour
    alpha        = params.get('alpha', .05) # Epsilon factor, percent of arc length
    morphOpen   = params.get('morphOpen', 3)
    morphClose  = params.get('morphClose', 5)
    reprojMax   = params.get('reprojMax', 8.0) # Max reprojection error
    enableBlur   = params.get('enableBlur', True) # Enable gaussian blur
    blurKernelSize = params.get('blurKernelSize', 5) # Kernel size for gaussian blur
    displayColorMask = params.get('displayColorMask', False) # Disply only hsv color mask, useful for color tuning
    preprocessing = params.get('preprocessing', True) # Preproccessing of image using clahe
    claheClipLimit = params.get('claheClip', 2.0) # Clahe clip limit
    claheTileSize = params.get('claheTile', 8) # Clahe tile size
    enableMaskCleanup = params.get('maskCleanup', True) # Enable morphology opening and closing on color mask before finding contours
    drawContours = params.get('drawContours', False) # Draw found contours on overlay
    drawPoints = params.get('drawPoints', True) # Draw found points on overlay
    drawPointOrder = params.get('drawPointOrder', False) # Draw point order on overlay, 1 bl, 2 br, 3 tr, 4 tl
    drawPose = params.get('drawPose', True) # Draw estimated pose on overlay
    minSolidity = params.get('minSolidity', 0.9) # Minimum contour solidity
    minExtent = params.get('minExtent', 0.7) # Minimum contour extent
    minAspect = params.get('minAspect', 0.6) # Minimum contour aspect ratio
    maxAspect = params.get('maxAspect', 1.4) # Maximum contour aspect ratio

    # Define geometries to look for
    square_points = np.array([[0,0,0],[cubeSize,0,0],[cubeSize,cubeSize,0],[0,cubeSize,0]], dtype=np.float32) # Define Square geometry to compare with contour, 5x5cm Square.
    cube_points = np.array([[0,0,0],[cubeSize,0,0],[cubeSize,cubeSize,0],[0,cubeSize,0],
                            [0,0,-cubeSize],[cubeSize,0,-cubeSize],[cubeSize,cubeSize,-cubeSize],[0,cubeSize,-cubeSize]]) # Define Cube geometry, 5x5 Cube
    
    cubes = []

    if preprocessing:
        image = PreprocessImage(image, claheClipLimit, claheTileSize)

    if enableBlur:
        image = cv.blur(image,(blurKernelSize, blurKernelSize))

    overlay = image.copy()
    hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)

    if displayColorMask:
        color = list(colors.keys())[0]
        mask = cv.inRange(hsv, colors[color][0], colors[color][1])
        if enableMaskCleanup:
            kernel_open  = cv.getStructuringElement(cv.MORPH_RECT, (morphOpen,morphOpen))
            kernel_close = cv.getStructuringElement(cv.MORPH_RECT, (morphClose,morphClose))
            mask = cv.morphologyEx(mask, cv.MORPH_OPEN,  kernel_open)
            mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel_close)
        return cubes, mask
    
    # Begin main loop
    for name, clr in colors.items():
        mask = cv.inRange(hsv, clr[0], clr[1])

        # Mask cleanup before finding contours
        if enableMaskCleanup:
            kernel_open  = cv.getStructuringElement(cv.MORPH_RECT, (morphOpen,morphOpen))
            kernel_close = cv.getStructuringElement(cv.MORPH_RECT, (morphClose,morphClose))
            mask = cv.morphologyEx(mask, cv.MORPH_OPEN,  kernel_open)
            mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel_close)

        # Find contours
        cnts, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

        # Draw contours
        if drawContours:
            overlay = DrawContours(overlay, cnts, minArea, maxArea)
        
        for cnt in cnts:
            # Check area
            area = cv.contourArea(cnt)
            if not minArea < area < maxArea: continue

            # Check for odd shapes
            hull = cv.convexHull(cnt)
            solidity = area / float(cv.contourArea(hull))
            x,y,w,h = cv.boundingRect(cnt)
            extent = area / float(w*h)
            aspect = w / float(h)
            if solidity < minSolidity:         continue  # ragged or concave
            if extent   < minExtent:         continue  # holes / glare
            if not minAspect < aspect < maxAspect:  continue  # too elongated

            # Approximate shape of contour
            peri   = cv.arcLength(cnt, True)
            eps    = max(3.0, 0.02 * peri)
            approx = cv.approxPolyDP(cnt, eps, True)

            if len(approx) == 4: # If contour is rectangle
                corners = sortCorners(approx.reshape(-1,2))
                # Draw points and point order
                if drawPoints:
                    overlay = DrawPoints(overlay, corners, drawPointOrder)

                image_points = np.array(corners, dtype=np.float32)

                # Estimate pose
                success, rvec, tvec, inliers = cv.solvePnPRansac(square_points, image_points, K, D, reprojectionError=reprojMax, confidence=0.99, flags=cv.SOLVEPNP_ITERATIVE)

                if not success: continue

                imgpts, _ = cv.projectPoints(cube_points, rvec, tvec, K, D)
                imgpts = np.int32(imgpts).reshape(-1,2)

                # Calculate reprojection error
                inl     = inliers.flatten()
                err_inl = np.mean(np.linalg.norm(imgpts[inl] - image_points[inl], axis=1))

                # Draw pose
                if drawPose:
                    overlay = DrawPose(overlay, imgpts)

                cubes.append({
                "color"      : name,
                "tvec"       : tvec.flatten().tolist(), # [x, y, z]
                "rvec"       : rvec.flatten().tolist(), # [rx, ry, rz]
                "error"      : err_inl
                })
    return cubes, overlay
            
    

def PreprocessImage(img_bgr: np.ndarray, claheClip, claheTile) -> np.ndarray:
    """Return a normalised BGR image using clahe method."""
    hsv = cv.cvtColor(img_bgr, cv.COLOR_BGR2HSV)
    v    = hsv[:,:,2]
    clahe = cv.createCLAHE(clipLimit=claheClip, tileGridSize=(claheTile, claheTile))
    v_eq = clahe.apply(v)
    hsv[:,:,2] = v_eq
    return cv.cvtColor(hsv, cv.COLOR_HSV2BGR)

def DrawContours(image, cnts, min, max):
    okContour = list() # Contours within area limits
    oobContour = list() # Contours outside area limits
    for cnt in cnts:
        area = cv.contourArea(cnt)
        hull = cv.convexHull(cnt)
        solidity = area / float(cv.contourArea(hull))
        x,y,w,h = cv.boundingRect(cnt)
        extent = area / float(w*h)
        aspect = w / float(h)
        if (min < area < max) and (solidity < 0.9) and (extent < 0.70) and not (0.6 < aspect < 1.4):
            okContour.append(cnt)
        else:
            oobContour.append(cnt)
    cv.drawContours(image, okContour, -1, (0,255,0), 3)
    cv.drawContours(image, oobContour, -1, (0,0,255), 3)
    return image

def sortCorners(corners):
        '''
        Order the points from poly approximation to make the following solvePnP function more stable.
        Order is: bottom left, bottom right, top right, top left.
        '''
        sorted_points = np.zeros((4,2), dtype=int)

        sum = corners.sum(axis=1)
        difference = corners[:,0] - corners[:,1]

        sorted_points[0] = corners[np.argmin(difference)] # Bottom left is smallest difference x - y
        sorted_points[1] = corners[np.argmax(sum)] # Bottom right is biggest sum x + y
        sorted_points[2] = corners[np.argmax(difference)] # Top right is biggest difference x - y
        sorted_points[3] = corners[np.argmin(sum)] # Top left is smallest sum x + y
        
        return sorted_points

def DrawPoints(image, points, drawOrder):
    i = 0
    for point in points:
        cv.circle(image, tuple(point), radius=5, color=(0,255,0), thickness=-1)
        if drawOrder:
            i += 1
            cv.putText(image, str(i), tuple(point), cv.FONT_HERSHEY_SIMPLEX, 1, (255,255,255))
    return image

def DrawPose(image, imgpts):
    cv.drawContours(image,[imgpts[:4]], -1, (255,0,0), 2)
    for i in range(4):
        cv.line(image, tuple(imgpts[i]), tuple(imgpts[i+4]), (0,255,0), 2)
    cv.drawContours(image, [imgpts[4:]], -1, (0,0,255), 2)
    return image