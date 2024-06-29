import cv2
import math
import logging
import rclpy
import numpy             as np
import networkx          as nx
import matplotlib.pyplot as plt

from   shapely.geometry         import LineString
from   nav_msgs.msg             import OccupancyGrid
from   scipy.spatial.transform  import Rotation
from   tf2_ros                  import TransformListener, Buffer

#TODO: Make a function to calculate line of sight conditions. Remove redundant points
#TODO: Make a weighted graph based on number of points to be passed to reach a given location
#TODO: Draw max overlapping lines based on available coordinates from graph.(Only to the given contour)
#TODO: Consider the coordinates except which lies with in the line

class LOSpathFinder():

    logging.basicConfig(level=logging.INFO)  # Set the default log level to INFO

    def __init__(self, node):
        self.node = node

        self.tf_buffer    = Buffer()
        self.tf_listener  = TransformListener(self.tf_buffer, self.node)

        self.map_subscription = self.node.create_subscription(OccupancyGrid, 'map', self.map_callback, 10) # Map Subscription

        self.kernel5x5    = np.ones((5,5)  , dtype=np.float32)  #For noise reduction
        self.kernel7x7    = np.ones((7,7)  , dtype=np.float32)
        self.kernel3x3    = np.ones((3,3)  , dtype=np.float32)

        self.robot_x = 0
        self.robot_y = 0
        self.map     = OccupancyGrid()

    def map_callback(self, map):
        self.map = map

    def get_transform(self, target_frame ,source_frame):
        try:
            transform_stamped = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))  
            return transform_stamped.transform

        except Exception as e:
            self.node.get_logger().warn("Failed to get transform: {}".format(e))
            return None
        
    def getRecoveryPath(self):
            
            map          = self.map
            x_world,y_world,resolution      = self.getWorldCoordinates(map)     # Get world origin and map resolution
            map_img, map_height, map_width  = self.loadMap(map)                 #smaller maps 0.75      larger maps 0.45
            map_img                         = cv2.flip(map_img, 0)              # Get original pose in opencv

            x_origin = int((-x_world)*self.scale)
            y_origin = int((map_height+y_world)*self.scale)

            transform = self.get_transform('map', 'base_link')
            self.robot_x, self.robot_y = self.getRobotCoordinates(resolution, (x_origin, y_origin), (transform.translation.x, transform.translation.y))

            freespace_black, freespace_black_filtered, boundary, boundary_inflated = self.regionExtractor(map_img)
            freespace_white_filtered                                               = 255 - freespace_black_filtered
            freespace_white_filtered_rgb                                           = cv2.cvtColor(freespace_white_filtered,cv2.COLOR_GRAY2BGR)
            
            data_points = self.LOSpointCalc(freespace_white_filtered_rgb)
            graph       = self.calculateOptimalPoints(freespace_white_filtered_rgb, data_points)
            best_lines  = self.filterGraph(freespace_white_filtered_rgb,graph)
            LOS_points  = self.calculateBestPoints(best_lines,freespace_white_filtered_rgb)  #TODO: Change display image

            LOS_points_sorted = sorted(LOS_points,key=lambda x: x[2], reverse=True)
            self.mergeNearbyPoints(LOS_points_sorted)
            nearest_junc = self.robotNearestKeyPoint(freespace_white_filtered,(self.robot_x,self.robot_y),LOS_points_sorted)
            path         = self.createGraph(LOS_points_sorted,(self.robot_x,self.robot_y),nearest_junc)
            keyPoints    = self.visualizePath(path,freespace_white_filtered)
            keyPoints    = self.convertPixelsToMAPFrame(keyPoints, (x_origin,y_origin), resolution)

            if logging.getLogger().isEnabledFor(logging.DEBUG):
                robot_location_img = freespace_white_filtered_rgb.copy() #TODO: Remove after visualization
                cv2.namedWindow('mouse_control')
                #cv2.setMouseCallback('mouse_control', self.mouse_callback)
                cv2.circle(robot_location_img,(self.robot_x,self.robot_y),8,(255,0,0),-1)
                cv2.imshow('mouse_control',robot_location_img)

                #for visualization
                for i in range(len(LOS_points_sorted)):                  #TODO: change later. just for testing
                    if i==5:
                        break
                    cv2.circle(freespace_white_filtered_rgb,(LOS_points_sorted[i][0],LOS_points_sorted[i][1]),8,(0,0,255),-1)
                    logging.debug("{}: {}            {}".format((LOS_points_sorted[i][0], LOS_points_sorted[i][1]), LOS_points_sorted[i][2], LOS_points_sorted[i][3]))

                # Map Origin 
                
                cv2.circle(freespace_white_filtered_rgb,(x_origin,y_origin),5,(100,255,50),-1)
                #cv2.imshow("Output",freespace_white_filtered_rgb)        #TODO: visualization
                #cv2.waitKey(1)
           
            logging.info("robot location: %s",(round((self.robot_x-x_origin)*resolution/self.scale, 1),round((-self.robot_y+y_origin)*resolution/self.scale, 1)))
            logging.debug("world origin   : %s", (x_world,y_world))
            logging.info("Number of lines: %s", len(best_lines))

            return keyPoints
            
    def loadMap(self, map): 
        try:
            occupancy_data = np.array(map.data)                             # Occupany data is a 1D array with occupancy grid data
            width  = map.info.width
            height = map.info.height

            occupancy_grid  = occupancy_data.reshape((height, width))       # Reshape the 1D array to a 2D array (occupancy grid) 
            map_image       = cv2.convertScaleAbs(occupancy_grid)           # Convert to unsigned format
        
            map_image[map_image == 1] = 50                                  #TODO: Could be changed based on map generation parameters
            self.scale                     = self.scalingFactor([width, height])

            logging.debug(" width, height : %s",(width,height))
            logging.info(" Scaling fac : %s"  ,self.scale)
   
            map_image = cv2.resize(map_image, (int(width*self.scale), int(height*self.scale)))   # Resize the image by 50%

        except Exception as err:
            logging.error("Error in loading Image: %s", err)

        return map_image, height, width

    #TODO: modify the map, add  gaussian blur and then median filter
    def regionExtractor(self, map_img): #Done
        
        _, boundary             = cv2.threshold(map_img, 80, 255, cv2.THRESH_BINARY)
        boundary_dilate         = cv2.dilate(boundary,self.kernel5x5,iterations=4)
        boundary_dilate_erode   = cv2.erode(boundary_dilate,self.kernel5x5,iterations=2)

        gaussian_filtered       = cv2.GaussianBlur(map_img, (5, 5), 0)
        median_filtered         = cv2.medianBlur(gaussian_filtered, ksize=5) 
        _, freespace            = cv2.threshold(median_filtered, 40, 255, cv2.THRESH_BINARY)
        freespace_dilate        = cv2.dilate(freespace, self.kernel5x5, iterations=2)
        freespace_dilate_erode  = cv2.erode(freespace_dilate, self.kernel5x5,iterations=2)
        boundary_inflated       = cv2.bitwise_and(boundary_dilate_erode,freespace_dilate_erode)
        
        if logging.getLogger().isEnabledFor(logging.DEBUG):
            cv2.imshow("median filtered image",median_filtered)
        return freespace, freespace_dilate_erode, boundary, boundary_inflated

    #TODO: Use a extruded map for thinning function
    def LOSpointCalc(self, img): 
        gray_image      = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        thinned_image   = cv2.ximgproc.thinning(gray_image)             # Draw red circles on detected contour points after approximation
        contours, _     = cv2.findContours(thinned_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        data_points     = []  # Draw red circles on detected contour points after approximation
        contour_image = img.copy()
        for contour in contours:
        
            epsilon = 0.005 * cv2.arcLength(contour, True)  # Approximate the contour with Douglas-Peucker algorithm
            approx = cv2.approxPolyDP(contour, epsilon, True)
            for point in approx:
                data_points.append(point[0])
            
        if logging.getLogger().isEnabledFor(logging.DEBUG):
            # Display the results
            cv2.imshow('Original Image', img)
            cv2.imshow('Thinned Image' , thinned_image)
            cv2.imshow('Contours with Red Circles (Approximated in Blue)', contour_image)
            cv2.waitKey(1)

        return data_points

    def getWorldCoordinates(self, map):     
        x_world     = map.info.origin.position.x
        y_world     = map.info.origin.position.y
        resolution  = map.info.resolution

        return round(x_world/resolution),round(y_world/resolution), resolution
    
    def getRobotCoordinates(self, resolution, map_origin, robot_location):
        return round(self.scale*robot_location[0]/resolution + map_origin[0]),round(-self.scale*robot_location[1]/resolution + map_origin[1])
    
    def calculateOptimalPoints(self, img,data_points): 
        gray_img    = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

        graph = []
        for i in range(len(data_points)):
            point_one = data_points[i]
            for point_two in data_points[i+1:]:
                mask_pixel_count, overlap_pixel_count= self.getOverlap(point_one,point_two,gray_img,2)
                if (mask_pixel_count-overlap_pixel_count)>30:
                    continue
   
                graph.append([point_one,point_two,overlap_pixel_count])

        return graph

    def getOverlap(self, pnt1,pnt2,gray_img,thickness):
        temp_img    = np.zeros_like(gray_img)
        cv2.line(temp_img,pnt1,pnt2,255,thickness)   # To check overlap; mask along line with thickness 2
        masked_img  = cv2.bitwise_and(temp_img,gray_img)  

        return np.sum(temp_img), np.sum(masked_img)

    def filterGraph(self, img,list1):
        best_lines      = []
        sorted_list     = sorted(list1, key=lambda x: x[2], reverse=True)
        img_gray        = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)      # Update occupied regions of lines a s a mask
        img_boundary    = img_gray.copy()                           # TO clip the extended lines
        img_display     = img.copy()                                # for displaying clipped lines
        img_rgb = img.copy()                                        # For displaying extendedelines 
        for j in range(len(sorted_list)):
            mask_pixel_count, overlap_pixel_count= self.getOverlap(sorted_list[j][0],sorted_list[j][1],img_gray,20)
            value = (sorted_list[j][2] - overlap_pixel_count)
            if value <180000000000:
                continue
            else:
                cv2.line(img_gray,sorted_list[j][0],sorted_list[j][1],0,30)    # Updating boundaries for previously drawn lines
                line = self.plotExtendedLine(img_display,img_boundary,img_rgb,sorted_list[j][0],sorted_list[j][1])
                best_lines.append(line)

        if logging.getLogger().isEnabledFor(logging.DEBUG):      
            cv2.imshow('Image with Extended Line', img_rgb)
            cv2.imshow('Clipped lines', img_display)
            cv2.imshow("Filtered map",img)
            cv2.imshow("Filtered boundaries",img_gray)

        return best_lines

    def plotExtendedLine(self, img_display, img_gray, img_rgb, point1, point2, extension_length=1000):

        limit       = 10000
        point1      = (int(point1[0]), int(point1[1]))      # Convert points to tuple of integers
        point2      = (int(point2[0]), int(point2[1]))

        if point2[0] - point1[0] != 0:                      # Calculate the slope of the line
            slope = (point2[1] - point1[1]) / (point2[0] - point1[0])
        else:
            slope = 1e10  # Vertical line

        # Extend the line in both directions
        extended_point1 = (int(point1[0] - extension_length), int(max((point1[1] - extension_length * slope),-limit)))
        extended_point2 = (int(point2[0] + extension_length), int(min((point2[1] + extension_length * slope),limit)))
        
        # Draw the extended line on the image
        color       = (0, 255, 0)  # Green color
        thickness   = 2
        line        = self.clipExtendedLines(img_gray,extended_point1,extended_point2,img_display)

        if logging.getLogger().isEnabledFor(logging.DEBUG):
            cv2.line(img_rgb, extended_point1, extended_point2, color, thickness)

        return line

    def clipExtendedLines(self, img,pnt1, pnt2, display):   # Grayscale image
        mask            = np.zeros_like(img)
        mask_filtered   = mask.copy()
        line            = np.zeros_like(img)

        cv2.line(mask,pnt1,pnt2,255,10)
        cv2.line(line,pnt1,pnt2,255,5)

        clipped_mask        = cv2.bitwise_and(mask,img)
        contours, hierarchy = cv2.findContours(clipped_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        largest_contour     = max(contours, key=cv2.contourArea)

        cv2.drawContours(mask_filtered, [largest_contour], -1, 255, -1)

        clipped_line    = cv2.bitwise_and(mask_filtered,line)
        x, y, w, h      = cv2.boundingRect(clipped_line)
        ret, p1, p2     = cv2.clipLine((x, y, w, h), pnt1,pnt2)
        
        if ret:
            cv2.line(display, p1, p2, (255,0,0), 3)
            return [p1,p2]
    
    def findIntersectionPoints(self, line1, line2):
        line_segment1 = LineString([(line1[0][0], line1[0][1]), (line1[1][0], line1[1][1])])
        line_segment2 = LineString([(line2[0][0], line2[0][1]), (line2[1][0], line2[1][1])])

        intersection = line_segment1.intersection(line_segment2)

        if intersection.is_empty:
            return None             # Lines do not intersect or overlap

        return intersection.x, intersection.y

    def calculateBestPoints(self, lines,img):
        LOS_points  = []
        for i in range(len(lines)):
            line1   = lines[i]
            for line2 in lines[i+1:]:
                intersection    = self.findIntersectionPoints(line1, line2)
                if intersection:
                    threshold   = self.calculateThreshold(line1,line2)              #TODO: save it to a file if possible
                    LOS_points.append([int(intersection[0]),int(intersection[1]),threshold,[tuple(line1),tuple(line2)]])
           
        return LOS_points

    def calculateThreshold(self,line1,line2):             #TODO: change later
        dist_line1 = math.dist(line1[0], line1[1])
        dist_line2 = math.dist(line2[0], line2[1])

        return dist_line1+dist_line2

    def checkInRange(self, pnt1, pnt2, size):
        return  abs(pnt1[0]-pnt2[0]) <size and abs(pnt1[1]-pnt2[1]) <size

    def mergeNearbyPoints(self, intersections):           #TODO: change intersection point insert method in the LOS points
        no_of_points = len(intersections)
        index1       = 0
        while index1 < no_of_points-1:
            index2   = index1+1
            while index2 < no_of_points:
                if self.checkInRange((intersections[index1][0],intersections[index1][1]),(intersections[index2][0],intersections[index2][1]),30):
                    intersections[index1][2] += intersections[index2][2]                    # Merge thresholds 
                    intersections[index1][3].extend(intersections[index2][3])               # Update number of lines
                    intersections.pop(index2)
                    intersections[index1][3] = list(set(intersections[index1][3]))
                    no_of_points -= 1
                else:
                    index2+=1

            index1+=1

    def maximizeCoverage(self, LOS_points,threshold):     #TODO: change this method of threshold. Use a image to check overlap with drawn lines
        max_coverage_points = []
        coverage            = threshold+1
        index1              = 0
        length              = len(LOS_points)
        while coverage > threshold:
            if index1 == length-3:
                break
            if length<3:
                return LOS_points
            max_coverage_points.append(LOS_points[index1])
            coverage    = LOS_points[index1][2]
            set1        = set(LOS_points[index1][3])
            
            for index2 in range(index1+1,len(LOS_points)):     
                set2            = set(LOS_points[index2][3])
                intersection    = list(set1.intersection(set2))
                if not intersection:
                    continue
                
                value = 0
                for line in intersection:
                    value += math.dist(line[0],line[1])
                LOS_points[index2][2] -= value              # Update threshold
        
            temp_LOS_points         = LOS_points[index1+1:]         # Sort remaning LOS points based on coverage
            temp_LOS_points_sorted  = sorted(temp_LOS_points,key = lambda x: x[2], reverse=True) 
            LOS_points[index1+1:]   = temp_LOS_points_sorted
            index1 += 1

        return max_coverage_points

    def createGraph(self, LOS_points, robot_location, nearest_junc): 

        point2 = robot_location
        if len(LOS_points) == 0:
            return
        else:
            G           = nx.Graph()
            point1      = (LOS_points[0][0],LOS_points[0][1])
            for index1 in range(len(LOS_points)):
                set1    = set(LOS_points[index1][3])
              
                if self.checkInRange(point2,(LOS_points[index1][0],LOS_points[index1][1]),10):
                    point2  = (LOS_points[index1][0],LOS_points[index1][1])
            
                for index2 in range(index1+1,len(LOS_points)):
                    set2            = set(LOS_points[index2][3])
                    intersection    = set1.intersection(set2)
                    if intersection:
                        G.add_edge(str(LOS_points[index1][0])+','+ str(LOS_points[index1][1]),str(LOS_points[index2][0])+','+ str(LOS_points[index2][1]), weight=1)

            for point in nearest_junc:
                G.add_edge(str(point[0])+','+ str(point[1]),str(robot_location[0])+','+ str(robot_location[1]), weight=1)


            pos     = nx.spring_layout(G)
            nx.draw(G, pos, with_labels=True, font_weight='bold', node_size=700)
            labels  = nx.get_edge_attributes(G, 'weight')
            nx.draw_networkx_edge_labels(G, pos, edge_labels=labels)
            path    = []
            try: 
                start_node = str(point1[0])+','+str(point1[1])
                end_node   = str(point2[0])+','+str(point2[1])
        
                path       = self.dijkstra(G, start_node, end_node)

            except Exception as err:
                logging.error("Graph Error: %s", err)

                
               
        return path
          
    def dijkstra(self, graph, start, end):
        shortest_distance           = {node: float('infinity') for node in graph.nodes}
        shortest_distance[start]    = 0
        priority_queue              = [(0, start)]
        
        while priority_queue:
            current_distance, current_node = min(priority_queue)
            priority_queue.remove((current_distance, current_node))
            
            for neighbor in graph.neighbors(current_node):
                weight      = graph[current_node][neighbor]['weight']
                distance    = current_distance + weight
                if distance < shortest_distance[neighbor]:
                    shortest_distance[neighbor] = distance
                    priority_queue.append((distance, neighbor))
        
        # Reconstructing the path from start to end
        path        = [end]
        current     = end
        while current != start:
            current = min((neighbor for neighbor in graph.neighbors(current)), key=shortest_distance.get)
            path.insert(0, current)
        
        return path

    def visualizePath(self, text_list, img):
        keyPoints   = []
        path_img    = cv2.cvtColor(img,cv2.COLOR_GRAY2RGB)
        i = 1
        for text in text_list:
            point   = tuple(map(int,text.split(',')))
            keyPoints.append(point)
            cv2.circle(path_img,point,8,(255,0,255),-1)
            cv2.putText(path_img, str(i), (point[0]+10,point[1]+10),cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 1)
            i+=1

        #logging.DEBUG('keypoints %s',(keyPoints))
        if logging.getLogger().isEnabledFor(logging.INFO):
            cv2.imshow("Path",path_img)

        cv2.waitKey(5000)
        return keyPoints

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_MOUSEMOVE:
            self.robot_x = x
            self.robot_y = y

    def robotNearestKeyPoint(self, gray_map,robot_location,LOS_points):
        nearest_junc        = []
        robot_mask          = np.zeros_like(gray_map)
        cv2.circle(robot_mask,robot_location,15,1,-1)
        for intersection in LOS_points:
            line_mask = np.zeros_like(gray_map)
            for line in intersection[3]:
                cv2.line(line_mask,line[0],line[1],1,30)  #TODO: calibrate the thickness

            overlap   = cv2.bitwise_and(robot_mask,line_mask)

            if (np.sum(overlap)>30):
                nearest_junc.append((intersection[0],intersection[1]))
        #logging.INFO("nearest junctions: %s",(nearest_junc))
        return nearest_junc

    def scalingFactor(self, map_dimensions):
        min_len         = min(map_dimensions)
 
        # Thresholds
        LARGEMAP_THERSH = 300
        SMALLMAP_THRESH = 150

        if  (min_len > LARGEMAP_THERSH):
            scale = 0.45                # Identified a s large map
        elif(min_len < SMALLMAP_THRESH):
            scale = 0.75
        else:
            scale = 0.75 - 0.3*(min_len-SMALLMAP_THRESH)/(LARGEMAP_THERSH-SMALLMAP_THRESH)

        return round(scale,2)
    
    def convertPixelsToMAPFrame(self, keypoints, mapOrigin, resolution):
        if(keypoints != None):
            for i in range(len(keypoints)):
                keypoints[i] = (round(((keypoints[i][0]-mapOrigin[0])*resolution/self.scale),1),round(((-keypoints[i][1]+mapOrigin[1])*resolution/self.scale),1))
            return keypoints




