#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import json

class VisionProcessor(Node):
    def __init__(self):
        super().__init__('vision_processor')
        
        self.declare_parameter('bubble_min_radius', 3)
        self.declare_parameter('bubble_max_radius', 80)
        
        self.bubble_min_radius = self.get_parameter('bubble_min_radius').value
        self.bubble_max_radius = self.get_parameter('bubble_max_radius').value
        
        self.bridge = CvBridge()
        self.current_color_name = None
        self.prev_gray = None
        self.frame_count = 0
        
        # For change detection
        self.last_reported_change = None
        
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.processed_image_pub = self.create_publisher(Image, '/camera/processed_image', 10)
        self.bubbles_pub = self.create_publisher(String, '/vision/bubbles', 10)
        self.color_pub = self.create_publisher(String, '/vision/color_status', 10)
        
        self.get_logger().info('Vision Processor - Fast processing mode')
    
    def get_detailed_color_name(self, hue, saturation, value):
        """Return detailed color name based on HSV values"""
        
        if saturation < 20:
            if value < 50:
                return "Black"
            elif value < 80:
                return "Charcoal Gray"
            elif value < 110:
                return "Dark Gray"
            elif value < 140:
                return "Medium Gray"
            elif value < 180:
                return "Light Gray"
            elif value < 220:
                return "Silver"
            else:
                return "White"
        
        if saturation < 50:
            if value > 200:
                if hue < 10 or hue > 170:
                    return "Pale Pink"
                elif hue < 25:
                    return "Peach"
                elif hue < 40:
                    return "Cream"
                elif hue < 70:
                    return "Pale Yellow"
                elif hue < 85:
                    return "Mint Green"
                elif hue < 100:
                    return "Pale Green"
                elif hue < 130:
                    return "Pale Cyan"
                elif hue < 145:
                    return "Sky Blue"
                elif hue < 155:
                    return "Lavender"
                else:
                    return "Pale Purple"
            else:
                if hue < 10 or hue > 170:
                    return "Dusty Rose"
                elif hue < 25:
                    return "Tan"
                elif hue < 40:
                    return "Beige"
                elif hue < 70:
                    return "Olive"
                elif hue < 100:
                    return "Sage Green"
                elif hue < 130:
                    return "Teal"
                elif hue < 155:
                    return "Steel Blue"
                else:
                    return "Mauve"
        
        if hue < 10 or hue > 170:
            if value < 80:
                return "Dark Maroon"
            elif value < 120:
                return "Burgundy"
            elif value < 160:
                return "Deep Red"
            elif value < 200:
                return "Blood Red"
            else:
                return "Bright Red" if saturation > 200 else "Crimson"
        
        elif hue < 25:
            if value < 100:
                return "Dark Orange"
            elif value < 150:
                return "Burnt Orange"
            elif value < 200:
                return "Deep Orange"
            else:
                return "Bright Orange" if saturation > 200 else "Orange"
        
        elif hue < 40:
            if value < 120:
                return "Dark Amber"
            elif value < 180:
                return "Amber"
            else:
                return "Golden Yellow"
        
        elif hue < 70:
            if value < 100:
                return "Dark Yellow"
            elif value < 150:
                return "Mustard Yellow"
            elif value < 200:
                return "Gold"
            else:
                return "Bright Yellow" if saturation > 200 else "Lemon Yellow"
        
        elif hue < 85:
            if value < 120:
                return "Dark Lime"
            elif value < 180:
                return "Lime Green"
            else:
                return "Chartreuse"
        
        elif hue < 100:
            if value < 80:
                return "Dark Forest Green"
            elif value < 120:
                return "Forest Green"
            elif value < 160:
                return "Deep Green"
            elif value < 200:
                return "Emerald Green"
            else:
                return "Bright Green" if saturation > 200 else "Lime Green"
        
        elif hue < 130:
            if value < 100:
                return "Dark Teal"
            elif value < 150:
                return "Teal"
            elif value < 200:
                return "Turquoise"
            else:
                return "Bright Cyan" if saturation > 200 else "Aqua"
        
        elif hue < 145:
            if value < 100:
                return "Dark Blue"
            elif value < 150:
                return "Deep Blue"
            elif value < 200:
                return "Ocean Blue"
            else:
                return "Azure Blue"
        
        elif hue < 155:
            if value < 80:
                return "Navy Blue"
            elif value < 120:
                return "Dark Blue"
            elif value < 160:
                return "Royal Blue"
            elif value < 200:
                return "Cobalt Blue"
            else:
                return "Bright Blue" if saturation > 200 else "Sky Blue"
        
        elif hue < 165:
            if value < 120:
                return "Dark Indigo"
            elif value < 180:
                return "Indigo"
            else:
                return "Periwinkle"
        
        else:
            if value < 100:
                return "Dark Purple"
            elif value < 150:
                return "Deep Purple"
            elif value < 200:
                return "Violet"
            else:
                return "Bright Purple" if saturation > 200 else "Lavender Purple"
    
    def detect_bubbles_fast(self, frame):
        """Optimized fast bubble detection"""
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        h, w = gray.shape
        
        # Skip every other frame for bubble detection
        if self.frame_count % 2 != 0:
            self.frame_count += 1
            return []
        
        # Simple fast thresholding
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        mean_br = np.mean(blurred)
        _, thresh = cv2.threshold(blurred, int(mean_br + 30), 255, cv2.THRESH_BINARY)
        
        # Quick morphology
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
        
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        bubbles = []
        min_area = np.pi * (self.bubble_min_radius ** 2)
        max_area = np.pi * (self.bubble_max_radius ** 2)
        
        for contour in contours[:20]:  # Limit to 20 contours for speed
            area = cv2.contourArea(contour)
            if area < min_area or area > max_area:
                continue
            
            perimeter = cv2.arcLength(contour, True)
            if perimeter == 0:
                continue
            
            circularity = 4 * np.pi * area / (perimeter * perimeter)
            if circularity < 0.6:
                continue
            
            M = cv2.moments(contour)
            if M['m00'] == 0:
                continue
            
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            radius = int(np.sqrt(area / np.pi))
            
            bubbles.append({
                'x': int(cx), 
                'y': int(cy), 
                'radius': int(radius),
                'confidence': float(round(circularity, 2))
            })
        
        self.prev_gray = gray
        self.frame_count += 1
        return bubbles
    
    def detect_color_change(self, frame):
        """Detect color and track changes"""
        # Only check color every 5 frames for speed
        if self.frame_count % 5 != 0:
            if self.current_color_name:
                return {
                    'changed': False,
                    'current_color': self.current_color_name,
                    'previous_color': self.last_reported_change
                }
        
        hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
        h, w = frame.shape[:2]
        center = hsv[h//4:3*h//4, w//4:3*w//4]
        
        hue = float(np.mean(center[:,:,0]))
        sat = float(np.mean(center[:,:,1]))
        val = float(np.mean(center[:,:,2]))
        
        color_name = self.get_detailed_color_name(hue, sat, val)
        
        # First time initialization
        if self.current_color_name is None:
            self.current_color_name = color_name
            self.last_reported_change = None
            return {
                'changed': False,
                'current_color': color_name,
                'previous_color': None
            }
        
        # Check if color actually changed
        if color_name != self.current_color_name:
            # New change detected
            previous = self.current_color_name
            self.current_color_name = color_name
            self.last_reported_change = previous
            
            return {
                'changed': True,
                'current_color': color_name,
                'previous_color': previous
            }
        else:
            # No change
            return {
                'changed': False,
                'current_color': color_name,
                'previous_color': self.last_reported_change
            }
    
    def draw_detections(self, frame, bubbles, color_info):
        """Fast drawing"""
        output = frame.copy()
        
        # Draw bubbles
        for bubble in bubbles:
            cv2.circle(output, (bubble['x'], bubble['y']), bubble['radius'], (0, 255, 255), 2)
        
        # Display color
        current = color_info.get('current_color', 'Unknown')
        
        if color_info.get('changed'):
            # Show change notification
            prev = color_info.get('previous_color', 'Unknown')
            cv2.putText(output, f"COLOR CHANGED!", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.putText(output, f"{prev} -> {current}", (10, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        else:
            # Just show current color
            cv2.putText(output, f"Color: {current}", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        cv2.putText(output, f"Bubbles: {len(bubbles)}", (10, 90), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        return output
    
    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            
            bubbles = self.detect_bubbles_fast(frame)
            color_info = self.detect_color_change(frame)
            processed_frame = self.draw_detections(frame, bubbles, color_info)
            
            # Compress image more for faster transmission
            _, buffer = cv2.imencode('.jpg', cv2.cvtColor(processed_frame, cv2.COLOR_RGB2BGR), 
                                     [cv2.IMWRITE_JPEG_QUALITY, 70])
            
            processed_msg = self.bridge.cv2_to_imgmsg(
                cv2.imdecode(buffer, cv2.IMREAD_COLOR), 
                encoding='bgr8'
            )
            processed_msg.header = msg.header
            self.processed_image_pub.publish(processed_msg)
            
            self.bubbles_pub.publish(String(data=json.dumps({
                'count': len(bubbles), 
                'bubbles': bubbles[:10],  # Limit data
                'timestamp': int(self.get_clock().now().to_msg().sec)
            })))
            
            self.color_pub.publish(String(data=json.dumps(color_info)))
            
        except Exception as e:
            self.get_logger().error(f'Error: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = VisionProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
