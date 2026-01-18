#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import asyncio
import websockets
import json
import base64
import threading
from http.server import HTTPServer, SimpleHTTPRequestHandler
from pathlib import Path

class WebServer(Node):
    def __init__(self):
        super().__init__('web_server')
        
        self.declare_parameter('http_port', 8000)
        self.declare_parameter('ws_port', 9000)
        
        self.http_port = self.get_parameter('http_port').value
        self.ws_port = self.get_parameter('ws_port').value
        
        self.bridge = CvBridge()
        self.latest_image = None
        self.latest_bubbles = None
        self.latest_color = None
        self.logs = []
        self.max_logs = 100
        self.ws_clients = set()
        
        # Event loop for async operations
        self.loop = None
        
        self.create_subscription(Image, '/camera/processed_image', self.image_callback, 10)
        self.create_subscription(String, '/vision/bubbles', self.bubbles_callback, 10)
        self.create_subscription(String, '/vision/color_status', self.color_callback, 10)
        
        self.get_logger().info(f'Web Server: HTTP on port {self.http_port}, WebSocket on port {self.ws_port}')
        self.add_log('info', 'Bubble Vision Monitor initialized')
    
    def add_log(self, level, message):
        log_entry = {
            'timestamp': self.get_clock().now().to_msg().sec,
            'level': level,
            'message': message
        }
        self.logs.append(log_entry)
        
        if len(self.logs) > self.max_logs:
            self.logs = self.logs[-self.max_logs:]
        
        # Schedule broadcast without blocking
        if self.loop and self.ws_clients:
            asyncio.run_coroutine_threadsafe(
                self.broadcast_data('log', log_entry), self.loop
            )
    
    def image_callback(self, msg):
        try:
            import cv2
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            _, buffer = cv2.imencode('.jpg', cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR))
            jpg_as_text = base64.b64encode(buffer).decode('utf-8')
            
            self.latest_image = jpg_as_text
            
            if self.loop and self.ws_clients:
                asyncio.run_coroutine_threadsafe(
                    self.broadcast_data('image', {'data': jpg_as_text}), self.loop
                )
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')
    
    def bubbles_callback(self, msg):
        try:
            data = json.loads(msg.data)
            self.latest_bubbles = data
            
            if data['count'] > 0:
                self.add_log('info', f"Detected {data['count']} bubble(s)")
            
            if self.loop and self.ws_clients:
                asyncio.run_coroutine_threadsafe(
                    self.broadcast_data('bubbles', data), self.loop
                )
        except Exception as e:
            self.get_logger().error(f'Error processing bubbles: {str(e)}')
    
    def color_callback(self, msg):
        try:
            data = json.loads(msg.data)
            self.latest_color = data
            
            if data.get('changed'):
                self.add_log('warning', 'Liquid color has changed!')
            
            if self.loop and self.ws_clients:
                asyncio.run_coroutine_threadsafe(
                    self.broadcast_data('color', data), self.loop
                )
        except Exception as e:
            self.get_logger().error(f'Error processing color: {str(e)}')
    
    async def broadcast_data(self, data_type, data):
        if self.ws_clients:
            message = json.dumps({'type': data_type, 'data': data})
            websockets.broadcast(self.ws_clients, message)
    
    async def handle_websocket(self, websocket):
        self.ws_clients.add(websocket)
        self.get_logger().info(f'Client connected. Total: {len(self.ws_clients)}')
        
        try:
            if self.latest_image:
                await websocket.send(json.dumps({'type': 'image', 'data': {'data': self.latest_image}}))
            
            if self.latest_bubbles:
                await websocket.send(json.dumps({'type': 'bubbles', 'data': self.latest_bubbles}))
            
            if self.latest_color:
                await websocket.send(json.dumps({'type': 'color', 'data': self.latest_color}))
            
            await websocket.send(json.dumps({'type': 'logs', 'data': self.logs}))
            
            async for message in websocket:
                pass
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            self.ws_clients.remove(websocket)
            self.get_logger().info(f'Client disconnected. Total: {len(self.ws_clients)}')
    
    async def start_websocket_server(self):
        async with websockets.serve(self.handle_websocket, "0.0.0.0", self.ws_port):
            await asyncio.Future()

def main(args=None):
    rclpy.init(args=args)
    node = WebServer()
    
    web_dir = Path(__file__).parent.parent / 'web'
    
    class CustomHandler(SimpleHTTPRequestHandler):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, directory=str(web_dir), **kwargs)
    
    http_server = HTTPServer(('0.0.0.0', node.http_port), CustomHandler)
    http_thread = threading.Thread(target=http_server.serve_forever, daemon=True)
    http_thread.start()
    
    print(f'\n{"="*60}')
    print(f'Bubble Vision Monitor')
    print(f'{"="*60}')
    print(f'Dashboard: http://localhost:{node.http_port}')
    print(f'WebSocket: ws://localhost:{node.ws_port}')
    print(f'{"="*60}\n')
    
    def run_ws_server():
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        node.loop = loop
        loop.run_until_complete(node.start_websocket_server())
    
    ws_thread = threading.Thread(target=run_ws_server, daemon=True)
    ws_thread.start()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        http_server.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
