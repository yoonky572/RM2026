#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
决策测试桥接服务器
连接Web前端和ROS服务
"""

from goal_distribute.srv import DecisionTest, DecisionTestRequest, DecisionTestResponse
from http.server import HTTPServer, BaseHTTPRequestHandler
from http import HTTPStatus
import errno
import json
import traceback
import urllib.parse
import rospy

class DecisionTestHandler(BaseHTTPRequestHandler):
    """HTTP请求处理器"""
    
    def send_json_response(self, code, payload):
        """统一返回JSON响应"""
        try:
            self.send_response(code)
            self.send_header('Content-type', 'application/json; charset=utf-8')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps(payload, ensure_ascii=False).encode('utf-8'))
        except BrokenPipeError:
            rospy.logwarn("客户端连接已断开，无法写入响应")
    
    def send_json_error(self, code, message):
        """返回JSON格式的错误响应，避免send_error的ASCII限制"""
        if not message:
            try:
                message = HTTPStatus(code).phrase
            except ValueError:
                message = "Error"
        self.send_json_response(code, {'success': False, 'error': message})

    # 覆盖父类的send_error，防止其使用latin-1编码
    def send_error(self, code, message=None, explain=None):
        self.send_json_error(code, message or explain)
    
    def do_OPTIONS(self):
        """处理CORS预检请求"""
        self.send_response(200)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.end_headers()
    
    def do_GET(self):
        """处理GET请求（返回前端页面）"""
        if self.path == '/' or self.path == '/index.html':
            self.send_response(200)
            self.send_header('Content-type', 'text/html; charset=utf-8')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            
            # 读取并返回HTML文件
            try:
                import os
                html_path = os.path.join(os.path.dirname(__file__), 'decision_test_frontend.html')
                with open(html_path, 'r', encoding='utf-8') as f:
                    self.wfile.write(f.read().encode('utf-8'))
            except Exception as e:
                self.wfile.write(f'<html><body>错误: {str(e)}</body></html>'.encode('utf-8'))
        else:
            self.send_response(404)
            self.end_headers()
    
    def parse_request_json(self):
        content_length = int(self.headers.get('Content-Length', 0))
        if content_length <= 0:
            raise ValueError("请求体为空")
        post_data = self.rfile.read(content_length)
        return json.loads(post_data.decode('utf-8'))

    def call_decision_service(self, payload):
        if rospy.is_shutdown():
            raise rospy.ROSInterruptException("rospy shutdown")
        
        # 服务代理已在服务器启动时创建
        decision_service = self.server.decision_service
        if decision_service is None:
            raise rospy.ROSException("decision service unavailable")
        
        req = DecisionTestRequest()
        req.self_hp = int(payload.get('self_hp', 600))
        req.self_ammo = int(payload.get('self_ammo', 500))
        req.ally_base_hp = int(payload.get('ally_base_hp', 600))
        req.ally_outpost_hp = int(payload.get('ally_outpost_hp', 600))
        req.enemy_base_hp = int(payload.get('enemy_base_hp', 600))
        req.enemy_outpost_hp = int(payload.get('enemy_outpost_hp', 600))
        req.enemy_visible = bool(payload.get('enemy_visible', False))
        req.enemy_x = float(payload.get('enemy_x', 0.0))
        req.enemy_y = float(payload.get('enemy_y', 0.0))
        req.enemy_z = float(payload.get('enemy_z', 0.0))
        team_color = str(payload.get('team_color', 'red')).strip().lower()
        if team_color not in ('red', 'blue'):
            team_color = 'red'
        req.team_color = team_color
        
        return decision_service(req)

    def do_POST(self):
        """处理POST请求（决策测试）"""
        if self.path != '/api/decision':
            self.send_json_error(404, "未知接口")
            return
        
        try:
            data = self.parse_request_json()
            resp = self.call_decision_service(data)
            result = {
                'success': resp.success,
                'goal_id': int(resp.goal_id),
                'strategy': resp.strategy,
                'reason': resp.reason,
                'confidence': float(resp.confidence),
                'goal_pose': {
                    'x': float(resp.goal_pose.pose.position.x),
                    'y': float(resp.goal_pose.pose.position.y),
                    'z': float(resp.goal_pose.pose.position.z)
                }
            }
            self.send_json_response(200, result)
        except ValueError as e:
            self.send_json_error(400, f"请求格式错误: {str(e)}")
        except json.JSONDecodeError:
            self.send_json_error(400, "JSON解析失败")
        except rospy.ROSInterruptException:
            self.send_json_error(503, "ROS节点已退出，请重新启动决策测试服务器")
        except rospy.ROSException as e:
            self.send_json_error(503, f"ROS服务不可用: {str(e)}")
        except Exception as e:
            rospy.logerr("决策请求处理异常: %s\n%s", str(e), traceback.format_exc())
            self.send_json_error(500, "服务器内部错误，请查看日志")

def create_server(port, retries):
    """创建HTTP服务器，必要时自动尝试后续端口"""
    for attempt in range(retries + 1):
        try:
            server_address = ('', port)
            httpd = HTTPServer(server_address, DecisionTestHandler)
            httpd.decision_service = None
            actual_port = httpd.server_port
            if actual_port != port:
                rospy.logwarn(f"端口被占用，自动使用系统分配端口: {actual_port}")
            return httpd, actual_port
        except OSError as e:
            if e.errno == errno.EADDRINUSE and attempt < retries:
                rospy.logwarn(f"端口 {port} 已被占用，尝试使用 {port + 1}")
                port += 1
                continue
            raise
    raise RuntimeError("无法找到可用端口")


def run_server(port=8080, retries=5, service_name='/goal_distribute/decision_test'):
    """运行HTTP服务器"""
    httpd, actual_port = create_server(port, retries)

    rospy.loginfo(f"等待ROS服务: {service_name}")
    rospy.wait_for_service(service_name)
    httpd.decision_service = rospy.ServiceProxy(service_name, DecisionTest)
    
    rospy.loginfo(f"决策测试桥接服务器已启动: http://localhost:{actual_port}")
    
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        rospy.loginfo("服务器关闭")
    finally:
        httpd.server_close()

if __name__ == '__main__':
    rospy.init_node('decision_test_bridge', anonymous=True)
    port = rospy.get_param('~port', 8080)
    retries = rospy.get_param('~port_retry', 5)
    service_name = rospy.get_param('~service_name', '/goal_distribute/decision_test')
    run_server(port, retries, service_name)

