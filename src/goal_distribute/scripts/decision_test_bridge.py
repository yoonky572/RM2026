#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
决策测试桥接服务器
连接Web前端和ROS服务
"""

from goal_distribute.srv import DecisionTest, DecisionTestRequest
from http.server import HTTPServer, BaseHTTPRequestHandler
from http import HTTPStatus
import errno
import json
import os
import threading
import traceback
import rospkg
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
        """处理GET请求（返回前端页面或规则接口）"""
        if self.path == '/' or self.path == '/index.html':
            self.send_response(200)
            self.send_header('Content-type', 'text/html; charset=utf-8')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            
            # 读取并返回HTML文件
            try:
                html_path = os.path.join(os.path.dirname(__file__), 'decision_test_frontend.html')
                with open(html_path, 'r', encoding='utf-8') as f:
                    self.wfile.write(f.read().encode('utf-8'))
            except Exception as e:
                self.wfile.write(f'<html><body>错误: {str(e)}</body></html>'.encode('utf-8'))
        elif self.path == '/api/rules':
            self.handle_rules_get()
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
        """处理POST请求（决策测试或规则新增）"""
        if self.path == '/api/decision':
            self.handle_decision_post()
            return
        if self.path == '/api/rules':
            self.handle_rules_post()
            return
        self.send_json_error(404, "未知接口")

    def do_DELETE(self):
        """处理DELETE请求（删除规则）"""
        if self.path == '/api/rules':
            self.handle_rules_delete()
            return
        self.send_json_error(404, "未知接口")

    def handle_decision_post(self):
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

    def get_rules_file_path(self):
        path = getattr(self.server, 'json_rules_file', None)
        if not path:
            raise RuntimeError("服务器未配置规则文件路径")
        return path

    def load_rules_config(self):
        path = self.get_rules_file_path()
        if not os.path.exists(path):
            return {
                'version': '1.0',
                'description': '自动生成的规则配置',
                'rules': [],
                'weights': {}
            }
        with open(path, 'r', encoding='utf-8') as f:
            data = json.load(f)
        if 'rules' not in data or not isinstance(data['rules'], list):
            data['rules'] = []
        return data

    def save_rules_config(self, config):
        path = self.get_rules_file_path()
        directory = os.path.dirname(path)
        if directory and not os.path.exists(directory):
            os.makedirs(directory, exist_ok=True)
        lock = getattr(self.server, 'rules_file_lock', None)
        if lock:
            lock.acquire()
        try:
            with open(path, 'w', encoding='utf-8') as f:
                json.dump(config, f, ensure_ascii=False, indent=2)
                f.write('\n')
        finally:
            if lock:
                lock.release()

    def normalize_rule_payload(self, payload):
        if not isinstance(payload, dict):
            raise ValueError("规则数据格式错误")
        name = str(payload.get('name', '')).strip()
        if not name:
            raise ValueError("规则名称不能为空")
        priority = int(payload.get('priority', 0))
        conditions = payload.get('conditions', {})
        if not isinstance(conditions, dict):
            raise ValueError("conditions 必须是对象")
        normalized_conditions = {}
        for key, value in conditions.items():
            if key == 'enemy_visible':
                normalized_conditions[key] = bool(value)
                continue
            if not isinstance(value, dict):
                raise ValueError(f"{key} 条件格式错误")
            operator = value.get('operator')
            if operator not in ('lt', 'lte', 'gt', 'gte', 'eq', 'ne'):
                raise ValueError(f"{key} 条件的操作符非法")
            try:
                threshold = float(value.get('value'))
            except (TypeError, ValueError):
                raise ValueError(f"{key} 条件的阈值非法")
            normalized_conditions[key] = {
                'operator': operator,
                'value': threshold
            }

        action = payload.get('action', {})
        if not isinstance(action, dict):
            raise ValueError("action 必须是对象")
        strategy = str(action.get('strategy', '')).strip()
        if not strategy:
            raise ValueError("strategy 不能为空")
        goal_selection = action.get('goal_selection', {})
        if not isinstance(goal_selection, dict) or (
            not goal_selection.get('type') and 'goal_id' not in goal_selection
        ):
            raise ValueError("goal_selection 至少需要 type 或 goal_id")
        confidence = float(action.get('confidence', 0.0))
        if confidence < 0.0 or confidence > 1.0:
            raise ValueError("confidence 需在 [0,1]")
        reason = str(action.get('reason', '')).strip()
        if not reason:
            raise ValueError("reason 不能为空")

        normalized_rule = {
            'name': name,
            'priority': priority,
            'conditions': normalized_conditions,
            'action': {
                'strategy': strategy,
                'goal_selection': goal_selection,
                'confidence': confidence,
                'reason': reason
            }
        }
        return normalized_rule

    def handle_rules_get(self):
        try:
            config = self.load_rules_config()
            summary = [
                {
                    'name': rule.get('name'),
                    'priority': rule.get('priority'),
                    'strategy': (rule.get('action') or {}).get('strategy'),
                    'conditions': rule.get('conditions', {})
                }
                for rule in config.get('rules', [])
            ]
            payload = {
                'success': True,
                'version': config.get('version', 'unknown'),
                'rules': summary,
                'rule_count': len(summary)
            }
            self.send_json_response(200, payload)
        except FileNotFoundError:
            self.send_json_error(404, "未找到规则文件")
        except json.JSONDecodeError:
            self.send_json_error(500, "规则文件解析失败")
        except Exception as e:
            rospy.logerr("规则查询错误: %s", str(e))
            self.send_json_error(500, "无法读取规则文件")

    def handle_rules_post(self):
        try:
            payload = self.parse_request_json()
            new_rule = self.normalize_rule_payload(payload)
            config = self.load_rules_config()
            rules = config.get('rules', [])
            if any(rule.get('name') == new_rule['name'] for rule in rules):
                raise ValueError("已存在同名规则，请更换名称")
            rules.append(new_rule)
            rules.sort(key=lambda r: int(r.get('priority', 0)), reverse=True)
            config['rules'] = rules
            self.save_rules_config(config)
            response = {
                'success': True,
                'message': f"规则 {new_rule['name']} 已添加",
                'rule_count': len(rules)
            }
            self.send_json_response(200, response)
        except ValueError as e:
            self.send_json_error(400, str(e))
        except json.JSONDecodeError:
            self.send_json_error(400, "JSON解析失败")
        except Exception as e:
            rospy.logerr("规则写入错误: %s\n%s", str(e), traceback.format_exc())
            self.send_json_error(500, "无法写入规则文件")

    def handle_rules_delete(self):
        try:
            payload = self.parse_request_json()
            rule_name = str(payload.get('name', '')).strip()
            if not rule_name:
                raise ValueError("必须提供要删除的规则名称")
            config = self.load_rules_config()
            rules = config.get('rules', [])
            new_rules = [rule for rule in rules if rule.get('name') != rule_name]
            if len(new_rules) == len(rules):
                raise ValueError(f"未找到名称为 {rule_name} 的规则")
            config['rules'] = new_rules
            self.save_rules_config(config)
            response = {
                'success': True,
                'message': f"规则 {rule_name} 已删除",
                'rule_count': len(new_rules)
            }
            self.send_json_response(200, response)
        except ValueError as e:
            self.send_json_error(400, str(e))
        except json.JSONDecodeError:
            self.send_json_error(400, "JSON解析失败")
        except Exception as e:
            rospy.logerr("规则删除错误: %s\n%s", str(e), traceback.format_exc())
            self.send_json_error(500, "无法删除规则")

def create_server(port, retries):
    """创建HTTP服务器，必要时自动尝试后续端口"""
    for attempt in range(retries + 1):
        try:
            server_address = ('', port)
            httpd = HTTPServer(server_address, DecisionTestHandler)
            httpd.decision_service = None
            httpd.json_rules_file = None
            httpd.rules_file_lock = threading.Lock()
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

    json_rules_param = rospy.get_param('~json_rules_file', '')
    if not json_rules_param:
        try:
            rospack = rospkg.RosPack()
            package_path = rospack.get_path('goal_distribute')
            json_rules_param = os.path.join(package_path, 'config', 'decision_rules.json')
        except rospkg.ResourceNotFound:
            rospy.logwarn("未找到 goal_distribute 包路径，规则管理接口将不可用")
            json_rules_param = ''
    httpd.json_rules_file = os.path.abspath(json_rules_param) if json_rules_param else None
    
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

