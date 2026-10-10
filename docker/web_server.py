#!/usr/bin/env python3
"""Local browser viewer: SDF visuals + live Gazebo link poses, no server-side 3D renderer."""
# 这个节点一边把 Gazebo/URDF 场景整理成浏览器可读的数据，一边提供实时状态和少量仿真控制接口。
import copy
import json
import mimetypes
import os
import subprocess
import sys
import tempfile
import threading
import time
import uuid
import xml.etree.ElementTree as ET
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import quote, unquote, urlsplit

import rospkg
import rospy
from gazebo_msgs.msg import LinkStates
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from std_srvs.srv import Empty
from flygripper_msgs.msg import ActuatorState

WEB = Path(__file__).resolve().parent / 'web'
# 回调线程写 STATE，HTTP 请求线程读 STATE；锁用于避免读到一半更新的数据。
LOCK = threading.Lock()
STATE = dict(links={}, sim_time=0, received=0, odom=None, actuator=None, grasp=None,
             paused=False, session_id=uuid.uuid4().hex, mission=dict(state='idle', completed=[]))
ASSETS = {}
DAE_DATA = {}
CHILD = None
COMMAND_LOCK = threading.Lock()


def numbers(text, default):
    # URDF/SDF 中的向量通常以空格分隔；缺省或空字符串时复制默认值。
    return [float(v) for v in text.split()] if text else list(default)


def asset_url(path):
    # 注册模型资源并生成浏览器 URL；所有资源路径都限制在对应模型目录内。
    path = path.resolve()
    if not path.is_file():
        raise FileNotFoundError(path)
    # Keep model-relative texture paths, confined to a registered model tree.
    parent = next((p for p in path.parents if p.name == 'models'), path.parent)
    key = next((k for k, v in ASSETS.items() if v == parent), str(len(ASSETS)))
    ASSETS[key] = parent
    url = '/assets/' + key + '/' + quote(str(path.relative_to(parent)))
    if path.suffix.lower() == '.dae' and path not in DAE_DATA:
        # COLLADA 网格的纹理路径可能相对文件位置或模型目录；解析后改写为可安全访问的相对路径。
        ns = 'http://www.collada.org/2005/11/COLLADASchema'
        ET.register_namespace('', ns)
        document = ET.parse(str(path))
        for image in document.findall('.//{%s}library_images/{%s}image/{%s}init_from' % (ns, ns, ns)):
            texture = (path.parent / unquote(image.text)).resolve()
            if not texture.is_file() or parent not in texture.parents:
                matches = list(parent.rglob(Path(image.text).name))
                if len(matches) != 1:
                    raise FileNotFoundError('Cannot resolve texture ' + image.text)
                texture = matches[0]
            image.text = os.path.relpath(texture, path.parent)
        DAE_DATA[path] = ET.tostring(document.getroot(), encoding='utf-8', xml_declaration=True)
    return url


def scene_description():
    # 将机器人 URDF 转成 Gazebo 使用的 SDF，再读取机器人和世界模型的可视几何。
    package = Path(rospkg.RosPack().get_path('flygripper_description'))
    roots = [package / 'models', Path('/usr/share/gazebo-11/models')]

    def resolve(uri):
        # 只解析 ROS/Gazebo 的 model:// 和 package:// 资源，不接受任意文件路径。
        if uri.startswith(('model://', 'package://')):
            name = uri.split('://', 1)[1]
            if name.startswith('flygripper_description/'):
                return package / name.split('/', 1)[1]
            for root in roots:
                if (root / name).exists():
                    return root / name
        raise ValueError('Unsupported model URI: ' + uri)

    visuals = []
    def add_model(model, name):
        # 提取每个 link 的 visual 几何、位姿和颜色，供网页端 Three.js 绘制。
        for link in model.findall('link'):
            for visual in link.findall('visual'):
                geometry = visual.find('geometry')
                if geometry is None:
                    continue
                shape = list(geometry)[0]
                g = dict(type=shape.tag)
                if shape.tag == 'mesh':
                    g.update(url=asset_url(resolve(shape.findtext('uri'))),
                             scale=numbers(shape.findtext('scale'), [1, 1, 1]))
                elif shape.tag == 'box':
                    g['size'] = numbers(shape.findtext('size'), [1, 1, 1])
                elif shape.tag in ('sphere', 'cylinder'):
                    g['radius'] = float(shape.findtext('radius', '0.1'))
                    g['length'] = float(shape.findtext('length', '0.1'))
                else:
                    continue
                visuals.append(dict(link=name + '::' + link.attrib['name'], geometry=g,
                                    pose=numbers(visual.findtext('pose'), [0]*6),
                                    color=numbers(visual.findtext('material/diffuse'), [.55, .64, .72, 1])))

    urdf = rospy.get_param('/flygripper/robot_description')
    # 使用与 Gazebo 相同的 URDF→SDF 转换，确保固定关节合并后的 link 名称一致。
    with tempfile.NamedTemporaryFile(suffix='.urdf', mode='w') as f:
        f.write(urdf)
        f.flush()
        result = subprocess.run(['gz', 'sdf', '-p', f.name], capture_output=True, text=True, timeout=30, check=True)
    # Same URDF -> SDF conversion as Gazebo, including fixed-joint lumping.
    add_model(ET.fromstring(result.stdout).find('model'), 'flygripper')
    world = Path(rospy.get_param('/hiarm/world_file'))
    for inc in ET.parse(str(world)).findall('.//world/include'):
        uri = inc.findtext('uri')
        if uri in ('model://sun', 'model://ground_plane'):
            continue
        add_model(ET.parse(str(resolve(uri + '/model.sdf'))).find('model'), inc.findtext('name'))
    for model in ET.parse(str(world)).findall('.//world/model'):
        add_model(model, model.attrib['name'])
    tasks = []
    # 读取规划器参数，仅向页面提供每阶段航点和形变信息用于可视化。
    count = rospy.get_param('/flygripper/real/task_num')
    for i in range(count):
        task = rospy.get_param(f'/flygripper/real/task_{i}')
        points = [task[f'plan_point_{j}'] for j in range(task['plan_point_num'])]
        tasks.append(dict(index=i, points=points, action=task['morph_action'], automatic=task['morph_auto']))
    return dict(visuals=visuals, tasks=tasks, profile=rospy.get_param('/hiarm/mission'),
                world=world.name, warning='物体已按抓取航点与夹爪偏移布置；实际抓取结果以插件反馈及物体运动为准。' if world.name == 'matched.world' else '短演示场景，物体未按原任务航点布置。')


def link_states(msg):
    # Gazebo 位姿更新频率可能很高，这里最多每 0.05 秒复制一次完整 link 状态。
    now = time.monotonic()
    if now - STATE['received'] < .05:
        return
    links = {name: [p.position.x, p.position.y, p.position.z,
                    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]
             for name, p in zip(msg.name, msg.pose)}
    with LOCK:
        STATE.update(links=links, sim_time=rospy.Time.now().to_sec(), received=now)


def odom(msg):
    # 里程计只用于页面显示飞机位置和速度，不作为控制输入。
    p, v = msg.pose.pose.position, msg.twist.twist.linear
    with LOCK:
        STATE['odom'] = dict(position=[p.x, p.y, p.z], velocity=[v.x, v.y, v.z])


def actuator(msg):
    # 保存机构当前位置与过载标志，网页轮询时一起返回。
    with LOCK:
        STATE['actuator'] = dict(position=msg.position, overload=msg.overload)


def grasp(msg):
    # 保存夹爪插件报告的附着状态。
    with LOCK:
        STATE['grasp'] = msg.data


def mission(msg):
    # 页面只需近期进度；完整采样和任务参数保留在 mission.json 报告中。
    data = json.loads(msg.data)
    # Send bounded UI state; full evidence stays in mission.json.
    data.pop('task_parameters', None)
    data.pop('object_samples', None)
    data.pop('scene_matching', None)
    data['events'] = data.get('events', [])[-12:]
    with LOCK:
        STATE['mission'] = data


def snapshot():
    # 复制共享状态后再补充数据新鲜度和当前演示模式，避免持锁访问 ROS 参数服务器。
    with LOCK:
        data = copy.deepcopy(STATE)
    data['age'] = time.monotonic() - data.pop('received')
    data['mode'] = rospy.get_param('/hiarm/demo_mode', 'manual')
    return data


class Handler(BaseHTTPRequestHandler):
    # 每个 HTTP 请求由独立线程处理，状态数据通过上面的锁共享。
    protocol_version = 'HTTP/1.1'

    def log_message(self, fmt, *args):
        if len(args) >= 2 and str(args[1]) == '200':
            return
        super().log_message(fmt, *args)

    def reply(self, data, content_type='application/json', status=200):
        # 统一生成 JSON/静态文件响应，并禁止浏览器猜测内容类型或缓存旧状态。
        if not isinstance(data, bytes):
            data = json.dumps(data, ensure_ascii=False).encode()
        self.send_response(status)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(data)))
        self.send_header('Cache-Control', 'no-cache')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        # GET 提供场景、状态、事件流和静态资源；资源解析后必须仍位于允许的目录内。
        path = unquote(urlsplit(self.path).path)
        try:
            if path == '/api/scene':
                return self.reply(SCENE)
            if path == '/api/state':
                return self.reply(snapshot())
            if path == '/api/events':
                # 以 Server-Sent Events 持续推送状态；客户端断开或 ROS 关闭时结束连接。
                self.close_connection = True
                self.send_response(200)
                self.send_header('Content-Type', 'text/event-stream')
                self.send_header('Cache-Control', 'no-cache')
                self.end_headers()
                self.connection.settimeout(5)
                while not rospy.is_shutdown():
                    self.wfile.write(('data: ' + json.dumps(snapshot()) + '\n\n').encode())
                    self.wfile.flush()
                    time.sleep(.05)
                return
            if path.startswith('/assets/'):
                _, _, key, tail = path.split('/', 3)
                root = ASSETS[key]
                file = (root / tail).resolve()
            else:
                root = WEB
                file = (root / (path.lstrip('/') or 'index.html')).resolve()
            if root not in file.parents or not file.is_file():
                return self.reply(dict(error='Not found'), status=404)
            data = DAE_DATA[file] if file in DAE_DATA else file.read_bytes()
            return self.reply(data, mimetypes.guess_type(file.name)[0] or 'application/octet-stream')
        except (BrokenPipeError, ConnectionResetError, TimeoutError):
            return
        except (KeyError, ValueError, FileNotFoundError):
            return self.reply(dict(error='Not found'), status=404)

    def do_POST(self):
        global CHILD
        # POST 只接受本机同源 JSON，并只开放白名单动作，不提供任意 ROS、shell 或文件操作。
        # Same-origin JSON only. Never expose arbitrary ROS topics, shell commands or filesystem writes.
        host = self.headers.get('Host', '')
        if (urlsplit('http://' + host).hostname not in ('localhost', '127.0.0.1', '::1') or
                self.headers.get('Origin') not in (None, 'http://' + host) or
                self.headers.get('Content-Type') != 'application/json'):
            return self.reply(dict(error='Local same-origin JSON requests required'), status=403)
        if self.path != '/api/command':
            return self.reply(dict(error='Not found'), status=404)
        try:
            length = int(self.headers.get('Content-Length', 0))
            if not 0 < length <= 1024:
                raise ValueError('Invalid request size')
            action = json.loads(self.rfile.read(length))['action']
            if not rospy.get_param('/use_sim_time', False):
                raise ValueError('Simulation only')
            with COMMAND_LOCK:
                if action == 'mission':
                    # 网页启动任务前确认没有重复任务，且仿真处于原始任务的初始状态。
                    if CHILD and CHILD.poll() is None:
                        raise ValueError('任务已在运行')
                    if SCENE['profile'] != 'original' or snapshot()['mode'] != 'manual':
                        raise ValueError('请先执行 ./docker/sim.sh start 或 restart，从初始状态启动原任务')
                    if snapshot()['age'] > 2:
                        raise ValueError('仿真数据尚未就绪')
                    log = open('/ws/docker/logs/mission.log', 'w')
                    CHILD = subprocess.Popen([sys.executable, '-u', '/ws/docker/mission.py'], stdout=log, stderr=subprocess.STDOUT)
                    log.close()
                elif action in ('pause', 'resume'):
                    # 暂停/恢复只调用 Gazebo 物理服务，并同步更新页面状态。
                    service = '/gazebo/' + ('pause_physics' if action == 'pause' else 'unpause_physics')
                    rospy.wait_for_service(service, timeout=3)
                    rospy.ServiceProxy(service, Empty)()
                    with LOCK:
                        STATE['paused'] = action == 'pause'
                else:
                    raise ValueError('Unknown action')
            self.reply(dict(ok=True))
        except Exception as exc:
            self.reply(dict(error=str(exc)), status=409)


if __name__ == '__main__':
    # 等待机器人模型和规划器参数就绪后，构建一次场景描述并订阅实时状态。
    rospy.init_node('hiarm_web', disable_signals=False)
    deadline = time.monotonic() + 120
    while not rospy.is_shutdown() and not all(rospy.has_param(p) for p in
            ('/flygripper/robot_description', '/flygripper/real/task_num')):
        if time.monotonic() > deadline:
            raise RuntimeError('Timed out waiting for robot description and planner')
        time.sleep(.2)
    SCENE = scene_description()
    subscribers = [rospy.Subscriber('/gazebo/link_states', LinkStates, link_states, queue_size=1),
                   rospy.Subscriber('/flygripper/odom', Odometry, odom, queue_size=1),
                   rospy.Subscriber('/flygripper/actuator_state', ActuatorState, actuator, queue_size=1),
                   rospy.Subscriber('/flygripper/grasp_status', Bool, grasp, queue_size=1),
                   rospy.Subscriber('/hiarm/mission_status', String, mission, queue_size=1)]
    server = ThreadingHTTPServer(('0.0.0.0', 8080), Handler)
    server.timeout = .5
    print(f'WebGL ready: {len(SCENE["visuals"])} visuals, {len(SCENE["tasks"])} stages', flush=True)
    try:
        # 手动逐个处理请求，便于 ROS 关闭时退出并清理服务器和任务子进程。
        while not rospy.is_shutdown():
            server.handle_request()
    finally:
        server.server_close()
        if CHILD and CHILD.poll() is None:
            CHILD.terminate()
