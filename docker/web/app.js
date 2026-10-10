import * as THREE from 'three';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';
import {ColladaLoader} from 'three/addons/loaders/ColladaLoader.js';
import {STLLoader} from 'three/addons/loaders/STLLoader.js';

const $ = id => document.getElementById(id);
const diagnostics = window.hiarmViewer = {ready:false, errors:[], frames:0, linkCount:0};
function fail(message) { diagnostics.errors.push(message); $('loading').hidden=false; $('loading').className='loading error'; $('loading').textContent=message; }
window.addEventListener('unhandledrejection', e => fail(String(e.reason)));
window.addEventListener('error', e => fail(e.message));
const host=$('canvas'), scene=new THREE.Scene();
scene.background=new THREE.Color(0xedf2f7);
THREE.Object3D.DEFAULT_UP.set(0,0,1);
const camera=new THREE.PerspectiveCamera(42,1,.01,200);
camera.up.set(0,0,1);
const renderer=new THREE.WebGLRenderer({antialias:true, powerPreference:'high-performance'});
renderer.setPixelRatio(Math.min(devicePixelRatio,2));
renderer.outputColorSpace=THREE.SRGBColorSpace;
host.append(renderer.domElement);
renderer.domElement.addEventListener('webglcontextlost', e => {e.preventDefault();fail('图形上下文已丢失，请刷新页面重新连接。');});
const gl=renderer.getContext(), debug=gl.getExtension('WEBGL_debug_renderer_info');
diagnostics.renderer=debug?gl.getParameter(debug.UNMASKED_RENDERER_WEBGL):gl.getParameter(gl.RENDERER);
$('renderer').textContent=diagnostics.renderer;
const controls=new OrbitControls(camera,renderer.domElement);
controls.enableDamping=true; controls.minDistance=.25; controls.maxDistance=40;
controls.maxPolarAngle=Math.PI*.49;
scene.add(new THREE.HemisphereLight(0xffffff,0x6f8190,1.6));
const sun=new THREE.DirectionalLight(0xffffff,1.7);sun.position.set(-3,-4,10);scene.add(sun);
const floor=new THREE.Mesh(new THREE.PlaneGeometry(80,80),new THREE.MeshStandardMaterial({color:0xe6edf3,roughness:1}));
floor.position.z=-.006;scene.add(floor);
const grid=new THREE.GridHelper(24,48,0xaabfcf,0xd2dde6);grid.rotation.x=Math.PI/2;grid.position.set(4,0,0);scene.add(grid);
const axes=new THREE.AxesHelper(.5);axes.position.z=.01;scene.add(axes);
const groups=new Map(), markers=[], trail=[], trailGeometry=new THREE.BufferGeometry();
const trailLine=new THREE.Line(trailGeometry,new THREE.LineBasicMaterial({color:0x008a87,transparent:true,opacity:.8}));scene.add(trailLine);
let latest=null, lastReceived=0, following=true, lastTarget=new THREE.Vector3(0,0,.5), frameCount=0, frameStart=performance.now(), lastTrailTime=-1;
camera.position.set(1.7,-2.2,1.5);controls.target.copy(lastTarget);
new ResizeObserver(()=>{const {width,height}=host.getBoundingClientRect();renderer.setSize(width,height);camera.aspect=width/height;camera.updateProjectionMatrix();}).observe(host);
const manager=new THREE.LoadingManager();
manager.onError=url=>fail('模型资源加载失败：'+url);
const dae=new ColladaLoader(manager), stl=new STLLoader(manager), cache=new Map();
async function loadMesh(url){
  if(!cache.has(url)) cache.set(url,(async()=>{
    if(url.toLowerCase().endsWith('.stl')){
      const geometry=await stl.loadAsync(url); geometry.computeVertexNormals();
      return new THREE.Mesh(geometry,new THREE.MeshStandardMaterial({color:0x43566b,roughness:.6,metalness:.15}));
    }
    const model=await dae.loadAsync(url), wrapper=new THREE.Group();
    // This legacy MeshLab asset declares transparency=0, which ColladaLoader reads
    // as opacity=0. The physical can is opaque. Scope the compatibility fix to this
    // known asset so genuinely transparent materials in other models stay intact.
    if(url.endsWith('/coke_can/meshes/coke_can.dae')){
      model.scene.traverse(node=>{
        if(!node.isMesh)return;
        for(const material of [node.material].flat()){
          material.opacity=1;material.transparent=false;material.depthWrite=true;material.needsUpdate=true;
        }
      });
    }
    // Keep raw mesh axes, as Gazebo does for these models (some declare Y_UP despite Z-height vertices).
    model.scene.rotation.set(0,0,0);wrapper.add(model.scene);return wrapper;
  })());
  return (await cache.get(url)).clone(true);
}
async function addVisual(v){
  let link=groups.get(v.link);
  if(!link){link=new THREE.Group();link.visible=false;groups.set(v.link,link);scene.add(link);}
  const local=new THREE.Group(), p=v.pose, g=v.geometry;
  local.position.fromArray(p);local.rotation.set(p[3],p[4],p[5],'ZYX');link.add(local);
  let object;
  if(g.type==='mesh'){object=await loadMesh(g.url);object.scale.multiply(new THREE.Vector3(...g.scale));}
  else{
    const material=new THREE.MeshStandardMaterial({color:new THREE.Color(...v.color.slice(0,3)),roughness:.65});
    let geometry;
    if(g.type==='box')geometry=new THREE.BoxGeometry(...g.size);
    if(g.type==='sphere')geometry=new THREE.SphereGeometry(g.radius,24,16);
    if(g.type==='cylinder'){geometry=new THREE.CylinderGeometry(g.radius,g.radius,g.length,24);geometry.rotateX(Math.PI/2);}
    object=new THREE.Mesh(geometry,material);
  }
  local.add(object);
}
const stageNames=['到达起点','抓取 1 · 易拉罐','释放 1 · 易拉罐','抓取 2 · 球体','释放 2 · 球体','抓取 3 · 瓶子','释放 3 · 瓶子','抓取 4 · 方块','释放 4 · 方块','收拢 · 垃圾箱'];
const description=await fetch('/api/scene').then(r=>{if(!r.ok)throw Error('无法读取场景');return r.json();});
$('profile').textContent=`${description.world} · ${description.tasks.length} 个阶段 · 实时关节姿态`;
$('scene-note').textContent=description.warning;
if(description.profile!=='original')document.querySelector('h2').textContent='短轨迹检查';
for(const task of description.tasks){
  const li=document.createElement('li');li.id=`stage-${task.index}`;
  li.innerHTML=`<span class="stage-index">${String(task.index+1).padStart(2,'0')}</span><div><span class="stage-name"></span><span class="stage-detail"></span></div>`;
  li.querySelector('.stage-name').textContent=description.profile==='original'?stageNames[task.index]:'飞往 (0.6, 0, 1.0)';
  li.querySelector('.stage-detail').textContent=({0:'保持形态',1:'展开夹爪',2:'收拢夹爪'}[task.action])+(task.automatic?' · 自动':' · 虚拟遥控触发');
  $('stages').append(li);
  const endpoint=task.points.at(-1), ring=new THREE.Mesh(new THREE.TorusGeometry(.095,.009,8,32),new THREE.MeshBasicMaterial({color:0xcb7435}));
  ring.position.set(endpoint.x,endpoint.y,endpoint.z);scene.add(ring);markers.push(ring);
  const path=[new THREE.Vector3(endpoint.x,endpoint.y,.005),ring.position.clone()];
  scene.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(path),new THREE.LineDashedMaterial({color:0xb3c1cc,dashSize:.05,gapSize:.05})).computeLineDistances());
}
await Promise.all(description.visuals.map(addVisual));
diagnostics.ready=true;diagnostics.visualCount=description.visuals.length;
$('model-count').textContent=`${description.visuals.length} 个模型部件 · ${groups.size} 个可见连杆`;
if(!diagnostics.errors.length)$('loading').hidden=true;
function updateUI(s){
  const m=s.mission, stateNames={idle:'等待启动',starting:'准备起飞',takeoff:'正在起飞并稳定悬停',running:'正在执行任务',completed:'十阶段轨迹与夹爪动作已完成',failed:'任务中断'};
  $('progress').textContent=stateNames[m.state]||m.state;
  $('mission').disabled=description.profile!=='original'||s.mode!=='manual'||s.age>2||s.paused||['starting','takeoff','running'].includes(m.state);
  $('pause').disabled=false;$('pause').textContent=s.paused?'继续仿真':'暂停仿真';
  $('time').textContent=s.sim_time.toFixed(1);
  if(s.odom){const p=s.odom.position;$('altitude').textContent=p[2].toFixed(2);$('speed').textContent=Math.hypot(...s.odom.velocity).toFixed(2);$('position').textContent='位置 / m：'+p.map(v=>v.toFixed(3)).join(', ');}
  if(s.actuator)$('actuator').textContent=s.actuator.position.toFixed(2);
  $('grasp').textContent=s.grasp===null?'未收到抓取事件':s.grasp?'已检测到物体连接':'已释放物体';
  if(m.error){$('message').textContent=m.error;$('message').className='error';}
  else if(m.state==='completed')$('message').textContent='任务动作已完成。'+(m.all_transfers_verified?'四次抓取、搬运和按计划释放均已通过。'+(m.can_motion_validation ? (m.grasp_and_stability_verified?'易拉罐抓取稳定性检查通过。':'易拉罐抓取稳定性检查未通过。') : ''):m.physical_grasp_verified?'记录到抓取反馈，四次完整搬运尚未全部通过。':'未记录到实际物体抓取。')+' 重启仿真可重新演示。';
  for(const task of description.tasks){const li=$(`stage-${task.index}`);li.className=m.completed?.includes(task.index)?'done':m.stage===task.index?'active':'';markers[task.index].material.color.setHex(li.className==='done'?0x008a87:li.className==='active'?0x15304b:0xcb7435);}
}
const stream=new EventSource('/api/events');
let sessionId;
stream.onmessage=event=>{
  const data=JSON.parse(event.data);
  if(sessionId && data.session_id!==sessionId){stream.close();location.reload();return;}
  sessionId=data.session_id;latest=data;lastReceived=performance.now();
  diagnostics.linkCount=Object.keys(latest.links).length;diagnostics.lastState=latest;updateUI(latest);
};
stream.onerror=()=>{$('connection').textContent='连接中断，正在重连…';$('connection').className='connection';$('mission').disabled=true;$('pause').disabled=true;};
async function command(action){
  try{const response=await fetch('/api/command',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({action})});const result=await response.json();if(!response.ok)throw Error(result.error);$('message').className='';$('message').textContent=action==='mission'?'开始起飞，随后执行原始十阶段任务。':'操作已发送。';}
  catch(error){$('message').className='error';$('message').textContent=error.message;}
}
$('mission').onclick=()=>{$('mission').disabled=true;command('mission');};
$('pause').onclick=()=>command(latest?.paused?'resume':'pause');
$('follow').onclick=()=>{following=true;$('follow').setAttribute('aria-pressed','true');if(latest?.odom){lastTarget.fromArray(latest.odom.position);controls.target.copy(lastTarget);camera.position.copy(lastTarget).add(new THREE.Vector3(1.7,-2.2,1.3));}};
$('overview').onclick=()=>{following=false;$('follow').setAttribute('aria-pressed','false');controls.target.set(4,.4,.5);camera.position.set(10,-10,8);};
function frame(){
  requestAnimationFrame(frame);diagnostics.frames++;frameCount++;
  if(latest){
    for(const [name,p] of Object.entries(latest.links)){const group=groups.get(name);if(group){group.visible=true;group.position.fromArray(p);group.quaternion.fromArray(p.slice(3));}}
    if(latest.odom){
      const target=new THREE.Vector3(...latest.odom.position);
      if(following){const delta=target.clone().sub(lastTarget);camera.position.add(delta);controls.target.add(delta);lastTarget.copy(target);}
      if(latest.sim_time!==lastTrailTime&&(!trail.length||trail.at(-1).distanceTo(target)>.015)){trail.push(target);if(trail.length>2500)trail.shift();trailGeometry.setFromPoints(trail);lastTrailTime=latest.sim_time;}
    }
    const fresh=performance.now()-lastReceived<2000;
    $('connection').textContent=!fresh?'连接中断，正在重连…':latest.paused?'仿真已暂停':latest.age>2?'等待仿真数据…':'● 仿真实时连接';
    $('connection').className='connection'+(fresh&&latest.age<2?' live':'');
    if(!fresh){$('mission').disabled=true;$('pause').disabled=true;}
  }
  const now=performance.now();if(now-frameStart>1000){diagnostics.fps=frameCount*1000/(now-frameStart);$('fps').textContent=`${Math.round(diagnostics.fps)} FPS · 本机渲染`;frameCount=0;frameStart=now;}
  controls.update();renderer.render(scene,camera);
}
// Read-only model inspection and camera focus for visual diagnostics; no ROS commands.
diagnostics.inspectModel=name=>{
  const group=groups.get(name);if(!group)return null;
  const box=new THREE.Box3().setFromObject(group), materials=[];
  group.traverse(node=>{if(node.isMesh)for(const m of [node.material].flat())materials.push({opacity:m.opacity,transparent:m.transparent,textureLoaded:!!m.map?.image?.width});});
  return {visible:group.visible,size:box.getSize(new THREE.Vector3()).toArray(),center:box.getCenter(new THREE.Vector3()).toArray(),materials};
};
diagnostics.focusModel=name=>{
  const group=groups.get(name);if(!group)return false;
  const box=new THREE.Box3().setFromObject(group), center=box.getCenter(new THREE.Vector3());
  following=false;$('follow').setAttribute('aria-pressed','false');controls.target.copy(center);
  camera.position.copy(center).add(new THREE.Vector3(.28,-.36,.24));controls.update();return true;
};
frame();
