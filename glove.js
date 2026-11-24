'use strict';
const model = window.gloveModel;
const ui = Object.fromEntries(['hand','status','connect','zero'].map(id=>[id, document.getElementById(id)]));
const uuid = n => 'a5a5000'+n+'-0c0f-4fee-9a5a-5f2e9c1e0d01';
const windowSize = 14;
const history = [];
const pose = Array.from({length:5},()=>[0,0,0,0]);
let neutral = [1,0,0,0];
let zero = pose.map(q=>q.slice());
try {
  const saved = JSON.parse(localStorage.getItem('surglove-reference'));
  if (saved) ({zero,neutral}=saved);
} catch {}
let attitude = [1,0,0,0], timestamp, fitted, lastFit = 0, lastFrame = 0;
let device, command, heartbeat, connected = false, session = 0;
const message = text => { ui.status.textContent = text; };
const rad = Math.PI/180;
const dot = (a,b) => a.reduce((s,x,i)=>s+x*b[i],0);
const cross = (a,b) => [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
const mul = ([a,b,c,d],[e,f,g,h]) => [a*e-b*f-c*g-d*h,a*f+b*e+c*h-d*g,a*g-b*h+c*e+d*f,a*h+b*g-c*f+d*e];
const inverse = ([w,x,y,z])=>[w,-x,-y,-z];
const rotate = (q,v)=>mul(mul(q,[0,...v]),inverse(q)).slice(1);
function orientation(frame, time) {
  const previous = timestamp;
  timestamp = time;
  if (previous === undefined) return;
  const dt = Math.min(.05, ((time-previous)>>>0)/1e6);
  const a = [12,14,16].map(i=>frame.getInt16(i,true));
  const g = [18,20,22].map(i=>frame.getInt16(i,true)*rad/16.4);
  const norm = Math.hypot(...a);
  if (norm > 1600 && norm < 2500) {
    const correction = cross(a.map(x=>x/norm),rotate(inverse(attitude),[0,0,1]));
    g.forEach((_,i)=>g[i]+=1.6*correction[i]);
  }
  const change = mul(attitude,[0,...g]);
  const next = attitude.map((x,i)=>x+change[i]*dt/2);
  attitude = next.map(x=>x/Math.hypot(...next));
}
function magneticFit() {
  const mean = history.reduce((s,h)=>s+h.current,0)/windowSize;
  const weights = history.map(h=>h.current-mean);
  const energy = dot(weights,weights);
  if (energy < windowSize*0.0001) return false;
  let count = 0;
  for (let f=0; f<5; f++) {
    const field = [];
    for (let sensor=0; sensor<6; sensor++) {
      const k = (4-f)*6+sensor, b = k*3;
      const raw = [0,1,2].map(axis=>history.map(h=>h.field[b+axis]));
      if (raw.some(v=>v.includes(32767))) continue;
      let repeated = 0;
      for (let i=1;i<windowSize;i++) if(raw.every(v=>v[i]===v[i-1])) repeated++;
      if (repeated>(windowSize-1)/2) continue;
      const slope = raw.map(v=>dot(v,weights)*1600/(16384*energy));
      const matrix = model.transforms[k];
      const vector = [0,1,2].map(i=>dot(matrix.slice(3*i,3*i+3),slope));
      field.push([sensor,...vector]);
    }
    if (!field.length) continue;
    const scale = Math.max(1,field.reduce((n,v)=>n+Math.hypot(v[1],v[2],v[3]),0)/field.length);
    let best = Infinity, index = 0;
    const table = model.tables[f];
    for (let i=0;i<table.length;i+=22) {
      let error = 0;
      for (const [s,x,y,z] of field) {
        const j=i+4+s*3;
        error += (x-table[j])**2+(y-table[j+1])**2+(z-table[j+2])**2;
      }
      error /= field.length*scale*scale;
      if(fitted) error += .000035 * ((table[i]-pose[f][0])**2+(table[i+1]-pose[f][1])**2);
      error += .00007*table[i+3]**2;
      if(error<best){best=error; index=i;}
    }
    for(let j=0;j<4;j++) pose[f][j] += (fitted?.55:1)*(table[index+j]-pose[f][j]);
    count++;
  }
  return count===5;
}
function receive(frame) {
  if (frame.byteLength!==204 || frame.getUint8(0)!==0xa6) {
    message('BLE 프레임 크기 오류 · '+ frame.byteLength +' / 204 bytes');
    return;
  }
  orientation(frame,frame.getUint32(2,true));
  const field = Array.from({length:90},(_,i)=>frame.getInt16(24+i*2,true));
  history.push({current:frame.getInt16(10,true)/1000,field});
  if(history.length>windowSize) history.shift();
  lastFrame = performance.now();
  if(history.length===windowSize && lastFrame-lastFit>=35) {
    lastFit=lastFrame;
    fitted=magneticFit();
    ui.zero.disabled=!fitted;
    message(fitted?'연결됨 · 실시간 추정 중':'연결됨 · 센서 및 코일 신호 확인 중');
  }
}
function connection(state, text) {
  connected=state;
  ui.connect.disabled=false;
  ui.connect.textContent=state?'연결 해제':'장갑 연결';
  message(text);
  if(!state){
    clearInterval(heartbeat);
    history.length=0;
    lastFrame=0;
    timestamp=undefined;
    fitted=false;
    ui.zero.disabled=true;
  }
}
window.androidStatus=(state,text)=>connection(state,text);
window.androidFrame=(base64)=>receive(new DataView(Uint8Array.from(atob(base64),c=>c.charCodeAt(0)).buffer));
async function connect() {
  if(window.AndroidGlove){
    if(connected) window.AndroidGlove.disconnect();
    else window.AndroidGlove.connect();
    return;
  }
  if(connected){device.gatt.disconnect(); return;}
  if(!navigator.bluetooth){message('Android Chrome에서 HTTPS 주소로 열어주세요.'); return;}
  const id=++session;
  ui.connect.disabled=true;
  message('장갑 검색 중…');
  try {
    const selected=await navigator.bluetooth.requestDevice({
      filters:[{services:[uuid(1)]},{name:'HandPose'},{name:'SURGlove'}],
      optionalServices:[uuid(1)]
    });
    device=selected;
    device.addEventListener('gattserverdisconnected',()=>{if(id===session) connection(false,'장갑 연결이 끊겼습니다.');},{once:true});
    const server=await device.gatt.connect();
    const service=await server.getPrimaryService(uuid(1));
    command=await service.getCharacteristic(uuid(3));
    const frames=await service.getCharacteristic(uuid(2));
    const keepalive=()=>command.writeValueWithoutResponse(Uint8Array.of(107));
    frames.addEventListener('characteristicvaluechanged',ev=>{if(id===session) receive(ev.target.value);});
    await frames.startNotifications();
    await keepalive();
    connection(true,'연결됨 · 센서 데이터 대기 중');
    heartbeat=setInterval(()=>keepalive().catch(()=>device.gatt.disconnect()),1000);
  } catch(error){
    if(device?.gatt.connected) device.gatt.disconnect();
    connection(false,'연결 실패 · '+error.message);
  }
}
ui.connect.onclick=connect;
ui.zero.onclick=()=>{
  zero=pose.map(q=>q.slice());
  neutral=inverse(attitude);
  try {localStorage.setItem('surglove-reference',JSON.stringify({zero,neutral}));} catch {}
};
window.addEventListener('keydown',ev=>{if(ev.key.toLowerCase()==='n'&&!ui.zero.disabled) ui.zero.click();});

const drawMesh = model.skin && createGloveMesh(ui.hand, model);
const displayedPose = pose.map(q=>q.map(()=>0));
let displayedRotation=[1,0,0,0], previousDraw;
function draw(now=performance.now()) {
  const dt=Math.min(50,previousDraw===undefined?16:now-previousDraw);
  previousDraw=now;
  const blend=1-Math.exp(-dt/32);
  for(let f=0;f<5;f++) for(let j=0;j<4;j++){
    const target=(fitted?pose[f][j]-zero[f][j]:0)*rad;
    const difference=target-displayedPose[f][j];
    displayedPose[f][j]=Math.abs(difference)<.001?target:displayedPose[f][j]+blend*difference;
  }
  const targetRotation=mul(neutral,attitude);
  const sign=dot(displayedRotation,targetRotation)<0?-1:1;
  displayedRotation=displayedRotation.map((v,i)=>v+blend*(sign*targetRotation[i]-v));
  const norm=Math.hypot(...displayedRotation);
  displayedRotation=displayedRotation.map(v=>v/norm);
  const canvas=ui.hand, w=canvas.clientWidth, h=canvas.clientHeight, dpr=Math.min(2,devicePixelRatio);
  if(canvas.width!==Math.round(w*dpr) || canvas.height!==Math.round(h*dpr)){
    canvas.width=Math.round(w*dpr); canvas.height=Math.round(h*dpr);
  }
  if(drawMesh && w && h){
    drawMesh(displayedPose,displayedRotation,w,h);
    requestAnimationFrame(draw);
    return;
  }
  const cx=canvas.getContext('2d');
  cx.setTransform(dpr,0,0,dpr,0,0);
  cx.clearRect(0,0,w,h);
  const scale=Math.min(w/205,h/225), rotation=displayedRotation;
  const items=[];
  const project=p=>{
    const [x,y,z]=rotate(rotation,[p[0],p[1]-48,p[2]]);
    const k=330/(330-z);
    return [w*.5+x*k*scale,h*.5-y*k*scale,z,k*scale];
  };
  function capsule(a,b,r,color){
    const u=project(a),v=project(b);
    items.push({z:(u[2]+v[2])/2,paint(){
      cx.strokeStyle=color;cx.lineWidth=2*r*(u[3]+v[3])/2;cx.lineCap='round';
      cx.beginPath();cx.moveTo(u[0],u[1]);cx.lineTo(v[0],v[1]);cx.stroke();
    }});
  }
  function shape(points, fill, hole) {
    const p=points.map(project), q=hole?.map(project);
    items.push({z:p.reduce((a,b)=>a+b[2],0)/p.length,paint(){
      cx.beginPath();
      for(const loop of q?[p,q]:[p]){
        cx.moveTo(loop[0][0],loop[0][1]);
        loop.slice(1).forEach(v=>cx.lineTo(v[0],v[1]));cx.closePath();
      }
      cx.fillStyle=fill;cx.fill('evenodd');
    }});
  }
  const palm=[[-28,-37],[-38,-9],[-41,11],[-33,35],[-20,51],[1,53],[24,48],[37,36],[38,8],[31,-37]];
  for(let z=-7;z<=7;z+=2) shape(palm.map(p=>[...p,z]),z===7?'#292d34':'#191c21');
  const circle=r=>Array.from({length:48},(_,i)=>{
    const t=i*Math.PI/24;
    return [model.coil[0]+r*Math.cos(t),model.coil[1]+r*Math.sin(t),9];
  });
  shape(circle(28),'#a76236',circle(16));
  for(let f=0;f<5;f++){
    let p=model.base[f], axes=model.frames[f].map((_,i)=>model.frames[f].map(row=>row[i]));
    const q=displayedPose[f];
    const mix=(a,b,t)=>a.map((x,i)=>x*Math.cos(t)+b[i]*Math.sin(t));
    [axes[0],axes[1]]=[mix(axes[0],axes[1],q[3]),mix(axes[1],axes[0],-q[3])];
    for(let j=0;j<3;j++){
      [axes[1],axes[2]]=[mix(axes[1],axes[2],-q[j]),mix(axes[2],axes[1],q[j])];
      const tip=p.map((x,i)=>x+axes[1][i]*model.lengths[f][j]);
      capsule(p,tip,(f===4?7:9)-j*.9,'#24282e');
      p=tip;
    }
  }
  items.sort((a,b)=>a.z-b.z).forEach(item=>item.paint());
  requestAnimationFrame(draw);
}
setInterval(()=>{
  if(connected && lastFrame && performance.now()-lastFrame>1600){
    ui.zero.disabled=true;
    message('센서 데이터가 멈췄습니다. 장갑 연결을 확인해주세요.');
  }
},500);
draw();
