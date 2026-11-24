'use strict';
function createGloveMesh(canvas, model) {
  const gl=canvas.getContext('webgl',{alpha:true,antialias:true});
  if(!gl) return null;
  const skin=model.skin, count=skin.points.length;
  const vertices=new Float32Array((count+256)*6);
  const faces=skin.triangles.slice();
  const ring=[];
  for(let i=0;i<64;i++){
    const t=i*Math.PI/32;
    for(const [r,z] of [[16.95,-.8],[29,-.8],[29,.8],[16.95,.8]]){
      ring.push([model.coil[0]+r*Math.cos(t),model.coil[1]+r*Math.sin(t),model.coil[2]+z]);
    }
    for(let j=0;j<4;j++){
      const a=count+4*i+j, b=count+4*i+(j+1)%4;
      const c=count+4*((i+1)%64)+j, d=count+4*((i+1)%64)+(j+1)%4;
      faces.push(a,c,b,b,c,d);
    }
  }
  function shader(type,source){
    const s=gl.createShader(type);
    gl.shaderSource(s,source); gl.compileShader(s);
    if(!gl.getShaderParameter(s,gl.COMPILE_STATUS)) throw Error(gl.getShaderInfoLog(s));
    return s;
  }
  const program=gl.createProgram();
  gl.attachShader(program,shader(gl.VERTEX_SHADER,`
    attribute vec3 position, normal;
    uniform mat3 rotation;
    uniform vec2 scale;
    uniform vec3 color;
    varying vec3 lit;
    void main(){
      vec3 p=rotation*(position-vec3(0.,48.,0.));
      vec3 n=normalize(rotation*normal);
      float shade=.49+.70*max(0.,dot(n,normalize(vec3(-.42,.61,.74))));
      lit=color*shade+.036*pow(max(n.z,0.),10.);
      gl_Position=vec4(p.xy*scale,-p.z/390.,1.-p.z/390.);
    }`));
  gl.attachShader(program,shader(gl.FRAGMENT_SHADER,`
    precision mediump float;
    varying vec3 lit;
    void main(){gl_FragColor=vec4(lit,1.);}
  `));
  gl.linkProgram(program);
  if(!gl.getProgramParameter(program,gl.LINK_STATUS)) throw Error(gl.getProgramInfoLog(program));
  gl.useProgram(program);
  const mesh=gl.createBuffer(), indices=gl.createBuffer();
  gl.bindBuffer(gl.ARRAY_BUFFER,mesh);
  gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER,indices);
  gl.bufferData(gl.ELEMENT_ARRAY_BUFFER,new Uint16Array(faces),gl.STATIC_DRAW);
  for(const [name,offset] of [['position',0],['normal',12]]){
    const a=gl.getAttribLocation(program,name);
    gl.enableVertexAttribArray(a); gl.vertexAttribPointer(a,3,gl.FLOAT,false,24,offset);
  }
  const rotation=gl.getUniformLocation(program,'rotation');
  const aspect=gl.getUniformLocation(program,'scale');
  const color=gl.getUniformLocation(program,'color');
  gl.enable(gl.DEPTH_TEST);
  const axis=(f,i)=>model.frames[f].map(r=>r[i]);
  const plus=(a,b)=>a.map((v,i)=>v+b[i]);
  const minus=(a,b)=>a.map((v,i)=>v-b[i]);
  const times=(a,s)=>a.map(v=>v*s);
  const unit=a=>times(a,1/Math.max(1e-9,Math.hypot(...a)));
  const quat=(a,t)=>[Math.cos(t/2),...times(a,Math.sin(t/2))];
  function deform(pose) {
    const joints=pose.map(([a,b,c,d],f)=>{
      const hinge=axis(f,0), up=axis(f,2), forward=axis(f,1);
      return [mul(quat(up,d),quat(hinge,-a)),quat(hinge,-b),quat(hinge,-c)].map((q,j)=>{
        const s=Math.hypot(...q.slice(1));
        return {axis:unit(q.slice(1)), angle:2*Math.atan2(s,q[0]), pivot:skin.pivots[f][j],
          plane:unit(plus(forward,rotate(q,forward))), palm:unit(times(plus(up,rotate(q,up)),-1))};
      });
    });
    vertices.fill(0);
    for(let v=0;v<count;v++){
      const rest=skin.points[v], out=rest.slice();
      for(let f=0;f<5;f++){
        const blend=skin.membership[v][f];
        if(blend<=.0001) continue;
        let point=rest;
        for(let j=2;j>=0;j--){
          const setup=joints[f][j], w=skin.weights[f][j][v];
          if(setup.angle<.00001) continue;
          const moved=rotate(quat(setup.axis,w*setup.angle),minus(point,setup.pivot));
          point=plus(setup.pivot,moved);
          if(f===0&&j===0) continue;
          const child=w>=.5;
          const dist=dot(moved,setup.plane);
          const over=(child?Math.min(dist,0):j===0?0:Math.max(dist,0))*skin.volar[f][j][v];
          point=plus(minus(point,times(setup.plane,over)),times(setup.palm,.32*Math.abs(over)));
        }
        for(let i=0;i<3;i++) out[i]+=blend*(point[i]-rest[i]);
      }
      vertices.set(out,v*6);
    }
    for(let i=0;i<ring.length;i++) vertices.set(ring[i],(count+i)*6);
    for(let i=0;i<faces.length;i+=3){
      const a=faces[i]*6,b=faces[i+1]*6,c=faces[i+2]*6;
      const n=cross([vertices[b]-vertices[a],vertices[b+1]-vertices[a+1],vertices[b+2]-vertices[a+2]],
                    [vertices[c]-vertices[a],vertices[c+1]-vertices[a+1],vertices[c+2]-vertices[a+2]]);
      for(const v of [a,b,c]) for(let j=0;j<3;j++) vertices[v+3+j]+=n[j];
    }
    gl.bufferData(gl.ARRAY_BUFFER,vertices,gl.DYNAMIC_DRAW);
  }
  let lastPose='';
  return (pose,attitude,w,h)=>{
    const key=JSON.stringify(pose);
    if(key!==lastPose){deform(pose); lastPose=key;}
    const horizontal=rotate(attitude,[1,0,0]);
    const vertical=rotate(attitude,[0,1,0]);
    const depth=rotate(attitude,[0,0,1]);
    let extentX=0, extentY=0;
    for(let i=0;i<vertices.length;i+=6){
      const x=vertices[i], y=vertices[i+1]-48, z=vertices[i+2];
      const perspective=1-(horizontal[2]*x+vertical[2]*y+depth[2]*z)/390;
      extentX=Math.max(extentX,Math.abs(horizontal[0]*x+vertical[0]*y+depth[0]*z)/perspective);
      extentY=Math.max(extentY,Math.abs(horizontal[1]*x+vertical[1]*y+depth[1]*z)/perspective);
    }
    const margin=Math.min(20,Math.min(w,h)*.08);
    const size=Math.min(w/205,h/235,(w/2-margin)/extentX,(h/2-margin)/extentY);
    gl.viewport(0,0,canvas.width,canvas.height);
    gl.clearColor(0,0,0,0); gl.clear(gl.COLOR_BUFFER_BIT|gl.DEPTH_BUFFER_BIT);
    gl.uniform2f(aspect,2*size/w,2*size/h);
    gl.uniformMatrix3fv(rotation,false,[...horizontal,...vertical,...depth]);
    gl.uniform3f(color,.15,.16,.19);
    gl.drawElements(gl.TRIANGLES,skin.triangles.length,gl.UNSIGNED_SHORT,0);
    gl.uniform3f(color,.72,.37,.19);
    gl.drawElements(gl.TRIANGLES,faces.length-skin.triangles.length,gl.UNSIGNED_SHORT,skin.triangles.length*2);
  };
}
