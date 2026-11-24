'use strict';
function createContacts(model) {
  const knuckles=model.base.slice(1);
  const width=Math.max(...knuckles.map(p=>p[0]))-Math.min(...knuckles.map(p=>p[0]));
  const length=Math.max(...knuckles.map(p=>p[1]));
  const palm=[(knuckles[0][0]+knuckles[3][0])/2,length*.48,-12];
  const palmRadius=[width*.56,length*.48,19];
  const radii=model.lengths.map(lengths=>lengths.slice(1).flatMap((len,j)=>{
    const r=Math.max(5,Math.min(14,len*(j===0?.38:.48)))*.65;
    return [r,r,r];
  }));
  function points(f,q){
    let p=model.base[f];
    const axes=model.frames[f].map((_,i)=>model.frames[f].map(row=>row[i]));
    const blend=(a,b,t)=>a.map((x,i)=>x*Math.cos(t)+b[i]*Math.sin(t));
    [axes[0],axes[1]]=[blend(axes[0],axes[1],q[3]),blend(axes[1],axes[0],-q[3])];
    const samples=[];
    for(let j=0;j<3;j++){
      [axes[1],axes[2]]=[blend(axes[1],axes[2],-q[j]),blend(axes[2],axes[1],q[j])];
      const segment=axes[1].map(v=>v*model.lengths[f][j]);
      if(j>0) for(const t of [.25,.60,1]) samples.push(p.map((v,i)=>v+t*segment[i]));
      p=p.map((v,i)=>v+segment[i]);
    }
    return samples;
  }
  function distanceSquared(a,b){
    return (a[0]-b[0])**2+(a[1]-b[1])**2+(a[2]-b[2])**2;
  }
  const rest=model.base.map((_,f)=>points(f,[0,0,0,0]));
  const limits=rest.map((a,f)=>rest.map((b,g)=>a.map((p,i)=>b.map((q,j)=>{
    const contact=Math.min(radii[f][i]+radii[g][j],Math.sqrt(distanceSquared(p,q)))-.5;
    return Math.max(0,contact)**2;
  }))));
  let lastKey, lastResult;
  return pose=>{
    const key=JSON.stringify(pose);
    if(key===lastKey) return lastResult;
    const current=rest.slice();
    const safe=pose.map((q,f)=>{
      function fits(candidate){
        return candidate.every((p,i)=>{
          const distance=p.reduce((sum,v,k)=>sum+((v-palm[k])/(palmRadius[k]+radii[f][i]))**2,0);
          return distance>=.96 && current.every((other,g)=>g===f ||
            other.every((point,j)=>distanceSquared(p,point)>=limits[f][g][i][j]));
        });
      }
      const full=points(f,q);
      if(fits(full)){current[f]=full; return q.slice();}
      let low=0, high=1;
      for(let i=0;i<7;i++){
        const mid=(low+high)/2, candidate=points(f,q.map(v=>v*mid));
        if(fits(candidate)){low=mid; current[f]=candidate;}
        else high=mid;
      }
      return q.map(v=>v*low);
    });
    lastKey=key;
    lastResult=safe;
    return safe;
  };
}
