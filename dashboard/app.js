"use strict";
const $ = id => document.getElementById(id);
const samples = [];
let lastId = 0, windowSeconds = 10, pausedAt = null, events = [], latestStatus = {};
let lastSample = null, degreeUnits = false, lastEventId = -1;
let viewSession=null,viewGeneration=0;
function resetPlotView(){
  samples.length=0;events=[];lastId=0;lastSample=null;lastEventId=-1;pausedAt=null;
  $("pause").textContent="冻结波形";
  for(const view of chartViews.values()){view.hover=null;view.tooltip.hidden=true;}
  renderEvents();drawAll();
}
const COLORS = {teal:"#45e1ce",blue:"#6ea9ff",purple:"#b59bff",amber:"#ffc479"};
const chartViews = new Map();
const SERIES_NAMES = {
  angle_adc:"原始角度",filtered_angle_adc:"滤波角度",target_adc:"动态角度目标",
  position_counts:"编码器位置",position_target_counts:"执行目标",destination_counts:"最终目的地",
  speed_counts_s:"实测速度",speed_target_counts_s:"规划速度",
  pwm_command:"PWM 指令",pwm_applied:"实际 PWM",x:"实测 X",accepted_x:"采用 X",y:"实测 Y"
};
function seriesText(sample, channel) {
  const raw=sample[channel.field];
  if(raw===null || raw===undefined || !Number.isFinite(Number(raw)))return "--";
  if(channel.motion){
    const speed=channel.field.includes("speed");
    const counts=`${fmt(raw,speed?2:0)} ${speed?"计数/s":"计数"}`;
    return degreeUnits ? `${fmt(valueOf(sample,channel),2)} ${speed?"°/s":"°"}（${counts}）` : counts;
  }
  const unit=channel.field.includes("adc")?" ADC":channel.field.startsWith("pwm")?" PWM计数":" px";
  return fmt(raw,unit===" px"?2:0)+unit;
}
function chartView(canvas) {
  if(chartViews.has(canvas.id))return chartViews.get(canvas.id);
  const tooltip=document.createElement("div");tooltip.className="chart-tooltip";tooltip.hidden=true;
  tooltip.setAttribute("role","tooltip");tooltip.id=`${canvas.id}-tooltip`;
  canvas.parentElement.append(tooltip);canvas.setAttribute("aria-describedby",tooltip.id);
  const view={tooltip,hover:null,channels:[],minSpan:1};chartViews.set(canvas.id,view);
  canvas.addEventListener("pointermove",event=>{
    const rect=canvas.getBoundingClientRect();
    view.hover={x:(event.clientX-rect.left)/rect.width,y:(event.clientY-rect.top)/rect.height};
    draw(canvas.id,view.channels,view.minSpan);
  });
  const leave=()=>{view.hover=null;tooltip.hidden=true;draw(canvas.id,view.channels,view.minSpan);};
  canvas.addEventListener("pointerleave",leave);canvas.addEventListener("pointercancel",leave);
  return view;
}
function fmt(v, digits=0) { return v === null || v === undefined || !Number.isFinite(Number(v)) ? "--" : Number(v).toLocaleString("zh-CN",{maximumFractionDigits:digits}); }
function signed(v,digits=0) { return v === null || v === undefined ? "--" : (v>=0?"+":"")+fmt(v,digits); }
function values(s, status) {
  const fresh = status.fresh;
  $("modeValue").textContent = status.mode;
  $("angleValue").textContent = fresh ? fmt(s?.angle_adc) : "--";
  $("angleSub").textContent = fresh ? `目标 ${fmt(s?.target_adc)} · 控制滤波 ${fmt(s?.filtered_angle_adc)}` : "等待新数据";
  $("positionValue").textContent = fresh ? fmt(s?.position_counts) : "--";
  $("positionSub").textContent = fresh ? `执行目标 ${fmt(s?.position_target_counts)} · 最终目的地 ${fmt(s?.destination_counts)}` : "等待新数据";
  $("speedValue").textContent = fresh ? signed(s?.speed_counts_s) : "--";
  $("speedSub").textContent = fresh ? `目标 ${signed(s?.speed_target_counts_s,1)} 计数/s · 50 ms 估计` : "等待新数据";
  $("pwmValue").textContent = fresh ? signed(s?.pwm_command) : "--";
  $("pwmSub").textContent = fresh ? `实际应用 ${signed(s?.pwm_applied)} · 周期 3600` : "等待新数据";
  const cameraFresh = fresh && status.camera_age_s !== null && status.camera_age_s < .650;
  $("xyValue").textContent = cameraFresh ? fmt(s?.x,2) : "--";
  $("xySub").textContent = cameraFresh ? `采用 X ${fmt(s?.accepted_x,2)} · 红色像素 ${fmt(s?.red_pixels)}` : "等待有效目标或相机帧";
  const settingFresh=!!(fresh && s?.coherent && s?.protocol_version>=5 && typeof s.angle_setting==="boolean");
  const settingStatus=$("settingStatus");
  settingStatus.className="setting-status";
  if(!fresh){settingStatus.textContent="等待串口数据";}
  else if(!settingFresh){
    settingStatus.textContent=`当前 V${s?.protocol_version??"?"} · 需要加载 V5 固件`;
    settingStatus.classList.add("warning");
  }else if(s.angle_setting){settingStatus.textContent="K4 设置中";settingStatus.classList.add("active");}
  else if(!s.balance_valid || s.state_code!==4){settingStatus.textContent="当前不可设置 · 尚未直立平衡";settingStatus.classList.add("warning");}
  else if(s.turning || s.braking){settingStatus.textContent="运动或停车交接中 · 请稍候";}
  else{settingStatus.textContent="已就绪 · 按实体 K4 进入设置";}
  for(const [id,field] of [["settingK1","setting_k1_presses"],["settingK2","setting_k2_presses"],["settingK3","setting_k3_presses"]])
    $(id).textContent=settingFresh && s.angle_setting?fmt(s[field]):"/";
  $("settingDegrees").textContent=settingFresh?fmt(s.angle_setting?s.setting_degrees:0):"/";
  $("keySub").textContent=!fresh?"等待新数据":!settingFresh?"请烧录 pendulum_vision_k4_telemetry_20261029.fs，当前数据没有设置次数":
    s.angle_setting?"按 K1/K2/K3 累加，按 K4 执行并清零显示":
    !s.balance_valid || s.state_code!==4?`当前：${s.state}；恢复直立平衡后才可进入 K4 设置`:
    s.turning || s.braking?"当前旋转或停车交接未结束，K4不会进入设置":"按 K4 后显示0次；K1加100°、K2加10°、K3加1°";
  $("portValue").textContent = `${status.port} · ${fmt(status.baud)} bps`;
  $("protocolValue").textContent = `VTX1 v${status.version ?? "--"} · ${s?.coherent ? "同步配对" : "等待配对/旧版"}`;
  $("framesValue").textContent = `${fmt(status.frames)} / ${fmt(status.bad_crc)}`;
  $("ageValue").textContent = `${fmt(status.vision_age_s,2)}s / ${fmt(status.camera_age_s,2)}s`;
  $("positionError").textContent = fresh ? `${signed(s?.position_error_counts)} 计数` : "--";
  $("cameraStatus").textContent = s?.camera_error ? `错误 ${s.camera_error} · 寄存器 0x${Number(s.camera_failed_register).toString(16)}` : s?.locked ? "配置通过 · 同步锁定" : s?.configured ? "配置通过 · 等待同步" : "初始化中";
  $("sideProtocol").textContent = `AA56 + VTX1 v${status.version ?? "--"}`;
  $("logPath").textContent = status.log_csv || "等待记录文件";
  const pill = $("connectionPill");
  pill.className = `connection-pill ${status.demo ? "demo" : fresh ? "" : "off"}`;
  pill.replaceChildren(); const orb = document.createElement("i");pill.append(orb);
  pill.append(document.createTextNode(status.demo ? "演示数据" : fresh ? "实时采集" : status.connected ? "等待完整数据" : "串口未连接"));
  let notice = "";
  if (status.demo) notice = "演示模式：以下曲线为模拟数据，未连接倒立摆。";
  else if (status.connected && !fresh) notice = "串口已打开，但尚未收到新鲜的配对遥测；检查位流和波特率。";
  else if (!(status.version >= 3) && status.connected) notice = "收到旧版遥测，缺少 Y、执行目标与实际 PWM；请加载完整遥测版位流。";
  else if (status.camera_age_s !== null && status.camera_age_s >= .650) notice = "串口仍有数据，但摄像头帧号停止更新；视觉坐标已过期。";
  if (status.capture_error) notice = status.capture_error;
  if(fresh && s?.protocol_version<5)notice=`当前收到 VTX1 v${s.protocol_version}，没有 K4 设置角度遥测。请加载 best-end/impl/pnr/pendulum_vision.fs。${s.state_code===5?" 当前设备处于故障停机，恢复直立平衡后再按K4。":""}`;
  if (pausedAt !== null) notice += " 波形已冻结，后台采集和文件记录继续。";
  $("notice").textContent = notice;
}
function renderEvents() {
  const id = events.at(-1)?.id ?? 0;
  if (id === lastEventId) return;
  lastEventId=id;
  const list=$("events");list.replaceChildren();
  for (const e of [...events].reverse().slice(0,30)) {
    const li=document.createElement("li"), time=document.createElement("time"), text=document.createElement("span");
    time.textContent=`${fmt(e.elapsed_s,2)}s`;text.textContent=e.text;li.className=e.kind;
    li.append(time,text);list.append(li);
  }
  if (!events.length) { const li=document.createElement("li");li.textContent="等待事件";list.append(li); }
}
function valueOf(sample, channel) {
  const v=sample[channel.field];
  return v === null || v === undefined ? null : v*(channel.motion && degreeUnits ? 360/(latestStatus.counts_per_rev||1040) : 1);
}
function draw(id, channels, minSpan) {
  const canvas=$(id), width=Math.max(240,canvas.clientWidth), height=Math.max(180,canvas.clientHeight), dpr=Math.min(devicePixelRatio||1,2);
  const view=chartView(canvas);view.channels=channels;view.minSpan=minSpan;view.tooltip.hidden=true;
  if(canvas.width!==Math.round(width*dpr)||canvas.height!==Math.round(height*dpr)){canvas.width=Math.round(width*dpr);canvas.height=Math.round(height*dpr);}
  const ctx=canvas.getContext("2d");ctx.setTransform(dpr,0,0,dpr,0,0);ctx.clearRect(0,0,width,height);
  const m={left:59,right:14,top:18,bottom:29}, w=width-m.left-m.right,h=height-m.top-m.bottom;
  const end=pausedAt ?? samples.at(-1)?.elapsed_s ?? windowSeconds,start=Math.max(0,end-windowSeconds);
  const visible=samples.filter(s=>s.elapsed_s>=start && s.elapsed_s<=end);
  let low=Infinity,high=-Infinity,count=0;
  for(const s of visible)for(const c of channels){const v=valueOf(s,c);if(v!==null&&Number.isFinite(v)){low=Math.min(low,v);high=Math.max(high,v);count++;}}
  if(!count){low=-minSpan/2;high=minSpan/2;}
  const span=Math.max(high-low,minSpan),mid=(low+high)/2;low=mid-span*.6;high=mid+span*.6;
  const xx=t=>m.left+(t-start)/windowSeconds*w,yy=v=>m.top+(high-v)/(high-low)*h;
  ctx.font='10px "Segoe UI",sans-serif';ctx.lineWidth=1;
  for(let i=0;i<=4;i++){
    const y=m.top+i*h/4,x=m.left+i*w/4;
    ctx.strokeStyle="#28384c";ctx.beginPath();ctx.moveTo(m.left,y);ctx.lineTo(width-m.right,y);ctx.stroke();
    ctx.fillStyle="#8296ac";ctx.textAlign="right";ctx.fillText(fmt(high-i*(high-low)/4,high-low<20?1:0),m.left-8,y+3);
    ctx.strokeStyle="#203044";ctx.beginPath();ctx.moveTo(x,m.top);ctx.lineTo(x,m.top+h);ctx.stroke();
    ctx.textAlign="center";ctx.fillText(`${fmt(start+i*windowSeconds/4)}s`,x,height-8);
  }
  ctx.save();ctx.beginPath();ctx.rect(m.left,m.top,w,h);ctx.clip();
  for(const e of events){
    if(e.elapsed_s<start||e.elapsed_s>end||!["disturbance","button","reset"].includes(e.kind))continue;
    ctx.strokeStyle=e.kind==="disturbance"?"#ffc47980":"#7b8ea45c";ctx.setLineDash(e.kind==="disturbance"?[]:[3,4]);
    ctx.beginPath();ctx.moveTo(xx(e.elapsed_s),m.top);ctx.lineTo(xx(e.elapsed_s),m.top+h);ctx.stroke();
  }
  for(const c of channels){
    ctx.setLineDash(c.dashed?[5,4]:[]);ctx.strokeStyle=COLORS[c.color];ctx.lineWidth=c.dashed?1.4:1.8;ctx.beginPath();
    let began=false,previous=null;
    for(const s of visible){const v=valueOf(s,c);
      if(v===null||!Number.isFinite(v)){began=false;previous=null;continue;}
      if(previous && (s.elapsed_s-previous.elapsed_s>.25||s.capture_segment!==previous.capture_segment))began=false;
      if(began)ctx.lineTo(xx(s.elapsed_s),yy(v));else ctx.moveTo(xx(s.elapsed_s),yy(v));
      began=true;previous=s;
    }ctx.stroke();
  }
  ctx.restore();ctx.setLineDash([]);
  if(!count){ctx.fillStyle="#8b9db3";ctx.textAlign="center";ctx.font='12px "Microsoft YaHei",sans-serif';ctx.fillText("等待有效数据",m.left+w/2,m.top+h/2);}
  if(view.hover && visible.length){
    const hx=view.hover.x*width,hy=view.hover.y*height;
    if(hx<m.left || hx>width-m.right || hy<m.top || hy>m.top+h)return;
    const time=start+(hx-m.left)/w*windowSeconds;
    // Select one actual sample for all series, including overlapping targets.
    let lo=0,hi=visible.length;
    while(lo<hi){const mid=(lo+hi)>>>1;if(visible[mid].elapsed_s<time)lo=mid+1;else hi=mid;}
    let sample=visible[Math.min(lo,visible.length-1)];
    if(lo>0 && Math.abs(visible[lo-1].elapsed_s-time)<Math.abs(sample.elapsed_s-time))sample=visible[lo-1];
    if(Math.abs(sample.elapsed_s-time)>.25)return;
    const x=xx(sample.elapsed_s),tip=view.tooltip;
    ctx.save();ctx.beginPath();ctx.rect(m.left,m.top,w,h);ctx.clip();
    ctx.strokeStyle="#c7d9ee99";ctx.lineWidth=1;ctx.setLineDash([3,3]);
    ctx.beginPath();ctx.moveTo(x,m.top);ctx.lineTo(x,m.top+h);ctx.stroke();ctx.setLineDash([]);
    for(const channel of channels){
      const v=valueOf(sample,channel);if(v===null || !Number.isFinite(v))continue;
      ctx.fillStyle=COLORS[channel.color];ctx.beginPath();ctx.arc(x,yy(v),3,0,Math.PI*2);ctx.fill();
    }
    ctx.restore();tip.replaceChildren();
    const heading=document.createElement("div");heading.className="tooltip-time";
    const timestamp=sample.received_at?` · ${sample.received_at.slice(11,23)}`:"";
    heading.textContent=`${fmt(sample.elapsed_s,3)} s${timestamp}`;tip.append(heading);
    for(const channel of channels){
      const row=document.createElement("div"),label=document.createElement("span"),number=document.createElement("strong");
      row.className="tooltip-row";label.style.color=COLORS[channel.color];
      label.textContent=SERIES_NAMES[channel.field]||channel.field;number.textContent=seriesText(sample,channel);
      row.append(label,number);tip.append(row);
    }
    tip.hidden=false;
    const parent=canvas.parentElement,px=view.hover.x*parent.clientWidth,py=view.hover.y*parent.clientHeight;
    let left=px+12;if(left+tip.offsetWidth>parent.clientWidth-4)left=px-tip.offsetWidth-12;
    tip.style.left=`${Math.max(4,left)}px`;
    tip.style.top=`${Math.max(4,Math.min(py+12,parent.clientHeight-tip.offsetHeight-4))}px`;
  }
}
function drawAll(){
  draw("angleChart",[{field:"filtered_angle_adc",color:"blue"},{field:"target_adc",color:"purple"}],60);
  draw("positionChart",[{field:"position_counts",color:"blue",motion:true}],degreeUnits?4:12);
  draw("speedChart",[{field:"speed_counts_s",color:"amber",motion:true},{field:"speed_target_counts_s",color:"purple",motion:true}],degreeUnits?8:24);
  draw("pwmChart",[{field:"pwm_command",color:"purple"},{field:"pwm_applied",color:"amber"}],100);
  draw("xChart",[{field:"x",color:"teal"},{field:"accepted_x",color:"purple"}],5);
  draw("yChart",[{field:"y",color:"blue"}],5);
}
async function poll(){
  try{
    const response=await fetch(`/api/snapshot?since=${lastId}`,{cache:"no-store"});if(!response.ok)throw Error(`HTTP ${response.status}`);
    const data=await response.json();
    const session=data.status.session_id??null,generation=data.status.view_generation??0;
    if(session===viewSession && generation<viewGeneration)return;
    if(session!==viewSession || generation!==viewGeneration){resetPlotView();viewSession=session;viewGeneration=generation;}
    if(data.status.frames<lastId){samples.length=0;lastId=0;pausedAt=null;}
    for(const s of data.samples){if(s.id>lastId){samples.push(s);lastId=s.id;}}
    if(samples.length>18000)samples.splice(0,samples.length-18000);
    events=data.events;latestStatus=data.status;lastSample=data.latest;
    values(lastSample,latestStatus);renderEvents();drawAll();
  }catch(error){$("connectionPill").className="connection-pill off";$("connectionPill").textContent="网页服务未连接";$("modeValue").textContent="网页服务未连接";$("notice").textContent=`无法读取本地服务：${error.message}`;}
  finally{setTimeout(poll,200);}
}
for(const b of document.querySelectorAll(".range"))b.addEventListener("click",()=>{windowSeconds=Number(b.dataset.seconds);document.querySelectorAll(".range").forEach(x=>x.classList.toggle("active",x===b));drawAll();});
$("units").addEventListener("change",e=>{degreeUnits=e.target.value==="degrees";drawAll();});
$("pause").addEventListener("click",()=>{pausedAt=pausedAt===null?(samples.at(-1)?.elapsed_s??0):null;$("pause").textContent=pausedAt===null?"冻结波形":"恢复滚动";values(lastSample,latestStatus);drawAll();});
$("mark").addEventListener("click",async()=>{const button=$("mark");button.disabled=true;try{const r=await fetch("/api/mark",{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify({kind:"disturbance"})});if(!r.ok)throw Error(`HTTP ${r.status}`);button.textContent="已标记扰动";}catch(e){$("notice").textContent=`标记未保存：${e.message}`;}finally{setTimeout(()=>{button.disabled=false;button.textContent="标记一次扰动";},900);}});
$("clearWaveforms").addEventListener("click",async()=>{
  const button=$("clearWaveforms");button.disabled=true;
  try{
    const response=await fetch("/api/clear-waveforms",{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify({kind:"clear_waveforms"})});
    if(!response.ok)throw Error(`HTTP ${response.status}`);
    const result=await response.json();
    if(result.session_id!==viewSession || result.view_generation>=viewGeneration){
      resetPlotView();viewSession=result.session_id;viewGeneration=result.view_generation;
    }
    $("notice").textContent="波形和事件已清空，时间从0开始；原始采集文件继续保存。";
  }catch(error){$("notice").textContent=`清空失败，原波形保留：${error.message}。如后台仍是旧版，请重启后台。`;}
  finally{button.disabled=false;}
});
window.addEventListener("resize",drawAll);
setInterval(()=>{$("clock").textContent=new Date().toLocaleTimeString("zh-CN",{hour12:false});},1000);
drawAll();poll();
