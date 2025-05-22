// src/components/ui/ManualPanel.tsx
'use client'
import ROSLIB from 'roslib'
import {useState} from 'react'
import {useTelemetry} from '@/hooks/useTelemetry'

const ros = new ROSLIB.Ros({url:'ws://localhost:9090'})
const pub = new ROSLIB.Topic({ros,name:'/manual_cmd_vel',messageType:'geometry_msgs/Twist'})
const twist = {linear:{x:0,y:0,z:0}, angular:{x:0,y:0,z:0}}

export default function ManualPanel(){
  const mode = useTelemetry(s=>s.mode)          // 'Manual' | 'AI'
  const disabled = mode==='AI'

  const [v,setV]=useState(0)
  const [w,setW]=useState(0)
  function send(x:number,z:number){
    twist.linear.x = x; twist.angular.z = z
    pub.publish(new ROSLIB.Message(twist))
  }
  const btn = "px-3 py-1 rounded bg-slate-700 text-white hover:bg-slate-600 " +
              "disabled:bg-slate-300 disabled:text-slate-500"
  return (
   <div className="flex gap-2 mt-2 select-none">
     <button className={btn} disabled={disabled}
             onClick={()=>{setW(w+0.3);send(v,w+0.3)}}>⟲ Left</button>
     <button className={btn} disabled={disabled}
             onClick={()=>{setW(w-0.3);send(v,w-0.3)}}>Right ⟳</button>
     <button className={btn} disabled={disabled}
             onClick={()=>{setV(v+0.2);send(v+0.2,w)}}>+Speed</button>
     <button className={btn} disabled={disabled}
             onClick={()=>{setV(Math.max(0,v-0.2));send(Math.max(0,v-0.2),w)}}>-Speed</button>
   </div>)
}