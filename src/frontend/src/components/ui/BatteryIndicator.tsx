/* src/components/ui/BatteryIndicator.tsx */
'use client'
import {useTelemetry} from '@/hooks/useTelemetry'

export default function BatteryIndicator(){
  const packet = useTelemetry(s=>s.packet)
  if(!packet) return null
  const level = packet.battery ?? 0
  const color = level>50 ? 'bg-green-500' : level>20 ? 'bg-amber-400' : 'bg-red-500'
  return (
    <div className="w-72 mx-auto">
      <div className="flex justify-between text-sm mb-1"><span>Battery</span><span>{level}%</span></div>
      <div className="w-full h-3 bg-slate-200 rounded">
        <div className={`h-full rounded ${color}`} style={{width:`${level}%`}}></div>
      </div>
    </div>
  )
}