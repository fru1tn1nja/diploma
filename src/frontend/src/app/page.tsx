// app/page.tsx
'use client'
import MiniMap       from '@/components/map/MiniMap'
import BatteryChart  from '@/components/charts/BatteryChart'
import ManualPanel   from '@/components/ui/ManualPanel'
import {useTelemetry} from '@/hooks/useTelemetry'
import {useEffect}    from 'react'

export default function Home(){
  const connect = useTelemetry(s=>s.connect)
  useEffect(()=>{connect()},[connect])

  return (
    <main className="p-4 flex flex-col items-center gap-4">
      <h1 className="text-lg font-bold">UAV Dashboard</h1>
      <MiniMap/>
      <BatteryChart/>
      <ManualPanel/>
    </main>
  )
}