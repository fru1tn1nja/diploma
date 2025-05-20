"use client"  
import MapView from '@/components/map/MapView'
import BatteryChart from '@/components/charts/BatteryChart'
import ControlPanel from '@/components/ui/ControlPanel'
import {useTelemetry} from '@/hooks/useTelemetry'
{/* import DroneMini3D from '@/components/mini3d/Drone' */}
import {useEffect} from 'react'
export const dynamic = 'force-dynamic'
export default function Home() {
  const connect = useTelemetry(s=>s.connect)
  useEffect(()=>{ connect() }, [connect])

  return (
    <main className="p-4 space-y-6">
      <h1 className="text-2xl font-semibold">UAV Dashboard</h1>
      <MapView/>
      {/* <DroneMini3D /> */}
      <BatteryChart/>
      <ControlPanel/>
    </main>
  )
}