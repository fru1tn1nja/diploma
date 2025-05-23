"use client"
import dynamic from 'next/dynamic'
import BatteryChart from '@/components/charts/BatteryChart'
import ManualPanel from '@/components/ui/ManualPanel'
import BatteryIndicator from '@/components/ui/BatteryIndicator'
import { useTelemetry } from '@/hooks/useTelemetry'
import { useEffect } from 'react'

/* MiniMap зависит от Leaflet (window, document). Подключаем SSA-less */
const MiniMap = dynamic(() => import('@/components/map/MiniMap'), { ssr: false })

export default function Home() {
  const connect = useTelemetry((s) => s.connect)
  const mode = useTelemetry((s) => s.mode)

  useEffect(() => {
    connect()
  }, [connect])

  return (
    <main className="p-4 flex flex-col items-center gap-4 text-gray-900">
      <h1 className="text-lg font-bold flex items-center gap-3">
        Shared Control System
        <span
          className={
            mode === 'AI'
              ? 'px-2 py-0.5 rounded bg-green-500 text-white text-sm'
              : 'px-2 py-0.5 rounded bg-amber-400 text-black text-sm'
          }
        >
          {mode}
        </span>
      </h1>
      <MiniMap />
      <BatteryIndicator />
      <BatteryChart />
      <ManualPanel />
    </main>
  )
}
