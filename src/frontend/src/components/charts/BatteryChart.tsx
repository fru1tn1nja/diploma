'use client'
import {useTelemetry} from '@/hooks/useTelemetry'
import dynamic from 'next/dynamic'

const Plot = dynamic(() => import('react-plotly.js'), { ssr: false })

export default function BatteryChart() {
  const buf = useTelemetry(s => s.buffer)
  const y = buf.map(b => b.battery)
  const x = buf.map(b => new Date(b.ts))

  return (
    <Plot
      data={[{ x, y, type: 'scatter', mode: 'lines' }]}
      layout={{ height: 200, margin: { l: 30, r: 10, t: 10, b: 30 } }}
      style={{ width: '100%' }}
    />
  )
}