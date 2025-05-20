/* eslint-disable @typescript-eslint/no-explicit-any */
'use client'

import { useTelemetry } from '@/hooks/useTelemetry'
import dynamic from 'next/dynamic'

// ——— динамические импорты  (⟶ only in browser) ———
const MapContainer: any = dynamic(
  () => import('react-leaflet').then(m => m.MapContainer), { ssr: false }
)
const TileLayer: any = dynamic(
  () => import('react-leaflet').then(m => m.TileLayer),   { ssr: false }
)
const Marker: any = dynamic(
  () => import('react-leaflet').then(m => m.Marker),      { ssr: false }
)

export default function MapView() {
  const p = useTelemetry(s => s.packet)
  if (!p) return <p className="text-gray-500">waiting telemetry…</p>

  return (
    <MapContainer
      center={[p.lat, p.lon]}
      zoom={15}
      scrollWheelZoom={true}
      style={{ height: 400, width: '100%' }}
    >
      <TileLayer
        attribution="© OpenStreetMap contributors"
        url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
      />
      <Marker position={[p.lat, p.lon]} />
    </MapContainer>
  )
}
