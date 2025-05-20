'use client'
import dynamic from 'next/dynamic'
import { useTelemetry } from '@/hooks/useTelemetry'

// React Leaflet (динамический импорт, чтобы отключить SSR)
const MapContainer = dynamic(
  () => import('react-leaflet').then(mod => mod.MapContainer),
  { ssr: false }
)
const TileLayer = dynamic(() => import('react-leaflet').then(m => m.TileLayer), { ssr: false })
const Marker = dynamic(() => import('react-leaflet').then(m => m.Marker), { ssr: false })

export default function MapView() {
  const p = useTelemetry(s => s.packet)
  if (!p) return <p>waiting telemetry…</p>

  return (
    <MapContainer
      center={[p.lat, p.lon]}
      zoom={15}
      scrollWheelZoom={true}
      style={{ height: 400, width: '100%' }}
    >
      <TileLayer
        attribution='© OpenStreetMap'
        url='https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png'
      />
      <Marker position={[p.lat, p.lon]} />
    </MapContainer>
  )
}
