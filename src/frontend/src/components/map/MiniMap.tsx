/* src/components/map/MiniMap.tsx */
/* eslint-disable @typescript-eslint/no-explicit-any */
/* eslint-disable @typescript-eslint/no-require-imports */
'use client'

import { useEffect, useRef, useMemo } from 'react'
import dynamic from 'next/dynamic'
import { useTelemetry } from '@/hooks/useTelemetry'
import type { Map as LeafletMap } from 'leaflet'

// ───────────────────────────── Leaflet (browser‑only) ─────────────────────
let L: any
if (typeof window !== 'undefined') {
  // eslint-disable-next-line @typescript-eslint/no-var-requires
  L = require('leaflet')

  // корректная метрика для CRS.Simple
  L.CRS.Simple.distance = (a: [number, number], b: [number, number]) =>
    Math.hypot(a[0] - b[0], a[1] - b[1])

  // иконка цели (золотая звезда)
  L.GOAL_ICON = L.divIcon({
    className: '',
    iconSize: [16, 16],
    iconAnchor: [8, 8],
    html: `<div style="width:16px;height:16px;background:gold;
                      clip-path:polygon(50% 0,61% 35%,98% 35%,68% 57%,
                                         79% 91%,50% 70%,21% 91%,32% 57%,
                                          2% 35%,39% 35%)"></div>`
  })
}

// ─────────────────────────── dynamic imports (SSR‑safe) ───────────────────
const MapContainer: any = dynamic(
  () => import('react-leaflet').then(m => m.MapContainer),
  { ssr: false }
)
const Marker: any = dynamic(
  () => import('react-leaflet').then(m => m.Marker),
  { ssr: false }
)
const Polyline: any = dynamic(
  () => import('react-leaflet').then(m => m.Polyline),
  { ssr: false }
)
const Circle: any = dynamic(
  () => import('react-leaflet').then(m => m.Circle),
  { ssr: false }
)

// ─────────────────────────── статичные препятствия ────────────────────────
const OBST = [
  { xy: [20, 0] as [number, number], r: 8 },
  { xy: [-15, 15] as [number, number], r: 6 }
]

// ───────────────────── центровка + вращение карты ─────────────────────────
function useCenterAndRotate(
  map: LeafletMap | null,
  pos: [number, number] | null,
  yaw: number | null
) {
  useEffect(() => {
    if (!map || !pos || yaw === null) return

    map.setView(pos, map.getZoom(), { animate: false })

    const deg = (-yaw * 180) / Math.PI // «нос» вверх
    const pane = map.getContainer().querySelector('.leaflet-map-pane') as
      | HTMLElement
      | null
    if (pane) {
      pane.style.transition = 'transform .12s linear'
      pane.style.transform = `rotate(${deg}deg)`
    }
  }, [map, pos, yaw])
}

// ─────────────────────────────── MiniMap ──────────────────────────────────
export default function MiniMap() {
  // Zustand‑store
  const { packet, buffer, mode, mission } = useTelemetry()

  const mapRef = useRef<LeafletMap | null>(null)

  // вычисляемые данные
  const pos = packet ? ([packet.lat, packet.lon] as [number, number]) : null
  const yaw = packet ? packet.yaw ?? 0 : null

  const track = useMemo<[number, number][]>(
    () => buffer.slice(-300).map(b => [b.lat, b.lon] as [number, number]),
    [buffer]
  )

  // применяем вращение
  useCenterAndRotate(mapRef.current, pos, yaw)

  // нет данных → placeholder
  if (!pos || !L) return <p className="text-gray-500">loading…</p>

  return (
    <MapContainer
      center={pos}
      zoom={1}
      crs={L.CRS.Simple}
      whenCreated={(map: LeafletMap) => {
        mapRef.current = map
      }}
      style={{
        height: 360,
        width: 360,
        borderRadius: 12,
        overflow: 'hidden',
        boxShadow: '0 0 8px #000'
      }}
      maxBounds={[[-60, -60], [60, 60]]}
      dragging={false}
      zoomControl={false}
    >
      {/* препятствия */}
      {OBST.map((o, i) => (
        <Circle
          key={i}
          center={o.xy}
          radius={o.r}
          pathOptions={{ color: '#1e90ff', fillOpacity: 0.4 }}
        />
      ))}

      {/* цель */}
      {mission?.goal && <Marker position={mission.goal} icon={L.GOAL_ICON} />}

      {/* маршрут планировщика */}
      {mission?.waypoints && mode === 'AI' && (
        <Polyline positions={mission.waypoints} color="orange" weight={2} />
      )}

      {/* fallback‑трекинг (красный) */}
      {mode === 'AI' && !mission?.waypoints && (
        <Polyline positions={track} color="red" weight={2} />
      )}

      {/* сам аппарат */}
      <Marker
        position={pos}
        icon={L.divIcon({
          className: '',
          iconSize: [14, 14],
          iconAnchor: [7, 7],
          html: `<div style="width:14px;height:14px;background:red;clip-path:polygon(50% 0,0 100%,100% 100%)"></div>`
        })}
      />
    </MapContainer>
  )
}
