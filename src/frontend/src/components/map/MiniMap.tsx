/* src/components/map/MiniMap.tsx */
'use client'
import { useEffect, useMemo, useRef } from 'react'
import dynamic from 'next/dynamic'
import * as L from 'leaflet'
import { useTelemetry } from '@/hooks/useTelemetry'

const MapContainer = dynamic(() => import('react-leaflet').then(m => m.MapContainer), { ssr:false })
const Marker       = dynamic(() => import('react-leaflet').then(m => m.Marker),       { ssr:false })
const Polyline     = dynamic(() => import('react-leaflet').then(m => m.Polyline),     { ssr:false })
const Circle       = dynamic(() => import('react-leaflet').then(m => m.Circle),       { ssr:false })

const OBST = [
  { xy: [20, 0] as [number, number], r: 8 },
  { xy: [-15, 15] as [number, number], r: 6 },
]

export default function MiniMap () {
  const { packet, buffer, mission } = useTelemetry()
  const mapRef = useRef<L.Map | null>(null)

  /* ─────────────── стабильные координаты ─────────────── */
  const pos = useMemo<[number, number] | null>(() => (
    packet ? [packet.lat, packet.lon] : null
  ), [packet?.lat, packet?.lon])            // зависим не от массива, а от чисел

  const yaw = packet?.yaw ?? 0

  /* ─────────────── трек – тоже не условный ───────────── */
  const track = useMemo<[number, number][]>(() =>
    buffer.slice(-300).map(p => [p.lat, p.lon]),
  [buffer])

  /* ─────────────── держим лодку в центре ─────────────── */
  useEffect(() => {
    if (mapRef.current && pos) {
      mapRef.current.setView(pos, mapRef.current.getZoom(), { animate:false })
    }
  }, [pos])

  /* ─────────────── иконка, пересоздаётся по yaw ───────── */
  const vehicleIcon = useMemo(() => L.divIcon({
    iconSize  : [22,22],
    iconAnchor: [11,11],
    className : 'leaflet-div-icon',      // не трогаем базовые стили
    html: `<div style="
            width:22px;height:22px;
            background:#e11d48;
            clip-path:polygon(50% 0,0 100%,100% 100%);
            transform:rotate(${yaw}rad);
          "></div>`
  }), [yaw])

  /* ─────────────── раньше return'или, теперь всё OK ──── */
  if (!pos) return <p className="text-gray-500">loading…</p>

  return (
    <MapContainer
      attributionControl={false}
      ref={mapRef}                /* ref вместо whenCreated */
      whenReady={() => {/* карта готова – ref уже заполнен */}}

      center={pos}
      zoom={1}
      crs={L.CRS.Simple}
      style={{
        height: 360,
        width: 360,
        borderRadius: 12,
        background: '#f5f5f5',
        boxShadow: '0 0 10px rgba(0,0,0,.25)',
      }}
      maxBounds={[[-60,-60],[60,60]]}
      dragging={false}
      zoomControl={false}
    >
      {OBST.map((o,i) => (
        <Circle key={i} center={o.xy} radius={o.r}
                pathOptions={{color:'#1e90ff', fillOpacity:0.4}}/>
      ))}

      {mission?.waypoints && <Polyline positions={mission.waypoints} color="orange" weight={2} />}
      {!mission?.waypoints   && <Polyline positions={track}           color="red"    weight={2} />}

      <Marker
        key={`boat-${yaw.toFixed(2)}`}   /* заставляем React-Leaflet перерисовать маркер */
        position={pos}
        icon={vehicleIcon}
      />
    </MapContainer>
  )
}
