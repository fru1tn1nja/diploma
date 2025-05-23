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
  const mapRef = useRef<L.Map|null>(null)

  const pos = useMemo<[number,number] | null>(
    () => packet ? [packet.lat, packet.lon] : null,
    [packet?.lat, packet?.lon]
  )
  const yaw = packet?.yaw ?? 0

  // трек
  const track = useMemo<[number,number][]>(
    () => buffer.slice(-300).map(p => [p.lat, p.lon]),
    [buffer]
  )

  // лодка как раньше
  const vehicleIcon = useMemo(() => L.divIcon({
    iconSize:   [11,11],
    iconAnchor: [5,11],
    className:  '',
    html: `<div style="
      width:11px;height:11px;
      background:#e11d48;
      clip-path:polygon(50% 0,0 100%,100% 100%);
      transform:rotate(${yaw}rad);
      transform-origin:50% 100%;
    "></div>`,
  }), [yaw])

  // **новое**: иконка цели
  const goalIcon = useMemo(() => L.divIcon({
    iconSize:   [16,16],
    iconAnchor: [11,11],
    className:  '',
    html: `<div style="
      width:16px;height:16px;
      background:gold;
      clip-path:polygon(
        50% 0%,61% 35%,98% 35%,68% 57%,79% 91%,
        50% 70%,21% 91%,32% 57%,2% 35%,39% 35%
      );
    "></div>`,
  }), [])

  // центрируем карту под лодкой
  useEffect(() => {
    if (mapRef.current && pos) {
      mapRef.current.setView(pos, mapRef.current.getZoom(), { animate:false })
    }
  }, [pos])

  if (!pos) return <p className="text-gray-500">loading…</p>

  return (
    <MapContainer
      ref={mapRef}
      whenReady={() => {}}
      attributionControl={false}
      center={pos}
      zoom={1}
      crs={L.CRS.Simple}
      style={{
        height:360, width:360,
        borderRadius:12, background:'#f5f5f5',
        boxShadow:'0 0 10px rgba(0,0,0,.25)',
      }}
      maxBounds={[[-60,-60],[60,60]]}
      dragging={false}
      zoomControl={false}
    >
      {OBST.map((o,i) => (
        <Circle key={i} center={o.xy} radius={o.r}
                pathOptions={{color:'#1e90ff',fillOpacity:0.4}}/>
      ))}

      {/* если есть миссия — рисуем её трассу */}
      {/*{mission?.waypoints
        ? <Polyline positions={mission.waypoints} color="orange" weight={2} />
        : <Polyline positions={track}           color="red"    weight={2} />
      }*/}
      <Polyline positions={track}           color="red"    weight={2} />
      {/* 🚩 НОВЫЙ МАРКЕР ЦЕЛИ */}
      {mission?.goal && (
        <Marker
          position={mission.goal}
          icon={goalIcon}
          title="Target"
        />
      )}

      {/* 🔺 Лодка */}
      <Marker
        key={`boat-${yaw.toFixed(2)}`}
        position={pos}
        icon={vehicleIcon}
      />
    </MapContainer>
  )
}
