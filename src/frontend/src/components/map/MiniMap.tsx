/* src/components/map/MiniMap.tsx */
'use client'

import { useEffect, useRef, useMemo } from 'react'
import dynamic from 'next/dynamic'
import { useTelemetry } from '@/hooks/useTelemetry'

// ---------------------------------------------------------------------------
// Leaflet: загружаем только в браузере, добавляем иконку цели
// ---------------------------------------------------------------------------
let L: any = null
if (typeof window !== 'undefined') {
  // eslint-disable-next-line @typescript-eslint/no-require-imports
  L = require('leaflet')

  /* переопределяем distance для CRS.Simple, чтобы Polyline/Circle корректно
     рассчитывали длины при рисовании трека */
  // @ts-ignore – Leaflet не знает о нашем синдексе
  L.CRS.Simple.distance = (a: any, b: any) => Math.hypot(a[0] - b[0], a[1] - b[1])

  // ------------------------------- GOAL ICON ------------------------------
  // маленькая золотая звезда 16×16 px (CSS clip-path)
  L.GOAL_ICON = L.divIcon({
    className: '',
    iconSize: [16, 16],
    iconAnchor: [8, 8],            // центр
    html: `<div style="width:16px;height:16px;background:gold;
                     clip-path:polygon(
                       50% 0, 61% 35%, 98% 35%, 68% 57%,
                       79% 91%, 50% 70%, 21% 91%, 32% 57%,
                       2% 35%, 39% 35%
                     )"></div>`
  })
}

// ---------------------------------------------------------------------------
// динамические импорты react‑leaflet (иначе ломается SSR)
// ---------------------------------------------------------------------------
const Map      = dynamic(() => import('react-leaflet').then(m => m.MapContainer), { ssr: false }) as any
const Marker   = dynamic(() => import('react-leaflet').then(m => m.Marker),       { ssr: false }) as any
const Circle   = dynamic(() => import('react-leaflet').then(m => m.Circle),       { ssr: false }) as any
const Polyline = dynamic(() => import('react-leaflet').then(m => m.Polyline),     { ssr: false }) as any

// ---------------------------------------------------------------------------
// Статичные препятствия (пример)
// ---------------------------------------------------------------------------
const OBST = [
  { xy: [ 20,  0] as [number, number], r: 8 },
  { xy: [-15, 15] as [number, number], r: 6 },
]

// ---------------------------------------------------------------------------
// Хук центрирования карты и плавного вращения тайлового слоя
// ---------------------------------------------------------------------------
function useCenterAndRotate(mapRef: React.MutableRefObject<any>, pos: [number, number], yaw: number) {
  useEffect(() => {
    const map = mapRef.current
    if (!map) return

    // 1) центрируем без анимации, чтобы "аппарат" всегда был по центру
    map.setView(pos, map.getZoom(), { animate: false })

    // 2) вращаем слой .leaflet-map-pane – явная CSS-трансформация
    const deg = (-yaw * 180) / Math.PI   // «нос» вверх, поэтому минус
    const pane = map.getContainer().querySelector('.leaflet-map-pane') as HTMLElement | null
    if (pane) {
      pane.style.transition = 'transform .12s linear'
      pane.style.transform  = `rotate(${deg}deg)`
    }
  }, [pos, yaw, mapRef])
}

// ---------------------------------------------------------------------------
// Компонент MiniMap
// ---------------------------------------------------------------------------
export default function MiniMap() {
  // извлекаем всё, что нужно, из Zustand‑store
  const { packet, buffer, mode, mission } = useTelemetry()

  const mapRef = useRef<any>(null)

  // Пока нет данных – показываем «loading…»
  if (!packet || !L) return <p className="text-gray-500">loading…</p>

  // позиция и курс аппарата
  const pos  = [packet.lat, packet.lon] as [number, number]
  const yaw  = packet.yaw ?? 0

  // трек: скользящее окно последних 300 точек
  const track = useMemo(
    () => buffer.slice(-300).map(b => [b.lat, b.lon]) as [number, number][],
    [buffer]
  )

  // применяем центрирование и вращение
  useCenterAndRotate(mapRef, pos, yaw)

  return (
    <Map
      ref={mapRef}
      center={pos}
      zoom={1}
      crs={L.CRS.Simple}
      style={{ height: 360, width: 360, borderRadius: 12, overflow: 'hidden', boxShadow: '0 0 8px #000' }}
      maxBounds={[[-60, -60], [60, 60]]}
      dragging={false}          /* мини‑карта — без panning колёсиком */
      zoomControl={false}
    >
      {/* препятствия */}
      {OBST.map((o, i) => (
        <Circle key={i} center={o.xy} radius={o.r}
                pathOptions={{ color: '#1e90ff', fillOpacity: 0.4 }} />
      ))}

      {/* цель (приходит из mission.goal) */}
      {mission?.goal && <Marker position={mission.goal} icon={L.GOAL_ICON} />}

      {/* планировщик: путь waypoints (оранж.) */}
      {mission?.waypoints && mode === 'AI' && (
        <Polyline positions={mission.waypoints} color="orange" weight={2} />
      )}

      {/* фоловер‑линия (красная) – fallback, если waypoints нет */}
      {mode === 'AI' && !mission?.waypoints && (
        <Polyline positions={track} color="red" weight={2} />
      )}

      {/* сам аппарат: треугольная стрелка */}
      <Marker
        position={pos}
        icon={L.divIcon({
          className: '',
          iconSize: [14, 14],
          iconAnchor: [7, 7],
          html: `<div style="width:14px;height:14px;background:red;
                           clip-path:polygon(50% 0,0 100%,100% 100%)"></div>`
        })}
      />
    </Map>
  )
}
