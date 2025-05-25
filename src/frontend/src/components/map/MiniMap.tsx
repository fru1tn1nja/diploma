'use client'
import { useEffect, useMemo, useRef, useState } from 'react'
import dynamic from 'next/dynamic'
import * as L from 'leaflet'
import { useTelemetry } from '@/hooks/useTelemetry'
import mqtt, { MqttClient } from 'mqtt'

// динамический импорт react-leaflet компонентов, без SSR
const MapContainer = dynamic(
  () => import('react-leaflet').then(m => m.MapContainer),
  { ssr: false }
)
const Marker = dynamic(
  () => import('react-leaflet').then(m => m.Marker),
  { ssr: false }
)
const Polyline = dynamic(
  () => import('react-leaflet').then(m => m.Polyline),
  { ssr: false }
)
const Circle = dynamic(
  () => import('react-leaflet').then(m => m.Circle),
  { ssr: false }
)

// статические объекты препятствий

export default function MiniMap() {
  const { connect, packet, buffer, mission, obstacles } = useTelemetry()
  const [clickGoal, setClickGoal] = useState<[number, number] | null>(null)
  useEffect(() => {
    console.log("⚡ Received obstacles:", obstacles)
  }, [obstacles])
  
  const mapRef = useRef<L.Map | null>(null)
  const mqttRef = useRef<MqttClient | null>(null)
  const deviceId = 1  // или заберите из конфигурации
  useEffect(() => {
    connect()
  }, [connect])
  // 1) Подключаемся к EMQX по WebSocket для публикации команд миссии
  useEffect(() => {
    const host = process.env.NEXT_PUBLIC_MQTT_WS_HOST ?? window.location.hostname
    const port = process.env.NEXT_PUBLIC_MQTT_WS_PORT ?? '8083'
    const url = `ws://${host}:${port}/mqtt`
    const client = mqtt.connect(url)
    client.on('connect', () => console.log('[MQTT] connected via WS'))
    client.on('error', err => console.error('[MQTT]', err))
    mqttRef.current = client
    return () => {
      client.end()
    }
  }, [])

  // 2) Хендлер двойного клика: формируем JSON и шлём в MQTT + сразу рисуем локальный clickGoal
  const handleDblclick = (e: L.LeafletMouseEvent) => {
    const { lat, lng } = e.latlng
    console.log('▶ dblclick → send mission:', lat, lng)
    const msg = {
      waypoints: [[lat, lng]],
      goal: [lat, lng],
    }
    const topic = `mission/commands/${deviceId}`
    mqttRef.current?.publish(
      topic,
      JSON.stringify(msg),
      { qos: 0 },
      err => {
        if (err) console.error('[MQTT pub]', err)
        else console.log('[MQTT pub] sent to', topic)
      }
    )
    // мгновенно показываем звёздочку по клику
    setClickGoal([lat, lng])
  }

  // текущее положение лодки
  const pos = useMemo<[number, number] | null>(
    () => (packet ? [packet.lat, packet.lon] : null),
    [packet?.lat, packet?.lon]
  )
  const yaw = packet?.yaw ?? 0

  // скользящее окно трека
  const track = useMemo<[number, number][]>(
    () => buffer.slice(-300).map(p => [p.lat, p.lon]),
    [buffer]
  )

  // лодка как раньше
  const vehicleIcon = useMemo(
    () =>
      L.divIcon({
        iconSize: [11, 11],
        iconAnchor: [5, 11],
        className: '',
        html: `<div style="
          width:11px;height:11px;
          background:#e11d48;
          clip-path:polygon(50% 0,0 100%,100% 100%);
          transform:rotate(${yaw}rad);
          transform-origin:50% 100%;
        "></div>`,
      }),
    [yaw]
  )

  // иконка цели
  const goalIcon = useMemo(
    () =>
      L.divIcon({
        iconSize: [16, 16],
        iconAnchor: [11, 11],
        className: '',
        html: `<div style="
          width:16px;height:16px;
          background:gold;
          clip-path:polygon(
            50% 0%,61% 35%,98% 35%,68% 57%,79% 91%,
            50% 70%,21% 91%,32% 57%,2% 35%,39% 35%
          );
        "></div>`,
      }),
    []
  )

  // центрируем карту под лодкой (пока нет локальной clickGoal)
  useEffect(() => {
    const map = mapRef.current
    if (map && pos) {
      if (!clickGoal) {
        map.setView(pos, map.getZoom(), { animate: false })
      }
    }
  }, [pos, clickGoal])

  // вешаем dblclick-обработчик и отключаем зум по двойному
  useEffect(() => {
    const map = mapRef.current
    if (!map) return
    map.doubleClickZoom.disable()
    map.on('dblclick', handleDblclick)
    return () => {
      map.off('dblclick', handleDblclick)
      map.doubleClickZoom.enable()
    }
  }, [mapRef.current])

  if (!pos) {
    return <p className="text-gray-500">loading…</p>
  }

  return (
    <MapContainer
      ref={mapRef}
      whenReady={() => {}}
      attributionControl={false}
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
      maxBounds={[[-60, -60], [60, 60]]}
      dragging={false}
      zoomControl={false}
      doubleClickZoom={false}
    >

      {/* рисуем трек */}
      <Polyline positions={track} color="red" weight={2} />
      {obstacles.map(([x,y,r], i) => (
        <Circle
          key={i}
          center={[x,y]}
          radius={r}
          pathOptions={{ fillOpacity: 0.4 }}
        />
      ))}
      {/* 🚩 Маркер цели: сначала локальный клик, иначе mission.goal */}
      {(clickGoal || mission?.goal) && (
        <Marker
          position={clickGoal ?? mission!.goal}
          icon={goalIcon}
          title="Target"
        />
      )}

      {/* 🔺 Лодка */}
      <Marker key={`boat-${yaw.toFixed(2)}`} position={pos} icon={vehicleIcon} />
    </MapContainer>
  )
}
