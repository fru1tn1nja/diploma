'use client'
import dynamic from 'next/dynamic'
import {useTelemetry} from '@/hooks/useTelemetry'

// dynamic импорт, чтобы отключить SSR (Mapbox не работает на сервере)
const Map = dynamic(()=>import('react-map-gl'), { ssr:false })

export default function MapView() {
  const p = useTelemetry(state => state.packet)
  if(!p) return <p>waiting telemetry…</p>

  return (
    <Map
      mapStyle="mapbox://styles/mapbox/streets-v12"
      mapboxAccessToken={process.env.NEXT_PUBLIC_MAPBOX_TOKEN}
      initialViewState={{ longitude:p.lon, latitude:p.lat, zoom:15 }}
      style={{width:'100%', height:'400px'}}
    >
      {/* маркер */}
      <div
        style={{
          position:'absolute', left:'50%', top:'50%',
          width:12, height:12, background:'#d00', borderRadius:'50%',
          transform:'translate(-50%,-50%)'
        }}
      />
    </Map>
  )
}
