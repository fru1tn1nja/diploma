'use client'
import { Canvas } from '@react-three/fiber'
import { OrbitControls, useGLTF } from '@react-three/drei'
import {useTelemetry} from '@/hooks/useTelemetry'

function DroneModel() {
  // glTF квадрокоптера (400-кБ Iris preview). Положите файл в public/models/iris.glb
  const { scene } = useGLTF('/models/iris.glb')
  return <primitive object={scene} scale={1.5} />
}
export default function DroneMini3D() {
  const pkt = useTelemetry(s=>s.packet)
  // безопасно рендерим только если пришла телеметрия
  return (
    <div className="h-64 w-full rounded-xl shadow-inner bg-white">
      <Canvas camera={{ position:[2,2,2], fov:45 }}>
        <ambientLight intensity={0.8}/>
        <DroneModel />
        {/* Угол yaw для ориентации модели */}
        {pkt && <group rotation={[0, 0, -pkt.vel/10]} />}
        <OrbitControls enablePan={false} enableZoom={false} />
      </Canvas>
    </div>
  )
}