// src/components/ui/ModeSwitch.tsx
'use client'

import { useTelemetry } from '@/hooks/useTelemetry'
import { useEffect, useRef } from 'react'
import ROSLIB from 'roslib'

export default function ModeSwitch() {
  const mode = useTelemetry((s) => s.mode)   // 'Manual' | 'AI'
  const topicRef = useRef<ROSLIB.Topic | null>(null)
  const rosRef = useRef<ROSLIB.Ros | null>(null)

  useEffect(() => {
    if (typeof window === 'undefined') return

    // Формируем URL rosbridge
    const rosbridgeUrl = process.env.NEXT_PUBLIC_ROSBRIDGE_URL
      ? `ws://${process.env.NEXT_PUBLIC_ROSBRIDGE_URL}`
      : `ws://${window.location.hostname}:9090`

    // Создаём подключение
    
    const ros = new ROSLIB.Ros({ url: rosbridgeUrl })
    rosRef.current = ros

    const cmdTopic = new ROSLIB.Topic({
      ros,
      name: '/operator/commands',
      messageType: 'std_msgs/msg/String',
    })

    ros.on('connection', () => {console.log('[ModeSwitch] rosbridge connected'); cmdTopic.advertise();});
    ros.on('error',    (err) => console.error('[ModeSwitch] rosbridge error', err))
    ros.on('close',    ()    => console.warn('[ModeSwitch] rosbridge closed'))

    // Создаём и рекламируем топик сразу
    
    console.log('[ModeSwitch] advertising topic /operator/commands')
    topicRef.current = cmdTopic

    return () => {
      if (topicRef.current) topicRef.current.unadvertise()
      if (rosRef.current) rosRef.current.close()
    }
  }, [])

  const publishMode = (data: string) => {
    const t = topicRef.current
    if (t) {
      console.log(`► [ModeSwitch] publish ${data}`)
      t.publish(new ROSLIB.Message({ data }))
    }
  }

  return (
    <button
      onClick={() => publishMode(mode === 'AI' ? 'TAKE_OVER' : 'RESUME_AI')}
      className={
        'px-3 py-1 rounded font-semibold ' +
        (mode === 'AI'
          ? 'bg-green-500 text-white hover:bg-green-600'
          : 'bg-amber-400 text-black hover:bg-amber-500')
      }
    >
      {mode === 'AI' ? '⇨ Manual' : '⇨ AI'}
    </button>
  )
}
