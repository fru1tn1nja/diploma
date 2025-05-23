// src/components/ui/ModeSwitch.tsx
'use client'
import ROSLIB from 'roslib'
import { useTelemetry } from '@/hooks/useTelemetry'

const ros = new ROSLIB.Ros({ url: 'ws://localhost:9090' })   // rosbridge-server
const cmdTopic = new ROSLIB.Topic({
  ros,
  name: '/operator/commands',
  messageType: 'std_msgs/String',
})

export default function ModeSwitch() {
  const mode = useTelemetry((s) => s.mode)      // 'Manual' | 'AI'

  function toggle() {
    const next = mode === 'AI' ? 'TAKE_OVER' : 'RESUME_AI'
    cmdTopic.publish(new ROSLIB.Message({ data: next }))
  }

  return (
    <button
      onClick={toggle}
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