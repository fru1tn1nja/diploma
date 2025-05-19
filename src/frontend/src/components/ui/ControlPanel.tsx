'use client'
import {post} from '@/lib/api'

export default function ControlPanel() {
  return (
    <div className="flex gap-2 mt-4">
      <button className="btn" onClick={()=>post('/api/control/take-over')}>
        Take Over
      </button>
      <button className="btn" onClick={()=>post('/api/control/resume-ai')}>
        Resume AI
      </button>
    </div>
  )
}