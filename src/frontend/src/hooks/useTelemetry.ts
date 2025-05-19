import create from 'zustand'

type Telemetry = {
  lat:number; lon:number; vel:number; battery:number; ts:number
}

interface Store {
  packet?: Telemetry
  buffer: Telemetry[]
  connect: () => void
}

export const useTelemetry = create<Store>((set) => ({
  packet: undefined,
  buffer: [],
  connect: () => {
    const url = `ws://${process.env.NEXT_PUBLIC_GATEWAY}/ws/telemetry/1`
    const ws  = new WebSocket(url)
    ws.onmessage = (e) => {
      const p = JSON.parse(e.data) as Telemetry
      set((s)=>({ packet: p, buffer: [...s.buffer.slice(-200), p] }))
    }
  }
}))