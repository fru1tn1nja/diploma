/* eslint-disable @typescript-eslint/no-explicit-any */
import create from 'zustand'

type Mission = { waypoints: [number,number][], goal: [number,number] }

/* ---------- тип одного пакета телеметрии ---------- */
export type Telemetry = {
  lat: number
  lon: number
  vel: number
  battery: number
  yaw?: number          // опционально — курс, если симулятор шлёт
  ts: number            // epoch-ms
}

/* ---------- Zustand-store ---------- */
interface Store {
  packet?: Telemetry            // последний пакет
  buffer:  Telemetry[]          // скользящее окно точек
  mission?: Mission
  mode: 'Manual' | 'AI'         // текущий режим, приходит из /ws/mode
  connect: () => void           // запускает WS-подписки (вызываем один раз)
}

export const useTelemetry = create<Store>((set, get) => ({
  packet: undefined,
  buffer: [],
  mode: 'Manual',

  connect: () => {
    if (typeof window === 'undefined') return  // SSR-фаза

    /* чтобы не открывать WS дважды */
    if ((get() as any)._connected) return
    ;(get() as any)._connected = true

    const gw = process.env.NEXT_PUBLIC_GATEWAY ??
               `${window.location.hostname}:8000`

    /* ---------- WebSocket 1: поток телеметрии ---------- */
    const wsTelem = new WebSocket(`ws://${gw}/ws/telemetry/1`)
    wsTelem.onmessage = (e) => {
      try {
        const p: Telemetry = JSON.parse(e.data)
        set((s) => ({
          packet: p,
          buffer: [...s.buffer.slice(-500), p],   // храним ≤ 500 точек
        }))
      } catch { /* ignore bad JSON */ }
    }
    wsTelem.onclose = () => {
      set({ packet: undefined })
      ;(get() as any)._connected = false
    }

    /* ---------- WebSocket 2: режим Manual / AI ---------- */
    const wsMode = new WebSocket(`ws://${gw}/ws/mode`)
    wsMode.onmessage = (e) => {
      const m = e.data === 'AI' ? 'AI' : 'Manual'
      set({ mode: m })
    }
    
    const wsMission = new WebSocket(`ws://${gw}/ws/mission`)
    wsMission.onmessage = (e)=>{
      try{ set({mission: JSON.parse(e.data)}) }catch{}
    }

    wsMode.onclose = () => set({ mode: 'Manual' })   // фолбэк
  },
}))