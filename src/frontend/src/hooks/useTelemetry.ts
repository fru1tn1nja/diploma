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
export interface Store {
  packet?: Telemetry            // последний пакет
  buffer:  Telemetry[]          // скользящее окно точек
  mission?: Mission
  mode: 'Manual' | 'AI'         // текущий режим, приходит из /ws/mode
  obstacles: [number,number,number][]
  connect: () => void           // запускает WS-подписки (вызываем один раз)
}

export const useTelemetry = create<Store>((set, get) => ({
  packet: undefined,
  buffer: [],
  mode: 'Manual',
  obstacles: [],
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
      // сервер шлёт строку-код: '0' = AI, '1' = Manual
      const code = e.data
      const m: 'AI' | 'Manual' = code === '0' ? 'AI' : 'Manual'
      set({ mode: m })
    }
    wsMode.onclose = () => set({ mode: 'Manual' })   // фолбэк

    /* ---------- WebSocket 3: миссия ---------- */
    const wsMission = new WebSocket(`ws://${gw}/ws/mission/1`)
    wsMission.onmessage = (e) => {
      console.log("🛰 Mission raw payload:", e.data)
      try {
        set({ mission: JSON.parse(e.data) })
      } catch (err) {
        console.warn("Bad JSON in mission:", err)
      }
    }

    const wsObs = new WebSocket(`ws://${gw}/ws/obstacles/1`)
    wsObs.onmessage = e => {
      try {
        const { obstacles } = JSON.parse(e.data) as { obstacles: [number,number,number][] }
        set({ obstacles })
      } catch {}
    }
  },
}))
