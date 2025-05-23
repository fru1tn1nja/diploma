import './globals.css'
import type { Metadata } from 'next'
import 'leaflet/dist/leaflet.css';

export const metadata: Metadata = {
  title: 'Shared-Control Dashboard',
  description: 'Live UAV/USV monitoring',
}

/** Общий каркас Next.js (App Router) */
export default function RootLayout({
  children,
}: {
  children: React.ReactNode
}) {
  return (
    <html lang="en">
      <head><link
  rel="stylesheet"
  href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"
  integrity="sha256-o9N1jIdep5Sp4hviqt+4ZrK7kRJT5NDM/ArY5hLDk0w="
  crossOrigin=""
/></head>
      <body className="min-h-screen bg-slate-100 text-gray-900">
        {children}
      </body>
    </html>
  )
}
